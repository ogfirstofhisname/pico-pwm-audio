# MicroPython

from machine import Pin, PWM, mem32, ADC # type: ignore[import]
import rp2, time # type: ignore[import]
import struct
from array import array
import math
import uctypes # type: ignore[import]
# from micropython import const
'''
This script for the Raspberry Pi Pico 2 (RP2350) plays a 16-bit WAV file using PIO to generate PWM signals that drive a speaker directly.
It uses a PIO state machine to implement a PWM audio driver that reads 12-bit duty cycle values from a FIFO and outputs them to a GPIO pin.
The PWM, driving the inductive load of the speaker, produces a pseudo-analog signal.

The PIO state machine runs at 150 MHz.
#TODO continue this line
#! wallack
'''


'''
The fifo_join can be set to JOIN_TX to double the size of the state machine's FIFO (from 4 to 8 words).
This would allow the FIFO to cover more DMA dead time, i.e. DMA configuration commands.
The DMA configuration in MicroPython takes 50-100usec, so any FIFO size of 2 or more words is enough.
'''

from pio import pwm_audio_driver  # import the PIO program for the audio driver

LED_PIN_NUM = 25  # built-in LED on GPIO25
TEMP_SENSOR_ADC_NUM = 4  # ADC4 is the temperature sensor on the RP2350


class DMAWavPlayer:
    '''
    A class to play WAV files using DMA and PIO on the Raspberry Pi Pico 2 (RP2350).
    The class uses a PIO state machine to generate PWM signals that GPIO pins, that can be used do directly drive a speaker, headphones,
    or any other inductive load.
    The audio samples are read from a WAV file, converted to 12-bit unsigned samples, and streamed to the PIO state machine via DMA.
    The class supports stereo and mono WAV files with 16 bits per sample and a sample rate around 18.295 kHz.

    The class keeps the state machine object, the DMA object and all the necessary buffers internally.

    Arguments:
    - wav_file_path: Path to the WAV file to play. If None, the file must be provided later via the play_wav() method.
    - loop: If True, the playback will loop indefinitely. If False, the playback will stop after the file is played once. Can be set later via the play_wav() method.
    - buffer_n_samples: Number of samples in the buffer. Default is 1024 samples.
    - noninverting_gpio_n: GPIO pin number base for the non-inverting output pins (default is 0). This pin is the first of 5 consecutive GPIO pins driving the
    non-inverting output of the PWM audio driver.
    - inverting_gpio_n: GPIO pin number base for the inverting output pins (default is 5). This pin is the first of 5 consecutive GPIO pins driving the
    inverting output of the PWM audio driver. The inverting output is used to create a differential signal for better sound quality.
    '''
    def __init__(self, wav_file_path:str=None, loop:bool=False, buffer_n_samples:int=1024, noninverting_gpio_n:int=0, inverting_gpio_n:int=5):
        self._sanitize_gpio_nums(noninverting_gpio_n, inverting_gpio_n)
        self.noninverting_gpio_n = noninverting_gpio_n  # GPIO pin number base for the non-inverting output pins
        self.inverting_gpio_n = inverting_gpio_n # GPIO pin number base for the inverting output pins
        self.sm = self._init_audio_driver_sm() # uses self.noninverting_gpio_n and self.inverting_gpio_n to set the base pins for the state machine
        self._sanitize_buffer_n_samples(buffer_n_samples)
        self.buffer_n_samples = buffer_n_samples
        self.loop = loop
        self.file_buffer = None  # buffer for raw samples from the WAV file
        self.wav_file_path = None
        self.wav_file_obj = None  # file object for the WAV file
        self.wav_file_info = None  # dictionary to hold WAV file info (num_channels, sample_rate, bits_per_sample, data_offset)
        self.temp_sensor_adc = ADC(TEMP_SENSOR_ADC_NUM)  # create an ADC object for the temperature sensor
        self.led_pin = Pin(LED_PIN_NUM, Pin.OUT, value=0)  # built-in LED on GPIO25
        if wav_file_path is not None:
            self._process_wav_file(wav_file_path) # checks for sample rate, bits per sample, and number of channels. set up the file buffer
        self.pio_samples_buffer = array('H', [0] * buffer_n_samples)  # buffer for processed 12-bit unsigned samples
        # self.third_buffer = array('H', [0] * (buffer_n_samples * 2))  # double-sized buffer for DMA transfer in a ping-pong manner
        self.third_buffer = self._allocate_third_buffer()  # allocate the third buffer for DMA transfer
        self._set_up_dma()
        self.offset = None
        self._set_pin_drive_strength()  # set the drive strength for the GPIO pins used by the PIO state machine

    def _allocate_third_buffer(self):
        return array('H', [0] * (self.buffer_n_samples * 2))

    def _set_up_dma(self):
        dma = rp2.DMA()
        # DMA.config(read=None, write=None, count=None, ctrl=None, trigger=False)
        DREQ_SM0_TX = (0 << 3) | 0 # Section 12.6.4.1. of the RP2350 datasheet
        ctrl_value = dma.pack_ctrl(size=1, inc_write=False, treq_sel=DREQ_SM0_TX)
        self.dma = dma
        self.dma_ctrl_value = ctrl_value  # store the DMA control value for later use

    def _sanitize_buffer_n_samples(self, buffer_n_samples):
        if not (1 <= buffer_n_samples <= 16384):
            raise ValueError(f"Buffer size should be <= 16384 samples, got {buffer_n_samples} samples.")
        if buffer_n_samples % 2 != 0:
            raise ValueError(f"Buffer size must be an even number of samples for stereo playback, got {buffer_n_samples} samples.")
        if buffer_n_samples < 64:
            print(f"Warning: buffer size is {buffer_n_samples} samples, which is quite small. Consider increasing it for guaranteed performance.")

    def _sanitize_gpio_nums(self, noninverting_gpio_n, inverting_gpio_n):
        # make sure that both numbers are non-negative, no greater than 21, and are at least 5 apart
        if not (0 <= noninverting_gpio_n <= 21 and 0 <= inverting_gpio_n <= 21):
            raise ValueError("GPIO pin numbers must be between 0 and 21 inclusive.")
        if abs(noninverting_gpio_n - inverting_gpio_n) < 5:
            raise ValueError("GPIO pin numbers must be at least 5 apart to avoid conflicts.")
        
    def _init_audio_driver_sm(self):
        # Initialize the audio driver state machine with the given GPIO pin numbers
        sm = rp2.StateMachine(
            0,
            pwm_audio_driver,
            freq=150_000_000,  # 150 MHz clock frequency
            set_base=Pin(self.noninverting_gpio_n),  # base pin for non-inverting output
            sideset_base=Pin(self.inverting_gpio_n)  # base pin for inverting output
        )
        if sm.tx_fifo() == 0:
            sm.put(4095)

        return sm
    
    def _process_wav_file(self, wav_file_path):
        file_info = self.parse_wav_header(wav_file_path)
        # check that 16 bits per sample, 1 or 2 channels, and sample rate is 18.295 kHz up to +- 10%
        if file_info['bits_per_sample'] != 16:
            raise ValueError(f"WAV file must have 16 bits per sample, got {file_info['bits_per_sample']} bits per sample.")
        if file_info['num_channels'] not in (1, 2):
            raise ValueError(f"WAV file must have 1 or 2 channels, got {file_info['num_channels']} channels.")
        if not (0.9 * 18295 <= file_info['sample_rate'] <= 1.1 * 18295):
            raise ValueError(f"WAV file sample rate must be around 18.295 kHz, got {file_info['sample_rate']} Hz.")
        self.wav_file_path = wav_file_path  # store the WAV file path
        self.wav_file_info = file_info  # store the WAV file info
        self.file_buffer = array('h', [0] * self.buffer_n_samples * file_info['num_channels'])  # signed 16-bit samples, stereo or mono

    def parse_wav_header(self, wav_file_path):
        wav_file = open(wav_file_path, 'rb')
        def read_chunk_header(f):
            return f.read(4), struct.unpack("<I", f.read(4))[0]

        riff = wav_file.read(4)
        if riff != b"RIFF":
            raise ValueError("Not a valid WAV file: missing RIFF header")

        _ = wav_file.read(4)  # skip size
        wave = wav_file.read(4)
        if wave != b"WAVE":
            raise ValueError("Not a valid WAV file: missing WAVE header")

        fmt_chunk_found = False
        data_chunk_offset = None

        while True:
            chunk_id, chunk_size = read_chunk_header(wav_file)
            if chunk_id == b"fmt ":
                fmt_chunk_found = True
                audio_format = struct.unpack("<H", wav_file.read(2))[0]
                num_channels = struct.unpack("<H", wav_file.read(2))[0]
                sample_rate = struct.unpack("<I", wav_file.read(4))[0]
                byte_rate = struct.unpack("<I", wav_file.read(4))[0]
                block_align = struct.unpack("<H", wav_file.read(2))[0]
                bits_per_sample = struct.unpack("<H", wav_file.read(2))[0]
                wav_file.seek(chunk_size - 16, 1)
            elif chunk_id == b"data":
                data_chunk_offset = wav_file.tell()
                break
            else:
                wav_file.seek(chunk_size, 1)

        if not fmt_chunk_found or data_chunk_offset is None:
            raise ValueError("Missing required WAV chunks")

        wav_file.close()

        return {
            "num_channels": num_channels,
            "sample_rate": sample_rate,
            "bits_per_sample": bits_per_sample,
            "data_offset": data_chunk_offset
        }

    def read_temperature(self) -> float:
        '''
        Reads the temperature from the internal temperature sensor ADC.
        Returns the temperature in degrees Celsius.

        Returns:
        float: Temperature in degrees Celsius.
        '''
        time.sleep_ms(100)
        reading = self.temp_sensor_adc.read_u16() * 3.3 / 65535.0
        temperature = 27 - (reading - 0.706) / 0.001721  # convert to Celsius using the formula from the datasheet
        time.sleep_ms(100)  # wait for a short time to avoid reading too fast
        return temperature

    def _wait_for_dma_to_clear_first_half(self):
        while self.dma.count > self.buffer_n_samples:
            pass

    def _wait_for_dma_to_clear_second_half(self):
        while self.dma.count > 0:
            pass

    def play_wav(self, wav_file_path:str=None, loop:bool=None) -> None:
        '''
        Plays a WAV file using DMA and PIO on the Raspberry Pi Pico 2 (RP2350).
        Playback starts at the start of the WAV file, and the file is read in chunks until the end of the file is reached.
        Returns when playback is complete. If loop is True, it will loop the playback indefinitely and never return.

        Arguments:
        - wav_file_path: Path to the WAV file to play. If None, the file must be provided in the constructor.
        - loop: If True, the playback will loop indefinitely. If False, the playback will stop after the file is played once. If loop is not provided, the value from the constructor will be used.
        
        '''
        if loop is not None:
            self.loop = loop
        if wav_file_path is not None:
            self._process_wav_file(wav_file_path)  # process the WAV file if a new path is provided
        if self.wav_file_path is None:
            raise ValueError("No WAV file specified for playback. Provide a valid WAV file path, either in the constructor or in the play_wav() method.")
        self.wav_file_obj = open(self.wav_file_path, 'rb')  # open the WAV file for reading
        self.sm.active(1)  # start the audio driver state machine
        while True: # playback loop (every iteration is a full file playback)
            got_bytes = self._preload_buffers() # assumes the file is already opened and ready to read
            # now, the first half of the third buffer is pre-filled with samples, and the second half is empty.
            # the preceding pio_samples_buffer is pre-loaded with the next sample, and the file_buffer is pre-filled with the second-next samples.
            self.led_pin.on()  # turn on the built-in LED to indicate playback start

            while got_bytes > 0: # playback chunks loop
                self._config_dma_transfer()
                # fill the second half of the third buffer with the next samples. The DMA is still going through the first half.
                self.third_buffer[self.buffer_n_samples:] = self.pio_samples_buffer[:]
                # back-refill the preceding buffers
                self._convert_buffer_to_pio_samples()
                got_bytes = self._fill_buffer_from_wav()
                self.offset += got_bytes  # update offset for the next read
                # now the second half of the third buffer is filled with samples, waiting for the DMA to reach it.
                # wait for the DMA transfer to complete the first half
                # while self.dma.count > self.buffer_n_samples:
                #     pass
                self._wait_for_dma_to_clear_first_half()
                # now we can refill the first half of the third buffer with the next samples
                self.third_buffer[:self.buffer_n_samples] = self.pio_samples_buffer[:]
                # back-refill the preceding buffers
                self._convert_buffer_to_pio_samples()
                got_bytes = self._fill_buffer_from_wav()
                self.offset += got_bytes  # update offset for the next read
                # wait for the DMA transfer to complete
                # while self.dma.count > 0:
                #     pass
                self._wait_for_dma_to_clear_second_half()
            self.led_pin.off()  # turn off the built-in LED to indicate playback end or pause
            if not self.loop:
                break
            else:
                time.sleep_ms(150) # wait for a short time before restarting playback
            # if loop is True, continue playback indefinitely
        self._abort_dma_transfer_run()  # stop the DMA transfer
        self.sm.active(0)  # stop the audio driver state machine
        self.sm.exec('set(pins, 0b00000).side(0b00000)')  # set all pins low to stop the audio output
        self.wav_file_obj.close()  # close the WAV file when done

    def _abort_dma_transfer_run(self):
        self.dma.count = 0

    def _config_dma_transfer(self):
        # Configure the DMA transfer for the audio driver state machine
        self.dma.config(
            read=self.third_buffer,
            write=self.sm,
            count=len(self.third_buffer),
            ctrl=self.dma_ctrl_value,
            trigger=True
        )
    def _set_drive_single_pin(self, pin_num, mA):
        PADS_BANK0 = 0x40038000                     # base of pad-control block
        DRIVE_BITS = {2:0, 4:1, 8:2, 12:3}[mA]    # DRIVE field is bits 5-4
        addr = PADS_BANK0 + 4*(pin_num+1)                 # one 32-bit register per pin
        # read-modify-write the register to set the drive strength. Bits 5:4 are the DRIVE bits
        mem32[addr] = (mem32[addr] & ~0b110000) | (DRIVE_BITS << 4)

    def _set_pin_drive_strength(self):
        # set drive strength for the non-inverting and inverting GPIO pins
        for pin_num in range(self.noninverting_gpio_n, self.inverting_gpio_n + 5):
            self._set_drive_single_pin(pin_num, 12)

    def _preload_buffers(self):
        # fill the first half of the third buffer, then refill the pio_samples_buffer and the file_buffer
        self.offset = self.wav_file_info['data_offset']  # reset offset to the beginning of the data chunk
        if self.wav_file_obj is None:
            raise ValueError("WAV file is not opened. Call play_wav() to open and prepare the file for playback.")
        else:
            # move the file pointer to the offset
            self.wav_file_obj.seek(self.offset)
        got_bytes = self._fill_buffer_from_wav()
        self._convert_buffer_to_pio_samples()
        self.third_buffer[:self.buffer_n_samples] = self.pio_samples_buffer[:]  # copy the first half of the pio_samples_buffer to the third buffer
        # refill the preceding buffers
        self.offset += got_bytes  # update offset for the next read
        got_bytes = self._fill_buffer_from_wav()
        self._convert_buffer_to_pio_samples()
        self.offset += got_bytes
        got_bytes = self._fill_buffer_from_wav()
        self.offset += got_bytes
        return got_bytes  # return the number of bytes read into the file_buffer

    def _fill_buffer_from_wav(self):
        # fill the file_buffer with raw 16-bit samples from the WAV file
        if self.wav_file_obj is None:
            raise ValueError("WAV file is not opened. Call play_wav() to open and prepare the file for playback.")
        got_bytes = self.wav_file_obj.readinto(self.file_buffer)
        
        return got_bytes  # return the number of bytes read into the file_buffer
    
    def _convert_buffer_to_pio_samples(self):
        # Convert the raw 16-bit signed samples to 12-bit unsigned samples
        # iterate in steps as needed, since we may have stereo samples and we only need the left channel
        for i in range(len(self.pio_samples_buffer)):
            self.pio_samples_buffer[i] = (self.file_buffer[i * self.wav_file_info['num_channels']] + 32768) >> 4



    '''
    Much faster (~20x) convert function using viper. Not needed in this case since hardware buffering easily covers the conversion time.
    '''
    # _OFFSET = micropython.const(32768)
    # _SHIFT  = micropython.const(4)
    # @micropython.viper
    # def convert_buffer_to_pio_samples_viper(raw_buf, pio_buf, channels_in_wav):
    #     # Convert the raw 16-bit signed samples to 12-bit unsigned samples using viper
    #     src = ptr16(raw_buf)          # 16-bit signed buffer # type: ignore[no-redef]
    #     dst = ptr16(pio_buf)          # 12-bit unsigned buffer # type: ignore[no-redef]
    #     n   = int(len(pio_buf))
    #     for i in range(n):
    #         dst[i] = (int(src[i * channels_in_wav]) + _OFFSET) >> _SHIFT


class RingDMAWavPlayer(DMAWavPlayer):
    '''
    A subclass of DMAWavPlayer that configures the DMA transfer to a wrap-around its source buffer,
    saving the need to re-configure the DMA transfer run every time.
    !!! IMPORTANT: 
    This class DOES NOT WORK.
    The implementation is meant to be based on the DMA's wrap-around feature. This feature, however,
    works by masking the first n bits of the address, which means that the buffer size AND BASE ADDRESS
    must be a power of two.
    This is not the case for an array normally allocated in MicroPython.
    A workaround is not yet implemented, so the class is not functional.
    '''

    def _allocate_third_buffer(self):
        '''
        The implementation here will be based on:
        - allocating an excessively large buffer - twice the size of the required ping-pong buffer
        - get that buffer's base address in memory
        - use memoryview to slice a buffer of the required size with an aligned base address
        - keep a reference to the original buffer to avoid garbage collection
        '''
        # return array('H', [0] * (self.buffer_n_samples * 2))
        # allocate a large buffer, twice the size of the required ping-pong buffer
        # this buffer will remain in memory to avoid garbage collection
        '''
        We use buffer size of buffer_n_samples (number of samples). After sample conversion, we only have one audio channel at 16 bits per sample.
        So, each half of the ping pong buffer needs to be filled with buffer_n_samples (number of samples).
        So, the entire ping-pong buffer size is buffer_n_samples * 2 (two halves) (number of samples).
        The excessively large buffer should be twice the size of the ping-pong buffer, minus one sample, to guarantee that
        an offset can be found that aligns the base address.
        Therefore, we allocate an array of size buffer_n_samples * 4 - 1.
        '''
        self._large_buffer = array('H', [0] * (self.buffer_n_samples * 4 - 1))  # four times the size for safety
        # get the base address of the large buffer using uctypes
        base_address = uctypes.addressof(self._large_buffer)
        # calculate a start address for the third buffer that is aligned to the buffer_n_samples * 2
        '''
        As we mentioned, the third buffer should be buffer_n_samples * 2 samples in size.
        Since each sample is 2 bytes, the size in bytes is buffer_n_samples * 2 * 2 = buffer_n_samples * 4.
        '''
        align = self.buffer_n_samples * 4  # alignment size in bytes
        start = (base_address + align - 1) & ~(align - 1)  # advance by buffer_n_samples * 2 -1 and then zero out the lower bits
        # # calculate an offset to align the base address to self.buffer_n_samples * 2
        # offset = (-base_address) & (self.buffer_n_samples * 2 - 1)
        offset = (start - base_address)  # calculate the offset to align the base address
        offset_items = offset // 2  # offset in items, since the array is of type 'H' (16-bit unsigned integers)
        self._third_buffer_start_address = base_address + offset  # aligned start address of the third buffer
        self._third_buffer_size_items = self.buffer_n_samples * 2  # size in samples, or memoryview items for a memoryview produced from the 'H' array
        self._third_buffer_size_bytes = self.buffer_n_samples * 4
        self._third_buffer_end_address = self._third_buffer_start_address + (self._third_buffer_size_bytes)
        # create a memoryview of the large buffer and slice it to the required size
        self._whole_mv = memoryview(self._large_buffer)
        # aligned_mv = self._whole_mv[offset:offset + self._third_buffer_size_bytes]
        aligned_mv = self._whole_mv[offset_items:offset_items + self._third_buffer_size_items]
        # print(f'large buffer size in samples: {len(self._large_buffer)}, align: {align}, offset: {offset}, start: {hex(start)}, aligned start address: {hex(self._third_buffer_start_address)}, end address: {hex(self._third_buffer_end_address)}')
        # print(f'Aligned third buffer address: {hex(uctypes.addressof(aligned_mv))}, size: {len(aligned_mv)} samples ({len(aligned_mv) * 2} bytes).')
        # return the aligned memoryview
        return aligned_mv
    
    def _wait_for_dma_to_clear_first_half(self):
        # wait for the DMA to clear the first half of the third buffer
        while self.dma.read < self._third_buffer_start_address + self._third_buffer_size_bytes // 2:
            # print(f'not yet passed the halfway point: started this first half at {hex(self._third_buffer_start_address)}, now at {hex(self.dma.read)}, halfway at {hex(self._third_buffer_start_address + self._third_buffer_size_bytes // 2)}, end at {hex(self._third_buffer_start_address + self._third_buffer_size_bytes)}')
            pass

    def _wait_for_dma_to_clear_second_half(self):
        while self.dma.read > self._third_buffer_start_address + self._third_buffer_size_bytes // 2:
            # print(f'not yet passed the end point: started this second half at {hex(self._third_buffer_start_address + self._third_buffer_size_bytes // 2)}, now at {hex(self.dma.read)}, end at {hex(self._third_buffer_start_address + self._third_buffer_size_bytes)}')
            pass




    def _set_up_dma(self):
        dma = rp2.DMA()
        # DMA.config(read=None, write=None, count=None, ctrl=None, trigger=False)
        DREQ_SM0_TX = (0 << 3) | 0 # Section 12.6.4.1. of the RP2350 datasheet
        # ring_size = (len(self.third_buffer) * 2).bit_length()  # size in bits of the ring buffer address space
        # ring_size = int(math.log2(self._third_buffer_size_bytes))  # size in bits of the ring buffer address space
        # int seems to suffer from precision issues, use round() instead
        ring_size = round(math.log2(self._third_buffer_size_bytes))  # size in bits of the ring buffer address space
        ctrl_value = dma.pack_ctrl(
            size=1,
            inc_read=True,
            inc_write=False,
            ring_size=ring_size, # size in bits of the ring buffer address space
            ring_sel=False, # applies wrap-around to the read address
            treq_sel=DREQ_SM0_TX, # trigger transfers on the state machine's TX FIFO
        )
        self.dma = dma
        self.dma_ctrl_value = ctrl_value
        self.dma_running = False

    def _config_dma_transfer(self):
        if self.dma_running:
            # print("DMA transfer is already configured and running. No need to re-configure.")
            return
        else:
            # Configure the DMA transfer for the audio driver state machine
            self.dma.config(
                read=self.third_buffer,
                write=self.sm,
                count=len(self.third_buffer)*400,
                ctrl=self.dma_ctrl_value,
                trigger=True
            )
            self.dma_running = True  # mark the DMA as running

    class ContinuousDMAWavPlayer(DMAWavPlayer):
        '''
        A subclass of DMAWavPlayer that configures the DMA transfer to run continuously,
        without the need to re-configure the DMA transfer every time.
        This is achieved by utilizing two (2) DMA channels that trigger each other in an alternating manner.
        This way, the DMA configuration is done only once, and all the main program needs to do is to fill the buffers with new samples.
        '''

        def _set_up_dma(self):
            dma_a = rp2.DMA()
            dma_b = rp2.DMA()
            

if __name__ == "__main__":
    # try out the class

    print("Starting DMAWavPlayer example...")
    player = RingDMAWavPlayer(buffer_n_samples=2048)
#     for _ in range(3):
#         print(f"Temperature: {player.read_temperature():.2f} C")
#         time.sleep(1)
    player.play_wav(wav_file_path="sweet_child_mono_16b_18k295.wav", loop=False)
    print("Playback finished.")
#     for _ in range(3):
#         print(f"Temperature: {player.read_temperature():.2f} C")
#         time.sleep(_+0.5)


