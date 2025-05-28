# MicroPython

from machine import Pin, PWM, mem32, ADC # type: ignore[import]
import rp2, time # type: ignore[import]
import struct
from array import array
import micropython # type: ignore[import]
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
FILENAME = "sweet_child_stereo_16b_18k295.wav"
# FILENAME = "sweet_child_mono_16b_18k295_long.wav"
# FILENAME = "makka_16b_18k3.wav"
# FILENAME = "tilim_mono_16b_18k295.wav"

class DMAWavPlayer:
    def __init__(self, wav_file_path=None, loop=False, buffer_n_samples=1024, noninverting_gpio_n=0, inverting_gpio_n=5):
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
        self.third_buffer = array('H', [0] * (buffer_n_samples * 2))  # double-sized buffer for DMA transfer in a ping-pong manner
        self._set_up_dma()
        self.offset = None
        self._set_pin_drive_strength()  # set the drive strength for the GPIO pins used by the PIO state machine

    def _set_up_dma(self):
        dma = rp2.DMA()
        # DMA.config(read=None, write=None, count=None, ctrl=None, trigger=False)
        DREQ_SM0_TX = (0 << 3) | 0
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

    def read_temperature(self):
        time.sleep_ms(100)
        reading = self.temp_sensor_adc.read_u16() * 3.3 / 65535.0
        temperature = 27 - (reading - 0.706) / 0.001721  # convert to Celsius using the formula from the datasheet
        time.sleep_ms(100)  # wait for a short time to avoid reading too fast
        return temperature

    def play_wav(self, wav_file_path=None, loop=None):
        '''
        Returns when playback is complete. If loop is True, it will loop the playback indefinitely and never return.
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
                while self.dma.count > self.buffer_n_samples:
                    pass
                # now we can refill the first half of the third buffer with the next samples
                self.third_buffer[:self.buffer_n_samples] = self.pio_samples_buffer[:]
                # back-refill the preceding buffers
                self._convert_buffer_to_pio_samples()
                got_bytes = self._fill_buffer_from_wav()
                self.offset += got_bytes  # update offset for the next read
                # wait for the DMA transfer to complete
                while self.dma.count > 0:
                    pass
            self.led_pin.off()  # turn off the built-in LED to indicate playback end or pause
            if not self.loop:
                break
            else:
                time.sleep_ms(150) # wait for a short time before restarting playback
            # if loop is True, continue playback indefinitely
        self.sm.active(0)  # stop the audio driver state machine
        self.sm.exec('set(pins, 0b00000).side(0b00000)')  # set all pins low to stop the audio output
        self.wav_file_obj.close()  # close the WAV file when done

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

if __name__ == "__main__":
    # try out the class
    print("Starting DMAWavPlayer example...")
    player = DMAWavPlayer()
    player.play_wav(wav_file_path=FILENAME, loop=False)
    player.play_wav(wav_file_path="tilim_mono_16b_18k295.wav", loop=False)
    print("Playback finished.")
    



# # define an LED control pin for the built-in LED
# LED_PIN = Pin(LED_PIN_NUM, Pin.OUT)  # built-in LED on GPIO25

# adc = ADC(TEMP_SENSOR_ADC_NUM)  # create an ADC object for the temperature sensor
# def read_temperature(adc):
#     reading = adc.read_u16() * 3.3 / 65535.0  # convert to voltage
#     temperature = 27 - (reading - 0.706) / 0.001721  # convert to Celsius using the formula from the datasheet
#     return temperature


# def set_drive(pin, mA):                       # mA = 2, 4, 8 or 12
#     PADS_BANK0 = 0x40038000                     # base of pad-control block
#     DRIVE_BITS = {2:0, 4:1, 8:2, 12:3}[mA]    # DRIVE field is bits 5-4
#     addr = PADS_BANK0 + 4*(pin+1)                 # one 32-bit register per pin
#     # read-modify-write the register to set the drive strength. Bits 5:4 are the DRIVE bits
#     mem32[addr] = (mem32[addr] & ~0b110000) | (DRIVE_BITS << 4)

# PIN_NUM = 0 # GPIO0 is the base pin for the audio driver
# DRIVER_PINS_NUM = 5  # number of pins used by the PIO state machine
# PWM_MIDRANGE_PIN_NUM = PIN_NUM + DRIVER_PINS_NUM # base pin for the midrange voltage driver
# PWM_MIDRANGE_PINS_NUM = 5  # number of pins used by the midrange voltage driver

# SM_CLK = 150_000_000

# def init_audio_driver_sm(run=False):
#     sm = rp2.StateMachine(
#         0,
#         pwm_audio_driver,
#         freq     = SM_CLK,
#         set_base  = Pin(PIN_NUM),
#         sideset_base = Pin(PIN_NUM + DRIVER_PINS_NUM),  # sideset pins start after the main pin
#     )
#     if sm.tx_fifo() == 0:
#         sm.put(4095)
#     if run:
#         sm.active(1)
#     return sm

# sm = init_audio_driver_sm()


# # now, read an actual file into a buffer and play it

# def parse_wav_header(wav_file_path):
#     wav_file = open(wav_file_path, 'rb')
#     def read_chunk_header(f):
#         return f.read(4), struct.unpack("<I", f.read(4))[0]

#     riff = wav_file.read(4)
#     if riff != b"RIFF":
#         raise ValueError("Not a valid WAV file: missing RIFF header")

#     _ = wav_file.read(4)  # skip size
#     wave = wav_file.read(4)
#     if wave != b"WAVE":
#         raise ValueError("Not a valid WAV file: missing WAVE header")

#     fmt_chunk_found = False
#     data_chunk_offset = None

#     while True:
#         chunk_id, chunk_size = read_chunk_header(wav_file)
#         if chunk_id == b"fmt ":
#             fmt_chunk_found = True
#             audio_format = struct.unpack("<H", wav_file.read(2))[0]
#             num_channels = struct.unpack("<H", wav_file.read(2))[0]
#             sample_rate = struct.unpack("<I", wav_file.read(4))[0]
#             byte_rate = struct.unpack("<I", wav_file.read(4))[0]
#             block_align = struct.unpack("<H", wav_file.read(2))[0]
#             bits_per_sample = struct.unpack("<H", wav_file.read(2))[0]
#             wav_file.seek(chunk_size - 16, 1)
#         elif chunk_id == b"data":
#             data_chunk_offset = wav_file.tell()
#             break
#         else:
#             wav_file.seek(chunk_size, 1)

#     if not fmt_chunk_found or data_chunk_offset is None:
#         raise ValueError("Missing required WAV chunks")

#     wav_file.close()

#     return {
#         "num_channels": num_channels,
#         "sample_rate": sample_rate,
#         "bits_per_sample": bits_per_sample,
#         "data_offset": data_chunk_offset
#     }

# # def int16_to_uint12(sample_int16):
# #     # Convert 16-bit signed integer to 12-bit unsigned integer
# #     sample_uint12 = (sample_int16 + 32768) >> 4
# #     return sample_uint12 & 0xFFF


# file_info = parse_wav_header(FILENAME)

# buffer_n_samples = 1024  # number of samples to read from the WAV file
# # define a buffer to load raw 16-bit samples from a WAV file
# file_buffer = array('h', [0] * buffer_n_samples * file_info['num_channels'])  # signed 16-bit samples, stereo or mono

# # define a processed sample buffer to hold 12-bit unsigned samples, left channel only
# pio_samples_buffer = array('H', [0] * buffer_n_samples)  # 12-bit unsigned samples

# # a third buffer to hold the processed samples for DMA transfer
# third_buffer = array('H', [0] * buffer_n_samples)  # 12-bit unsigned samples, for DMA transfer


# # _OFFSET = const(32768)
# # _SHIFT  = const(4)
# # @micropython.viper
# # def convert_buffer_to_pio_samples(raw_buf, pio_buf):
# #     src = ptr16(raw_buf)          # 16-bit signed buffer
# #     dst = ptr16(pio_buf)          # 16-bit unsigned buffer
# #     n   = int(len(pio_buf))
# #     for i in range(n):
# #         dst[i] = (int(src[i * 2]) + _OFFSET) >> _SHIFT

# # non-viper version
# def convert_buffer_to_pio_samples(raw_buf, pio_buf, channels_in_wav):
#     # Convert the raw 16-bit signed samples to 12-bit unsigned samples
#     # iterate in steps of 2, since we have stereo samples and we only need the left channel
#     for i in range(len(pio_buf)):
#         pio_buf[i] = (raw_buf[i * channels_in_wav] + 32768) >> 4  # take only the left channel sample

# def fill_buffer_from_wav(filename, buffer, offset):
#     # file info needs to be parsed ahead of time. offset must be based offset + a running position in the file
#     with open(filename, 'rb') as f:
#         f.seek(offset)
#         # Read raw 16-bit samples into the file_buffer
#         got_bytes = f.readinto(buffer)  # fills the array
#         # TODO check if got_bytes is less than expected
#     return got_bytes


# # def copy_buffer_to_third_buffer(src, dst):
# #     # Copy the contents of src buffer to dst buffer
# #     dst[:] = src[:]



# # playback using sm.put() calls: in an outer loop, load from .wav and convert, then in an inner loop, put samples into the state machine's FIFO

# print(f"File info: {file_info}")
# # offset = file_info['data_offset']
# # outer_loop_counter = 0
# # fill_buffer_ticks_acc_us = 0
# # convert_buffer_ticks_acc_us = 0
# # print('starting playback using sm.put() calls')



# dma = rp2.DMA()
# # DMA.config(read=None, write=None, count=None, ctrl=None, trigger=False)
# DREQ_SM0_TX = (0 << 3) | 0
# ctrl_value = dma.pack_ctrl(size=1, inc_write=False, treq_sel=DREQ_SM0_TX)
# # dma.config(
# #     read=pio_samples_buffer,
# #     write=sm,
# #     count=len(pio_samples_buffer),
# #     ctrl=ctrl_value,
# #     trigger=False
# # )



# # in a loop, fill the buffer with samples from the WAV file, convert them, and then use DMA to transfer them to the state machine's FIFO
# offset = file_info['data_offset']  # reset offset to the beginning of the data chunk
# dma_config_counter = 0
# dma_config_ticks_acc_us = 0
# third_buffer_copy_counter = 0
# third_buffer_copy_ticks_acc_us = 0
# # load buffers once
# got_bytes = fill_buffer_from_wav(FILENAME, file_buffer, offset)
# convert_buffer_to_pio_samples(file_buffer, pio_samples_buffer, file_info['num_channels'])  # convert the buffer to 12-bit unsigned samples
# offset += got_bytes  # update offset for the next read
# dma_waits_counter = 0


# # now we will double the size of the third buffer, and use its two halves to transfer data in a ping-pong manner:
# third_buffer = array('H', [0] * (buffer_n_samples * 2))  # 12-bit unsigned samples, for DMA transfer
# for _ in range(2):
#     LED_PIN.on()
#     # fill the first half of the third buffer, then refill the pio_samples_buffer and the file_buffer
#     offset = file_info['data_offset']  # reset offset to the beginning of the data
#     got_bytes = fill_buffer_from_wav(FILENAME, file_buffer, offset)
#     convert_buffer_to_pio_samples(file_buffer, pio_samples_buffer, file_info['num_channels'])  # convert the buffer to 12-bit unsigned samples
#     third_buffer[:buffer_n_samples] = pio_samples_buffer[:]  # copy the first half of the pio_samples_buffer to the third buffer
#     # refill the preceding buffers
#     offset += got_bytes  # update offset for the next read
#     got_bytes = fill_buffer_from_wav(FILENAME, file_buffer, offset)
#     convert_buffer_to_pio_samples(file_buffer, pio_samples_buffer, file_info['num_channels'])  # convert the buffer to 12-bit unsigned samples
#     offset += got_bytes  # update offset for the next read
#     got_bytes = fill_buffer_from_wav(FILENAME, file_buffer, offset)
#     offset += got_bytes  # update offset for the next read
#     # now, the first half of the third buffer is filled with samples, and the second half is empty.
#     # the preceding pio_samples_buffer is loaded with the next sample, and the file_buffer is filled with the second-next samples.

#     for pin_num in range(PIN_NUM, PWM_MIDRANGE_PIN_NUM + PWM_MIDRANGE_PINS_NUM):
#         set_drive(pin_num, 12)  # set drive strength to 12 mA for all pins used by the PIO state machine

#     time.sleep(0.15)
#     sm.active(1)  # start the audio driver state machine

#     # for i in range(100):
#     while got_bytes > 0:
#         # config DMA and start the transfer
#         t0 = time.ticks_us()
#         dma.config(
#             read=third_buffer,
#             write=sm,
#             count=len(third_buffer),
#             ctrl=ctrl_value,
#             trigger=True
#         )
#         # fill the second half of the third buffer with the next samples. The DMA is still going through the first half.
#         third_buffer[buffer_n_samples:] = pio_samples_buffer[:]
#         # back-refill the preceding buffers
#         convert_buffer_to_pio_samples(file_buffer, pio_samples_buffer, file_info['num_channels'])
#         got_bytes = fill_buffer_from_wav(FILENAME, file_buffer, offset)
#         offset += got_bytes  # update offset for the next read
#         # now the second half of the third buffer is filled with samples, waiting for the DMA to reach it.
#         # wait for the remaining DMA transfer count to reach below half of the buffer size
#         while dma.count > buffer_n_samples:
#             pass
#         # now we can refill the first half of the third buffer with the next samples
#         third_buffer[:buffer_n_samples] = pio_samples_buffer[:]
#         # back-refill the preceding buffers
#         convert_buffer_to_pio_samples(file_buffer, pio_samples_buffer, file_info['num_channels'])
#         got_bytes = fill_buffer_from_wav(FILENAME, file_buffer, offset)
#         offset += got_bytes  # update offset for the next read
#         # print(f'temperature: {read_temperature(adc)} C')
#         # wait for the DMA transfer to complete
#         while dma.count > 0:
#             pass
        
#     LED_PIN.off()
#     time.sleep(0.5)
# sm.active(0)
# sm.exec('set(pins, 0b00000).side(0b00000)')  # set all pins low to stop the audio output


