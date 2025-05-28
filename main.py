from machine import Pin, PWM
import time

print('main.py started')

PWM_GPIO = 0


pwm = PWM(Pin(PWM_GPIO))
pwm.init(freq=440, duty_u16=1677)
time.sleep(0.25)
pwm.init(freq=880, duty_u16=1677)
time.sleep(0.25)
pwm.init(freq=440, duty_u16=1677)
time.sleep(0.25)
pwm.deinit()  # stop PWM
time.sleep(0.25)

# now loop with a pseudo-continuous frequency change
for freq in range(220, 2400, 20):
    pwm.init(freq=freq, duty_u16=1677)
    time.sleep_ms(12)


# stop PWM
pwm.deinit()  # stop PWM

time.sleep(0.25)

# now, take it a step further: use a fixed frequency and modulate the duty cycle for audio tone generation
DC_HIGH = 8276
DC_LOW = 0
FREQ = 150_000_000 // 2**12
pwm.init(freq=FREQ, duty_u16=DC_HIGH)
# get the resulting frequency and duty cycle
print(f'PWM frequency: {pwm.freq()}, expected: {FREQ}')
print(f'PWM duty cycle high: {pwm.duty_u16()}, expected: {DC_HIGH}')
pwm.duty_u16(DC_LOW)
print(f'PWM duty cycle low: {pwm.duty_u16()}, expected: {DC_LOW}')

t0 = time.ticks_ms()
while time.ticks_diff(time.ticks_ms(), t0) < 1000:
    pwm.duty_u16(DC_LOW)
    time.sleep_us(200)
    pwm.duty_u16(DC_HIGH)
    time.sleep_us(200)

# total sleep time is 400us, so 2.5kHz is expected, 2.3kHz is measured.
# now let's generate a real tone.
TONE_FREQ = 440
SAMPLE_RATE = 36000
sleep_time_us = max(1000000 // SAMPLE_RATE - 17, 0)
print(f'sleep time: {sleep_time_us}us')
buffer_size = SAMPLE_RATE // TONE_FREQ
print(f'buffer size: {buffer_size}')
# create a buffer of 0s. Format is uint8
buffer = bytearray(buffer_size)
# fill the buffer with a sine wave
import math
for i in range(buffer_size):
    # generate a sine wave
    buffer[i] = int(127.5 * (1 + math.sin(2 * math.pi * i / buffer_size)))

print(f'single tone buffer: {[int(value) for value in buffer]}')

# in a loop, play the buffer
t0 = time.ticks_ms()
while time.ticks_diff(time.ticks_ms(), t0) < 1000:
    for i in range(buffer_size):
        # set the duty cycle to the value in the buffer
        pwm.duty_u16(buffer[i] << 8)
        time.sleep_us(sleep_time_us)

# stop PWM
pwm.deinit()  # stop PWM
time.sleep(0.25)


# now, instead of a single tone, let's play a chord


# # create a buffer of 0s. Format is uint8
buffer_size = 4 * SAMPLE_RATE // TONE_FREQ
print(f'buffer size: {buffer_size}')
TONE_FREQS = [440, 554, 659, 880]
# # create a buffer of 0s. Format is uint8
# buffer = bytearray(buffer_size)
# # fill the buffer with a sine wave
# for i in range(buffer_size):
#     # generate a chord
#     sample_value = 0
#     for freq in TONE_FREQS:
#         sample_value = sample_value + int(127.5 * (1 + math.sin(2 * math.pi * i / (buffer_size / freq))))
#     # normalize the sample value to 0-255
#     sample_value = sample_value // math.sqrt(len(TONE_FREQS))
#     # set the sample value to the buffer
#     buffer[i] = int(sample_value)

# build one period long buffer that can hold any of the tones

buffer = bytearray(buffer_size)

for i in range(buffer_size):
    sample = 0.0
    for freq in TONE_FREQS:
        # either use the period length …
        period = SAMPLE_RATE / freq               # float!
        sample += math.sin(2 * math.pi * i / period)

        # … OR the equivalent “angular step” form:
        # sample += math.sin(2 * math.pi * freq * i / SAMPLE_RATE)

    # bring the summed value into the unsigned 0-255 range
    sample = 127.5 * (1 + sample / len(TONE_FREQS))
    buffer[i] = int(sample)

print(f'chord buffer: {[int(value) for value in buffer]}')

# in a loop, play the buffer
t0 = time.ticks_ms()
while time.ticks_diff(time.ticks_ms(), t0) < 1000:
    for i in range(buffer_size):
        # set the duty cycle to the value in the buffer
        pwm.duty_u16(buffer[i] << 8)
        time.sleep_us(sleep_time_us)




# stop PWM
pwm.deinit()  # stop PWM

print('done with software duty cycle modulation')
time.sleep(0.25)
print('now using PIO state machine for PWM')

# now, let's define and use a PIO state machine to load PWM values to
from machine import Pin
import rp2, time

# ────────────────────────────────────────────────────────────────
# PIO program: dynamic-duty 12-bit PWM  (low first, high afterwards)
# ────────────────────────────────────────────────────────────────
@rp2.asm_pio(
    set_init     = rp2.PIO.OUT_LOW,
    out_shiftdir=rp2.PIO.SHIFT_RIGHT,
    autopull      = False,             # we manage pulls manually
)
def pwm12_dynamic():
    # ── one-time setup ───────────────────────────────────────────
    pull(block)                  # 1) receive 0xFFF from CPU …
    mov(isr, osr)           #    … and park it permanently in ISR
    # set(pindirs, 1)        # make the pin an OUTPUT  (can be here or via set_init)
    # ── per-period loop ─────────────────────────────────────────
    label("period_start")
    pull()                  # 2) next 12-bit duty value
    out(x, 12)              #    X ← duty
    mov(y, isr)             #    Y ← 4095   (counter starts full-scale)
    set(pins, 0)            # pin LOW at beginning of cycle
    # ---- PWM count-down ---------------------------------------
    label("pwm_loop")
    jmp(x_not_y, "skip_high")   # fall through *once* when Y == X
    set(pins, 1)               # pin goes HIGH for the rest of the cycle
    label("skip_high")
    jmp(y_dec, "pwm_loop")      # keep counting down until Y == 0
    jmp("period_start")         # start next period (fetch new duty)


@rp2.asm_pio(  set_init     = rp2.PIO.OUT_LOW,
               out_shiftdir=rp2.PIO.SHIFT_RIGHT,
               autopull      = False,             # we manage pulls manually
)
def pwm_fixed():
    # ── one-time setup ───────────────────────────────────────────
    pull(block)                  # 1) receive 0xFFF from CPU …
    mov(isr, osr)           #    … and park it permanently in ISR
    # ── per-period loop ─────────────────────────────────────────
    label("period_start")
    pull()                  # 2) next 12-bit duty value
    out(x, 12)              #    X ← duty
    mov(y, isr)             #    Y ← 4095   (counter starts full-scale)
    set(pins, 0)            # pin LOW at beginning of cycle
    # set(y, 31)
    # ---- PWM count-down ---------------------------------------
    label("pwm_loop")
    jmp(x_not_y, "skip_high")   # fall through *once* when Y == X
    set(pins, 1)               # pin goes HIGH for the rest of the cycle
    label("skip_high")
    jmp(y_dec, "pwm_loop")      # keep counting down until Y == 0
    jmp("period_start")         # start next period (fetch new duty)


# create a buffer for playback
TONE_FREQ = 440
SAMPLE_RATE = 36000
sleep_time_us = max(1000000 // SAMPLE_RATE - 17, 0)
print(f'sleep time: {sleep_time_us}us')
buffer_size = SAMPLE_RATE // TONE_FREQ
print(f'buffer size: {buffer_size}')
# create a buffer of 0s. Format is uint8
buffer = bytearray(buffer_size)
# fill the buffer with a sine wave
import math
for i in range(buffer_size):
    # generate a sine wave
    buffer[i] = int(127.5 * (1 + math.sin(2 * math.pi * i / buffer_size)))



# GPIO to drive (single pin only):
PIN_NUM = 0            # change as required

# State-machine clock.
# The PWM period takes 4096 iterations × 2 instructions ≈ 8192 clocks.
# pwm_freq ≈ sm_clk / 8192
SM_CLK = 150_000_000 # gives sample update rate of ~18.29kHz
print('initializing state machine')

# sm = rp2.StateMachine(0, pwm12_dynamic,
#                       freq     = SM_CLK,
#                       set_base  = Pin(PIN_NUM))
sm = rp2.StateMachine(0, pwm_fixed,
                      freq     = 150_000_000,
                      set_base  = Pin(PIN_NUM))

print('state machine initialized')

# 1)  Send the fixed maximum (4095) *once* before enabling:
if sm.tx_fifo() == 0:
    print('putting 4095 to state machine')
    sm.put(4095)
    # sm.put(511)

# sm.put(4095)
print('put 4095, starting state machine')
# 2)  Enable the state machine:
sm.active(1)

# # 3)  Example: sweep through a short table of duty values forever
# triangle = [0, 512, 1024, 2048, 3072, 4095, 3072, 2048, 1024, 512]
# 3) Example: use the previously defined sine wave buffer
triangle = buffer
counter = 0
ticks_us_acc = 0

print('starting state machine loop')
# for i in range(32):
#     # value_to_put = 2 ** i
#     # value to put is i ones in binary: 0, 1, 11, 111, …
#     value_to_put = (1 << i) - 1
#     print(f'i = {i}, value_to_put = {value_to_put}')
#     start_time_ms = time.ticks_ms()
#     while time.ticks_diff(time.ticks_ms(), start_time_ms) < 1000:
#         sm.put(value_to_put)
t0 = time.ticks_ms()
while time.ticks_diff(time.ticks_ms(), t0) < 2000:
    for value in triangle:
        sm.put(value)


# now try a sine wave

#####################################
# TONE_FREQ = 440
# SAMPLE_RATE = 18300
# buffer_size = SAMPLE_RATE // TONE_FREQ
# print(f'buffer size: {buffer_size}')
# # create a buffer of 0s. Format is uint12
# buffer = bytearray(buffer_size, 'H')
# # fill the buffer with a sine wave
# for i in range(buffer_size):
#     # generate a sine wave
#     buffer[i] = int(4095 * (1 + math.sin(2 * math.pi * i / buffer_size)))
#     print(f'calculated value: {int(4095 * (1 + math.sin(2 * math.pi * i / buffer_size)))}, in buffer: {buffer[i]}')
# print(f'sine wave buffer: {[int(value) for value in buffer]}')
##########################################


import array

TONE_FREQ = 440
SAMPLE_RATE = 18300
buffer_size = SAMPLE_RATE // TONE_FREQ
print(f'buffer size: {buffer_size}')

# Use array module to create a uint16 array
buffer = array.array('H', [0] * buffer_size)

# Fill the buffer with a sine wave
for i in range(buffer_size):
    value = int(4095 * 0.5*(1 + math.sin(2 * math.pi * i / buffer_size)))
    buffer[i] = value
    print(f'calculated value: {value}, in buffer: {buffer[i]}')

print(f'sine wave buffer: {list(buffer)}')


# in a loop, play the buffer
t0 = time.ticks_ms()
while time.ticks_diff(time.ticks_ms(), t0) < 1000:
    for i in range(buffer_size):
        sm.put(buffer[i])





# now, read an actual file into a buffer and play it
import struct

def parse_wav_header(wav_file):
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

    return {
        "num_channels": num_channels,
        "sample_rate": sample_rate,
        "bits_per_sample": bits_per_sample,
        "data_offset": data_chunk_offset
    }

def int16_to_uint12(sample_int16):
    # Convert 16-bit signed integer to 12-bit unsigned integer
    sample_uint12 = (sample_int16 + 32768) >> 4
    return sample_uint12 & 0xFFF
    

def get_n_samples_from_wav(filename, N):
    with open(filename, "rb") as wav_file:
        info = parse_wav_header(wav_file)
        wav_file.seek(info["data_offset"])

        num_channels = info["num_channels"]
        bps = info["bits_per_sample"] // 8
        if bps != 2:
            raise NotImplementedError("Only 16-bit WAV files are supported")

        struct_fmt = "<" + "h" * num_channels * N
        data = wav_file.read(N * num_channels * bps)
        if len(data) < N * num_channels * bps:
            raise ValueError("WAV file too short for requested samples")

        unpacked = struct.unpack(struct_fmt, data)

        left = []
        right = [] if num_channels == 2 else None

        for i in range(N):
            # if num_channels == 2:
            #     left.append(int16_to_uint12(unpacked[i * 2]))
            #     right.append(int16_to_uint12(unpacked[i * 2 + 1]))
            # else:
            #     left.append(int16_to_uint12(unpacked[i]))
            left.append(int16_to_uint12(unpacked[i * 2]))

        return left, right, info


FILENAME = "siren16b16k.wav"
N_SAMPLES = 8000
import micropython
print('memory usage before loading WAV file:')
micropython.mem_info()

left_buf, right_buf, wav_info = get_n_samples_from_wav(FILENAME, N_SAMPLES)
# print some samples from left and right channels
print(f'left samples: {left_buf[:100]}')
if right_buf:
    print(f'right samples: {right_buf[:100]}')
print(f"Loaded {len(left_buf)} samples from left channel")
if right_buf:
    print(f"Loaded {len(right_buf)} samples from right channel")
else:
    print("Mono audio detected; right channel is None.")

print('memory usage after loading WAV file:')
micropython.mem_info()


# play the left channel
print('playing left channel')
for k in range(1):
    for i in range(len(left_buf)):
        sm.put(left_buf[i])







print('stopping state machine')
sm.active(0)             # stop the state machine
# print(f'average put() hang time: {1.0 * ticks_us_acc / counter}us')

# elapsed_ms = time.ticks_diff(time.ticks_ms(), start_time_ms)
# sample_rate = 1000.0 * counter / elapsed_ms
# print(f'average sample rate: {sample_rate}Hz')

print('stopped state machine')





print('main.py finished')








