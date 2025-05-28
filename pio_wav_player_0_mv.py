from machine import Pin # type: ignore[import]
import rp2, time # type: ignore[import]
import struct
import gc
from array import array

@rp2.asm_pio(
    set_init = rp2.PIO.OUT_LOW,
    out_shiftdir = rp2.PIO.SHIFT_RIGHT,
    autopull = False,
    fifo_join=rp2.PIO.JOIN_TX
)
def pwm_audio_driver():
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



PIN_NUM = 0
SM_CLK = 150_000_000

sm = rp2.StateMachine(
    0,
    pwm_audio_driver,
    freq     = SM_CLK,
    set_base  = Pin(PIN_NUM),
)

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



# now, read an actual file into a buffer and play it

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
    with open(filename, 'rb') as f:
        info = parse_wav_header(f)
        if info['bits_per_sample'] != 16:
            raise ValueError("Only 16-bit WAV supported")
        ch = info['num_channels']
        if ch not in (1, 2):
            raise ValueError("Mono or stereo only")

        f.seek(info['data_offset'])

        # -------- read raw 16-bit samples straight into an array ----------
        total_samples = N * ch
        raw = array('h', [0] * total_samples)       # signed 16-bit
        want_bytes = total_samples * 2
        got_bytes  = f.readinto(raw)                # fills the array
        if got_bytes < want_bytes:
            raise ValueError("WAV shorter than requested length")

    # -------- convert in-place to unsigned-12 and extract left -----------
    out = array('H', [0] * N)                       # final buffer
    for i in range(N):
        out[i] = (raw[i*ch] + 32768) >> 4           # left channel

    return out


# FILENAME = "siren16b16k.wav"
FILENAME = "output.wav"
N_SAMPLES = 38000
gc.collect()
left_buf = get_n_samples_from_wav(FILENAME, N_SAMPLES)
# print some samples from left and right channels

print(f"Loaded {len(left_buf)} samples from left channel")

# play the left channel
print('playing left channel')
for k in range(1):
    for i in range(len(left_buf)):
        sm.put(left_buf[i])

print('stopping state machine')
sm.active(0)