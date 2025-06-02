import rp2
@rp2.asm_pio(
    set_init = (rp2.PIO.OUT_LOW,)*5,
    sideset_init = (rp2.PIO.OUT_LOW,)*4,
    out_shiftdir = rp2.PIO.SHIFT_RIGHT,
    autopull = True,
    pull_thresh = 12,
    # fifo_join=rp2.PIO.JOIN_TX  # not needed here - FIFO of length 4 (words) is sufficient
)
def pwm_audio_driver(): 
    # 8199 clock cycles per sample at 150 MHz => 18294.91 Hz sample rate
    # ── one-time setup ───────────────────────────────────────────
    pull(block)                         # receive 0xFFF from CPU, used as a max count
    mov(isr, osr)                       # … and park it permanently in ISR to be used again and again
    # ── per-period loop ─────────────────────────────────────────
    label("period_start")
    # auto-pull should have already pulled the next duty value
    out(x, 12)                          #    X ← duty
    mov(y, isr)                         #    Y ← 4095   (counter starts full-scale)
    set(pins, 0b00000).side(0b11111)    # pin LOW at beginning of cycle
    nop()
    nop()
    # ---- PWM count-down ---------------------------------------
    label("pwm_loop")
    jmp(x_not_y, "skip_high")           # fall through *once* when Y == X
    set(pins, 0b11111).side(0b00000)    # pin goes HIGH for the rest of the cycle
    label("skip_high")
    jmp(y_dec, "pwm_loop")              # keep counting down until Y == 0
    jmp("period_start")                 # start next period (fetch new duty)