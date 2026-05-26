import time
import analogio
import board
import digitalio
import pwmio
from adafruit_motor import motor

# --- start/stop button (Maker Pi GP20, pressed = LOW) ---
BTN_PIN = board.GP20

# --- sensors (AO) ---
PIN_LEFT = board.GP28
PIN_CENTER = board.GP27
PIN_RIGHT = board.GP26

# --- motors: M1 = left, M2 = right (GP pins match Maker Pi silk) ---
M1A, M1B = board.GP8, board.GP9
M2A, M2B = board.GP10, board.GP11

# --- tuning ---
# LOOP_S: main period. Too small + heavy printing = CPU jitter (choppy). ~250 Hz
# is a good default; raise KP / filter alpha before chasing sub-2 ms loops.
LOOP_S = 0.00001
BASE_THROTTLE = 0.5
KP = 0.7

# 0..1: blend of new lateral error into err_filtered. Low values = slow to correct.
ERR_FILTER_ALPHA = 0.8
MIN_LINE_SUM = 10000
ADC_MAX = 65535

# If every sensor is above this (raw), treat as all white / lifted — do not drive.
ALL_WHITE_RAW_MIN = 58000

# Print L/C/R every N loops (serial is slow; low N at high Hz causes stutter).
PRINT_EVERY = 40

# Max |delta throttle| per loop toward target (raise if steering feels "lazy").
SLEW_PER_LOOP = 0.1

# --- setup ---
adc_left = analogio.AnalogIn(PIN_LEFT)
adc_center = analogio.AnalogIn(PIN_CENTER)
adc_right = analogio.AnalogIn(PIN_RIGHT)

m1a = pwmio.PWMOut(M1A, frequency=10000)
m1b = pwmio.PWMOut(M1B, frequency=10000)
motor_left = motor.DCMotor(m1a, m1b)

m2a = pwmio.PWMOut(M2A, frequency=10000)
m2b = pwmio.PWMOut(M2B, frequency=10000)
motor_right = motor.DCMotor(m2a, m2b)

pwms = (m1a, m1b, m2a, m2b)

btn = digitalio.DigitalInOut(BTN_PIN)
btn.direction = digitalio.Direction.INPUT
btn.pull = digitalio.Pull.UP


def read_lcr():
    return adc_left.value, adc_center.value, adc_right.value


def raw_to_line_strength(raw):
    """High when sensor sees black (raw ADC is low on black)."""
    x = ADC_MAX - raw
    if x < 0:
        return 0
    return x


def line_error(l, c, r):
    """l,c,r = line strengths. About -1 line left, +1 line right."""
    s = l + c + r
    if s < 1:
        return 0.0
    return (r - l) / s


def clamp_throttle(t):
    if t > 1.0:
        return 1.0
    if t < -1.0:
        return -1.0
    return t


def slew_toward(target, current, max_step):
    """Move current toward target by at most max_step (reduces jerky PWM)."""
    d = target - current
    if d > max_step:
        return current + max_step
    if d < -max_step:
        return current - max_step
    return target


print("Line follower (analog). Press GP20 to START, press again to STOP. Ctrl+C to quit.\n")

try:
    running = False
    btn_armed = True
    err_filtered = 0.0
    _loop_i = 0
    prev_lt = 0.0
    prev_rt = 0.0

    while True:
        pressed = not btn.value

        if pressed and btn_armed:
            running = not running
            btn_armed = False
            print("RUNNING" if running else "STOPPED")
        elif not pressed:
            btn_armed = True

        l, c, r = read_lcr()
        _loop_i += 1
        if _loop_i >= PRINT_EVERY:
            _loop_i = 0
            print("L", l, "  C", c, "  R", r)

        if not running:
            motor_left.throttle = 0
            motor_right.throttle = 0
            err_filtered = 0.0
            prev_lt = 0.0
            prev_rt = 0.0
            time.sleep(LOOP_S)
            continue

        if l >= ALL_WHITE_RAW_MIN and c >= ALL_WHITE_RAW_MIN and r >= ALL_WHITE_RAW_MIN:
            motor_left.throttle = 0
            motor_right.throttle = 0
            err_filtered = 0.0
            prev_lt = 0.0
            prev_rt = 0.0
            time.sleep(LOOP_S)
            continue

        lb = raw_to_line_strength(l)
        cb = raw_to_line_strength(c)
        rb = raw_to_line_strength(r)
        s = lb + cb + rb

        if s < MIN_LINE_SUM:
            motor_left.throttle = 0
            motor_right.throttle = 0
            err_filtered = 0.0
            prev_lt = 0.0
            prev_rt = 0.0
        else:
            err = line_error(lb, cb, rb)
            if err > 1.0:
                err = 1.0
            elif err < -1.0:
                err = -1.0
            a = ERR_FILTER_ALPHA
            err_filtered = a * err + (1.0 - a) * err_filtered
            turn = KP * err_filtered
            base = BASE_THROTTLE
            lt_tgt = clamp_throttle(base - turn)
            rt_tgt = clamp_throttle(base + turn)
            lt = slew_toward(lt_tgt, prev_lt, SLEW_PER_LOOP)
            rt = slew_toward(rt_tgt, prev_rt, SLEW_PER_LOOP)
            prev_lt = lt
            prev_rt = rt
            motor_left.throttle = lt
            motor_right.throttle = rt

        time.sleep(LOOP_S)
finally:
    motor_left.throttle = 0
    motor_right.throttle = 0
    adc_left.deinit()
    adc_center.deinit()
    adc_right.deinit()
    for p in pwms:
        p.deinit()
    btn.deinit()
