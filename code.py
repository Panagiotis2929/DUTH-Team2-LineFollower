import time
import analogio
import board
import digitalio
import pwmio
from adafruit_motor import motor

# --- Κουμπί Start/Stop (Maker Pi GP20, Pressed = LOW) ---
BTN_PIN = board.GP20

# --- Αναλογικοί Αισθητήρες (A0) ---
PIN_LEFT = board.GP28
PIN_CENTER = board.GP27
PIN_RIGHT = board.GP26

# --- Κινητήρες: M1 = Αριστερός, M2 = Δεξίος ---
M1A, M1B = board.GP8, board.GP9
M2A, M2B = board.GP10, board.GP11

# --- Παράμετροι Ρύθμισης ---
LOOP_S = 0.00001          # Χρόνος κύκλου λούπας
BASE_THROTTLE = 0.5       # Βασική ταχύτητα κίνησης
KP = 0.7                  # Αναλογική σταθερά διόρθωσης
ERR_FILTER_ALPHA = 0.8    # Συντελεστής φίλτρου EMA
MIN_LINE_SUM = 10000      # Ελάχιστο όριο για εντοπισμό γραμμής
ADC_MAX = 65535           # Μέγιστη τιμή αναλογικής ανάγνωσης
ALL_WHITE_RAW_MIN = 58000 # Όριο για "όλα λευκά" (εκτός πίστας)
PRINT_EVERY = 40          # Συχνότητα εκτύπωσης logs στη σειριακή
SLEW_PER_LOOP = 0.1       # Μέγιστο βήμα αλλαγής ταχύτητας μοτέρ

# --- Αρχικοποίηση Αισθητήρων ---
adc_left = analogio.AnalogIn(PIN_LEFT)
adc_center = analogio.AnalogIn(PIN_CENTER)
adc_right = analogio.AnalogIn(PIN_RIGHT)

# --- Αρχικοποίηση Κινητήρων ---
m1a = pwmio.PWMOut(M1A, frequency=10000)
m1b = pwmio.PWMOut(M1B, frequency=10000)
motor_left = motor.DCMotor(m1a, m1b)

m2a = pwmio.PWMOut(M2A, frequency=10000)
m2b = pwmio.PWMOut(M2B, frequency=10000)
motor_right = motor.DCMotor(m2a, m2b)

pwms = (m1a, m1b, m2a, m2b)

# --- Αρχικοποίηση Μπουτόν ---
btn = digitalio.DigitalInOut(BTN_PIN)
btn.direction = digitalio.Direction.INPUT
btn.pull = digitalio.Pull.UP

def read_lcr():
    """Ανάγνωση raw τιμών από τους 3 αισθητήρες"""
    return adc_left.value, adc_center.value, adc_right.value

def raw_to_line_strength(raw):
    """Μετατροπή: Υψηλή τιμή όταν βλέπει μαύρο"""
    x = ADC_MAX - raw
    if x < 0:
        return 0
    return x

def line_error(l, c, r):
    """Υπολογισμός σφάλματος θέσης στο διάστημα [-1, 1]"""
    s = l + c + r
    if s < 1:
        return 0.0
    return (r - l) / s

def clamp_throttle(t):
    """Περιορισμός τιμών throttle μεταξύ -1.0 και 1.0"""
    if t > 1.0: return 1.0
    if t < -1.0: return -1.0
    return t

def slew_toward(target, current, max_step):
    """Ομαλή μετάβαση ταχύτητας για αποφυγή κραδασμών"""
    d = target - current
    if d > max_step:
        return current + max_step
    if d < -max_step:
        return current - max_step
    return target

print("Line follower (analog). Press GP20 to START, press again to STOP.\n")

try:
    running = False
    btn_armed = True
    err_filtered = 0.0
    _loop_i = 0
    prev_lt = 0.0
    prev_rt = 0.0

    while True:
        # Έλεγχος κατάστασης μπουτόν (START/STOP)
        pressed = not btn.value
        if pressed and btn_armed:
            running = not running
            btn_armed = False
            print("RUNNING" if running else "STOPPED")
        elif not pressed:
            btn_armed = True

        # Ανάγνωση αισθητήρων και debugging εκτυπώσεις
        l, c, r = read_lcr()
        _loop_i += 1
        if _loop_i >= PRINT_EVERY:
            _loop_i = 0
            print("L", l, "  C", c, "  R", r)

        # Αν το ρομπότ είναι σταματημένο
        if not running:
            motor_left.throttle = 0
            motor_right.throttle = 0
            err_filtered = 0.0
            prev_lt = 0.0
            prev_rt = 0.0
            time.sleep(LOOP_S)
            continue

        # Αν βρεθεί εκτός πίστας (όλα λευκά)
        if l >= ALL_WHITE_RAW_MIN and c >= ALL_WHITE_RAW_MIN and r >= ALL_WHITE_RAW_MIN:
            motor_left.throttle = 0
            motor_right.throttle = 0
            err_filtered = 0.0
            prev_lt = 0.0
            prev_rt = 0.0
            time.sleep(LOOP_S)
            continue

        # Επεξεργασία σημάτων
        lb = raw_to_line_strength(l)
        cb = raw_to_line_strength(c)
        rb = raw_to_line_strength(r)
        s = lb + cb + rb

        # Αν η ένταση της γραμμής είναι χαμηλή, σταματάει
        if s < MIN_LINE_SUM:
            motor_left.throttle = 0
            motor_right.throttle = 0
            err_filtered = 0.0
            prev_lt = 0.0
            prev_rt = 0.0
        else:
            # Υπολογισμός σφάλματος και φιλτράρισμα EMA
            err = line_error(lb, cb, rb)
            if err > 1.0: err = 1.0
            elif err < -1.0: err = -1.0
            
            a = ERR_FILTER_ALPHA
            err_filtered = a * err + (1.0 - a) * err_filtered
            
            # Υπολογισμός διορθωτικής τιμής στροφής (Turn)
            turn = KP * err_filtered
            base = BASE_THROTTLE
            
            # Στόχοι ταχύτητας κινητήρων
            lt_tgt = clamp_throttle(base - turn)
            rt_tgt = clamp_throttle(base + turn)
            
            # Εφαρμογή Slew Rate Control
            lt = slew_toward(lt_tgt, prev_lt, SLEW_PER_LOOP)
            rt = slew_toward(rt_tgt, prev_rt, SLEW_PER_LOOP)
            
            prev_lt = lt
            prev_rt = rt
            
            # Ανάθεση τιμών PWM στους κινητήρες
            motor_left.throttle = lt
            motor_right.throttle = rt

        time.sleep(LOOP_S)

finally:
    # Ασφαλής απενεργοποίηση όλων των περιφερειακών
    motor_left.throttle = 0
    motor_right.throttle = 0
    adc_left.deinit()
    adc_center.deinit()
    adc_right.deinit()
    for p in pwms:
        p.deinit()
    btn.deinit()
