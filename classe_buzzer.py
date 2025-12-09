import machine
import time
import math

class BUZZER:
    def __init__(self, pin_number):
        # Inizializza il pin come PWM
        self.pwm = machine.PWM(machine.Pin(pin_number))
        self.pwm.freq(1000) # Frequenza di default
        self.pwm.duty_u16(0) # Volume a 0 (spento)

    def beep_ok(self):
        self.pwm.freq(2000)
        self.pwm.duty_u16(32768) # 50% duty cycle (Volume medio)
        time.sleep_ms(150)
        self.pwm.duty_u16(0)     # Spegni

    def beep_error(self):
        self.pwm.freq(1000)
        self.pwm.duty_u16(32768)
        time.sleep_ms(200)
        self.pwm.duty_u16(0)

    # Allarme sinusoidale
    def play_continuous_siren(self, duration_ms=2000):
        start = time.ticks_ms()
        self.pwm.duty_u16(32768) # Accendi volume
        x = 0
        while time.ticks_diff(time.ticks_ms(), start) < duration_ms:
            # Calcolo seno: convertiamo gradi in radianti
            # x * 10 aumenta la velocità dell'oscillazione
            sinVal = math.sin(x * 10 * math.pi / 180)
            
            # Modula la frequenza tra 1500Hz e 2500Hz
            tone = 2000 + int(sinVal * 500) 
            
            self.pwm.freq(tone)
            time.sleep_ms(10)
            x += 1
            
        self.pwm.duty_u16(0) # Spegni alla fine
