import network
import machine
import time
import math
from machine import Pin, I2C, PWM, SoftI2C
from time import sleep_ms, sleep
from ssd1306 import SSD1306_I2C
from mpu6050 import MPU6050
from TCRT5000 import TCRT5000 
from keypad import Keypad  


# 1. CONFIGURAZIONE 

# WI-FI 
WIFI_NAME = "iPhone di Chiara"       
WIFI_PASSWORD = "23032004"           

# PIN E HARDWARE 
I2C_SDA = 21
I2C_SCL = 22
OLED_WIDTH = 128
OLED_HEIGHT = 64

# PIN SISTEMA SICUREZZA 
LED_RED_PIN    = 4
LED_GREEN_PIN  = 18
LED_BLUE_PIN   = 2
BUZZER_PIN     = 5
SERVO_PIN      = 19 

# SENSORE IR 
TCRT_PIN       = 34 

# Pin TASTIERINO (Passati poi alla classe Keypad)
ROWS_PINS = [27, 14, 12, 13]
COLS_PINS = [26, 25, 33, 32]

SECRET_CODE = "2004"
MAX_ATTEMPTS = 3
BLOCK_TIME = 10
SOGLIA_SCASSO = 0.15 # Sensibilità MPU
PI = 3.14159265


# 2. INIZIALIZZAZIONE HARDWARE

# OLED
i2c = SoftI2C(scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=100000)
oled = SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c)
oled_present = True

# LED
led_blue = Pin(LED_BLUE_PIN, Pin.OUT); led_blue.value(0)
led_red = Pin(LED_RED_PIN, Pin.OUT)
led_green = Pin(LED_GREEN_PIN, Pin.OUT)

# BUZZER
buzzer = PWM(Pin(BUZZER_PIN)); buzzer.duty(0)

# SERVO
servo = PWM(Pin(SERVO_PIN), freq=50)

# INIZIALIZZAZIONE TASTIERINO (tramite Classe) 
kp = Keypad(ROWS_PINS, COLS_PINS)

# INIZIALIZZAZIONE SENSORE QUADRO (TCRT5000)
# invert=True solitamente fa sì che is_background() sia True quando NON c'è riflesso.
# Questo corrisponde alla logica: "Vedo lo sfondo" = "Quadro rimosso"
tcrt_sensor = TCRT5000(pin=TCRT_PIN, invert=True)


# 3. FUNZIONI DI SUPPORTO (Wifi, OLED, Suoni)

def oled_show_wifi(msg):
    oled.fill(0); oled.text(msg, 0, 20); oled.show()

def led_blink(led, interval=0.2):
    led.value(1); time.sleep(interval); led.value(0); time.sleep(interval)

def connect_wifi(timeout=15):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_NAME, WIFI_PASSWORD)
    start_time = time.time()
    dot_count = 0
    while not wlan.isconnected():
        dot_count = (dot_count + 1) % 4
        oled_show_wifi(f"Connessione{'.' * dot_count}")
        led_blink(led_blue, interval=0.3)
        if time.time() - start_time > timeout:
            oled_show_wifi("Connessione fallita!")
            led_blue.value(0)
            return None
    led_blue.value(1)
    oled_show_wifi("Wi-Fi connesso!")
    time.sleep(2)
    return wlan

def oled_frame():
    oled.rect(0, 0, 128, 64, 1)

def oled_center(text, y):
    x = max(0, (128 - len(text)*8)//2)
    oled.text(text, x, y)

def oled_show(title, msg=""):
    oled.fill(0); oled_frame()
    oled_center(title, 10)
    if msg: oled_center(msg, 35)
    oled.show()

def oled_countdown(title, sec):
    for i in range(sec, 0, -1):
        oled.fill(0); oled_frame()
        oled_center(title, 5); oled_center("Chiusura porta:", 25); oled_center(str(i), 45)
        oled.show(); sleep(1)

def servo_angle(angle):
    duty = int((angle / 180) * 102 + 26)
    servo.duty(duty)

def beep_ok():
    buzzer.freq(2000); buzzer.duty(512); sleep_ms(150); buzzer.duty(0)

def beep_error():
    buzzer.freq(1000); buzzer.duty(512); sleep_ms(200); buzzer.duty(0)

# Allarme sinusoidale
def play_continuous_siren(duration_ms=2000):
    start = time.ticks_ms()
    buzzer.duty(512)
    x = 0
    while time.ticks_diff(time.ticks_ms(), start) < duration_ms:
        sinVal = math.sin(x * 10 * PI / 180)
        tone = 2000 + int(sinVal * 500)
        buzzer.freq(tone)
        sleep_ms(10); x += 1
    buzzer.duty(0)

def activate_alarm(msg_line2):
    # Funzione unica per scatenare l'allarme
    led_red.on()
    oled_show("ALLARME!!!", msg_line2)
    play_continuous_siren(2000)
    led_red.off()
    oled_show("Inserisci", "codice")


# 4. MAIN LOOP

if __name__ == "__main__":
    
    # FASE 1: CONNESSIONE 
    wlan = connect_wifi()
    if wlan and wlan.isconnected():
        print("Sistema Connesso.")
    
    # FASE 2: SETUP SENSORI
    
    # Attivazione dei sensori che supportano il protocollo I2C
    scan = i2c.scan()
    mpu = None
    # L'oled e l'accelerometro vengono riconosciuti grazie al loro indirizzo
    if 0x68 in scan: mpu = MPU6050(i2c, addr=0x68)
    elif 0x69 in scan: mpu = MPU6050(i2c, addr=0x69)
    
    # All'inizio la porta è chiusa
    servo_angle(0)
    
    attempts = MAX_ATTEMPTS
    entered = ""
    accel_active = True
    last_key_time = 0

    oled_show("SISTEMA", "PRONTO!")
    sleep(1)
    oled_show("Inserisci", "codice")

    # FASE 3: LOOP DI SICUREZZA 
    while True:

        # A) CONTROLLO SENSORI (Solo se l'allarme è ATTIVO)
        if accel_active:
            
            # 1. Controllo Scasso (Accelerometro)
            if mpu and mpu.is_tampered(threshold=SOGLIA_SCASSO):
                print("Allarme: Vibrazione rilevata")
                activate_alarm("MOVIMENTO!")
                servo_angle(0) # Chiude la porta
            
            # 2. Controllo Quadro (TCRT5000)
            # Logica: Se is_background() è True, significa che vede il "vuoto" -> Quadro rimosso
            elif tcrt_sensor.is_background():
                print("Allarme: Quadro rimosso")
                activate_alarm("QUADRO TOLTO!")
                servo_angle(0) # Chiude la porta

        # B) GESTIONE TASTIERINO
        key = kp.get_key()  # Usa la nuova classe Keypad
        
        if key:
            if time.ticks_diff(time.ticks_ms(), last_key_time) < 300:
                continue
            last_key_time = time.ticks_ms()

            if key == "C":
                entered = ""; oled_show("Inserisci", "codice"); continue

            entered += key
            oled_show("Codice:", "*" * len(entered))

            if len(entered) == len(SECRET_CODE):
                # CODICE CORRETTO
                if entered == SECRET_CODE:
                    accel_active = False # Disattiva i sensori
                    led_green.on(); beep_ok()
                    oled_show("ACCESSO", "CONSENTITO")
                    
                    servo_angle(90) # Apre la porta per 10sec
                    oled_countdown("PORTA APERTA", 10)
                    servo_angle(0)  # Chiude la porta
                    led_green.off()
                    oled_show("PORTA", "CHIUSA")
                    sleep(1)
                    
                    # Reset variabili
                    attempts = MAX_ATTEMPTS; entered = ""; accel_active = True

                # CODICE ERRATO
                else:
                    attempts -= 1
                    led_red.on(); beep_error(); led_red.off()

                    if attempts > 0:
                        oled_show("Codice errato!", f"Hai {attempts} tentativi")
                        sleep(1.5); oled_show("Inserisci", "codice"); entered = ""
                    else:
                        accel_active = False # Blocca tutto
                        for i in range(BLOCK_TIME, 0, -1):
                            oled_show("CAVEAU BLOCCATO!", f"Attendi {i}s")
                            # Accenzione-spegnimento del led e del buzzer
                            led_red.on(); buzzer.freq(1000); buzzer.duty(512)
                            sleep_ms(150); led_red.off(); buzzer.duty(0)
                            sleep_ms(100)
                            led_red.on(); buzzer.freq(1000); buzzer.duty(512)
                            sleep_ms(150); led_red.off(); buzzer.duty(0)
                            sleep_ms(600)
                        # Richiede il codice
                        attempts = MAX_ATTEMPTS; entered = ""; accel_active = True
                        oled_show("Inserisci", "codice")

        sleep_ms(40)