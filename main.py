# questo è il main del progetto MUSEO di IOT che gestisce:
# - MQTT (pub)
# - lettura dell'accelerometro
# - controllo led del quadro
# - allarme buzzer
# - gestione comandi ricevuti da Node-RED (in questo caso solo eventi pubblicati)

import time
import machine
import network
from umqttsimple import MQTTClient
from machine import Pin, I2C, PWM
from TCRT5000 import TCRT5000
from MPU6050 import MPU6050
from machine import time_pulse_us
from time import sleep_ms
import ssd1306, framebuf

# import delle variabili definite in boot.py
from boot import (
    WIFI_NAME, WIFI_PASSWORD,
    MQTT_BROKER, MQTT_CLIENT_ID,
    MQTT_TOPIC_STATUS, MQTT_TOPIC_EVENTS, MQTT_TOPIC_COMMAND,
    SECRET_CODE, UNLOCK_DOOR,
    STATE_ARMED, STATE_UNLOCKED, STATE_ALARM,
    MOVEMENT_THRESHOLD,
    LED_RED_PIN, LED_GREEN_PIN, LED_BLUE_PIN,
    BUZZER_PIN, SERVO_PIN,
    ROWS_PINS, COLS_PINS,
    IR_SENSOR_PIN, I2C_SDA, I2C_SCL,
    PIN_TRIG, PIN_ECHO
)


# configurazione hardware

# led di stato
led_red = Pin(LED_RED_PIN, Pin.OUT)       # allarme
led_green = Pin(LED_GREEN_PIN, Pin.OUT)   # codice corretto
led_blue = Pin(LED_BLUE_PIN, Pin.OUT)     # WiFi

# led spenti all'avvio
led_red.value(0)
led_green.value(0)
led_blue.value(0)

# Buzzer allarme
buzzer = Pin(BUZZER_PIN, Pin.OUT)
buzzer.value(0)

# Tastierino 4x4
rows = [Pin(p, Pin.OUT) for p in ROWS_PINS]
cols = [Pin(p, Pin.IN, Pin.PULL_UP) for p in COLS_PINS]

KEYS = [
    ["1","2","3","A"],
    ["4","5","6","B"],
    ["7","8","9","C"],
    ["*","0","#","D"]
]

def read_keypad():
    for r in range(4):
        for rr in rows:
            rr.value(1)
        rows[r].value(0)

        for c in range(4):
            if cols[c].value() == 0:
                time.sleep_ms(180)  # antirimbalzo
                return KEYS[r][c]
    return None

# Servo
servo_pwm = PWM(Pin(SERVO_PIN), freq=50)

def servo_set(angle):
    min_duty, max_duty = 40, 115
    duty = int(min_duty + (angle / 180) * (max_duty - min_duty))
    servo_pwm.duty(duty)

def servo_open():
    servo_set(90)

def servo_close():
    servo_set(0)

# Sensore infrarossi IR
sensor_ir = TCRT5000(pin=IR_SENSOR_PIN, invert=True)

# OLED
i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA))
oled = ssd1306.SSD1306_I2C(128, 64, i2c)

def oled_show(text):
    oled.fill(0)
    oled.text(text, 0, 0)
    oled.show()



# CONNESSIONE WIFI CON ANIMAZIONE SU OLED

def connect_wifi_oled(timeout=15):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_NAME, WIFI_PASSWORD)

    start = time.time()
    dot_count = 0

    while not wlan.isconnected():
        # puntini animati
        dot_count = (dot_count + 1) % 4
        dots = "." * dot_count
        oled_show("Connessione" + dots)

        # led blu lampeggiante
        led_blue.value(1)
        time.sleep(0.2)
        led_blue.value(0)
        time.sleep(0.2)

        # timeout
        if time.time() - start > timeout:
            oled_show("Connessione FALLITA")
            led_blue.value(0)
            return None

    # connesso
    led_blue.value(1)
    ip = wlan.ifconfig()[0]
    oled_show("WiFi connesso!")
    print("IP:", ip)
    time.sleep(1)
    return wlan




# LOGO ALL'AVVIO E MESSAGGIO INIZIALE

def show_logo():
    fb = framebuf.FrameBuffer(logo, 128, 64, framebuf.MONO_HLSB)
    oled.fill(0)
    oled.blit(fb, 0, 0)
    oled.show()
    time.sleep(2)

# 1) logo
show_logo()
# 2) nome progetto
oled_show("Progetto MUSEO") # !!!!! nome da cambiare
time.sleep(1.5)
# 3) connessione WiFi con animazione
wlan = connect_wifi_oled()
if wlan is None:
    oled_show("Riavvio tra 3s")
    time.sleep(3)
    machine.reset()

oled_show("Inizializzazione...")






# ACCELEROMETRO
accel = None
ACCEL_AVAILABLE = False
baseline = None

def init_accel():
    global accel, ACCEL_AVAILABLE
    try:
        accel = MPU6050(i2c)
        ACCEL_AVAILABLE = True
        print("MPU6050 OK")
    except:
        ACCEL_AVAILABLE = False
        print("MPU6050 non trovato")

def calibrate_accel():
    global baseline
    if not ACCEL_AVAILABLE:
        return
    vals = accel.get_values()
    baseline = (vals['AcX'], vals['AcY'], vals['AcZ'])
    print("Baseline:", baseline)

def movement_detected():
    if not ACCEL_AVAILABLE or baseline is None:
        return False
    vals = accel.get_values()
    dx = abs(vals['AcX'] - baseline[0])
    dy = abs(vals['AcY'] - baseline[1])
    dz = abs(vals['AcZ'] - baseline[2])
    total = dx + dy + dz
    return total > MOVEMENT_THRESHOLD

init_accel()
time.sleep(0.5)
calibrate_accel()




# SENSORE DI DISTANZA HC-SR04


trig = Pin(PIN_TRIG, Pin.OUT)
echo = Pin(PIN_ECHO, Pin.IN)

def distance_cm():
    trig.value(0)
    sleep_ms(2)
    trig.value(1)
    sleep_ms(10)
    trig.value(0)

    duration = time_pulse_us(echo, 1, 30000)
    if duration < 0:
        return None
    dist = (duration / 2) / 29.1
    return dist


# MQTT

def connect_mqtt():
    client = MQTTClient(MQTT_CLIENT_ID, MQTT_BROKER)
    client.connect()
    print("MQTT connesso")
    oled_show("MQTT connesso")
    return client

mqtt = connect_mqtt()

def mqtt_status(text):
    mqtt.publish(MQTT_TOPIC_STATUS, text.encode())

def mqtt_event(text):
    mqtt.publish(MQTT_TOPIC_EVENTS, text.encode())

def mqtt_command(text):
    mqtt.publish(MQTT_TOPIC_COMMAND, text.encode())
    

mqtt_status("SYSTEM_START")








# LOGICA DEL CAVEAU
state = STATE_ARMED
entered_code = ""

servo_close()
oled_show("Caveau ARMATO")






# LOOP PRINCIPALE

while True:

    # 1) Movimento (accelerometro): tentativo di furto
    if state == STATE_ARMED and movement_detected():
        state = STATE_ALARM
        buzzer.value(1)
        led_red.value(1)
        mqtt_event("MOVIMENTO_RILEVATO")
        oled_show("ALLARME !!!")
        continue

    # 2) Infrarossi IR (mano o oggetto vicino)
    if state == STATE_ARMED and sensor_ir.read():
        mqtt_event("IR_TRIGGER")
        print("IR attivato")

    # 3) Sensore distanza HC-SR04
    dist = distance_cm()
    if dist is not None and dist < 10:  # soglia allarme
        print("Allarme distanza! Oggetto vicino:", dist, "cm")
        mqtt_event("DISTANZA_CRITICA")

        if state == STATE_ARMED:
            state = STATE_ALARM
            buzzer.value(1)
            led_red.value(1)
            oled_show("ALLARME DISTANZA !!!")
        # se è già in allarme non cambio stato

    # 4) Lettura tastierino
    key = read_keypad()
    if key:
        print("Premuto:", key)

        # reset codice
        if key == "*":
            entered_code = ""
            oled_show("Codice cancellato")
            continue

        # invio codice
        if key == "#":
            if entered_code == SECRET_CODE:
                state = STATE_UNLOCKED
                led_green.value(1)
                buzzer.value(0)
                servo_open()
                mqtt_event("ACCESSO_AUTORIZZATO")
                oled_show("Codice CORRETTO")
                time.sleep(UNLOCK_DOOR)
                led_green.value(0)
                servo_close()
                state = STATE_ARMED
                entered_code = ""
                oled_show("Caveau ARMATO")
            else:
                mqtt_event("CODICE_ERRATO")
                oled_show("Codice ERRATO")
                buzzer.value(1)
                time.sleep(1)
                buzzer.value(0)
                entered_code = ""
            continue

        # Aggiunge caratteri (escludo A,B,C,D)
        if key not in ["A", "B", "C", "D"]:
            entered_code += key
            oled_show("Codice: " + "*"*len(entered_code))

    # 5) Se è in allarme
    if state == STATE_ALARM:
        buzzer.value(1)
        led_red.value(1)
        mqtt_event("ALLARME_ATTIVO")
        time.sleep(0.2)

    time.sleep(0.1)

#funzionamento:
# - Logo all'avvio
# - Tastierino: numeri->inserimento, * cancella, # conferma
# - Codice corretto: apre servo, LED verde, evento ACCESSO_AUTORIZZATO, richiude dopo UNLOCK_DOOR, torna ARMATO
# - Accelerometro: se rileva movimento -> ALLARME
# - IR: evento IR_TRIGGER
# - HC-SR04: se distanza < 10 cm :  ALLARME DISTANZA
# - MQTT: pubblica stati ed eventi sui topic definiti nel boot
# - Stati: ARMATO, SBLOCCATO, ALLARME