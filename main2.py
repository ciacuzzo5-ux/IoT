# main.py
import network
import machine
import time
from machine import Pin, I2C, PWM, SoftI2C
from umqtt.simple import MQTTClient

from oled import OLED           
from buzzer import BUZZER       
from mpu6050 import MPU6050     
from TCRT5000 import TCRT5000
from ky003 import KY003
from keypad import Keypad       

# IMPORT CONFIGURAZIONE (boot.py) 
from boot import (
    WIFI_NAME, WIFI_PASSWORD, MQTT_BROKER, MQTT_CLIENT_ID, 
    MQTT_TOPIC_STATUS, MQTT_TOPIC_EVENTS, MQTT_TOPIC_COMMAND,
    SECRET_CODE, MAX_ATTEMPTS, BLOCK_TIME, SOGLIA_SCASSO,
    LED_RED_PIN, LED_GREEN_PIN, LED_BLUE_PIN, BUZZER_PIN,
    SERVO_PIN, RESET_BUTTON_PIN, TCRT_PIN, KY003_PIN, I2C_SDA, I2C_SCL,
    OLED_WIDTH, OLED_HEIGHT, ROWS_PINS, COLS_PINS
)

# 1. INIZIALIZZAZIONE HARDWARE
led_red = Pin(LED_RED_PIN, Pin.OUT); led_red.value(0)
led_green = Pin(LED_GREEN_PIN, Pin.OUT); led_green.value(0)
led_blue = Pin(LED_BLUE_PIN, Pin.OUT); led_blue.value(0)

buzzer = BUZZER(BUZZER_PIN)
servo = PWM(Pin(SERVO_PIN), freq=50)

reset_button = Pin(RESET_BUTTON_PIN, Pin.IN, Pin.PULL_UP) 
kp = Keypad(ROWS_PINS, COLS_PINS)
tcrt_sensor = TCRT5000(pin=TCRT_PIN, invert=True)
door_sensor = KY003(KY003_PIN)

i2c = SoftI2C(scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=100000)
oled = OLED(OLED_WIDTH, OLED_HEIGHT, i2c) 

mqtt_client = None
sensor_active = True

# Variabili Heartbeat 
last_heartbeat = 0
HEARTBEAT_INTERVAL = 30000 # 30 secondi

# 2. FUNZIONI DI SUPPORTO
def servo_angle(angle):
    duty = int((angle / 180) * 102 + 26)
    servo.duty(duty)
    
def led_blink(led, interval=0.2):
    led.value(1); time.sleep(interval); led.value(0); time.sleep(interval)

def system_reset():
    led_red.value(0); led_green.value(0); led_blue.value(0)
    servo_angle(0)
    for i in range(8):
        dots = i % 4
        if oled: oled.show("RESET SISTEMA!", f"Riavvio{'.' * dots}")
        time.sleep(0.4)
    machine.reset()

def activate_alarm(reason):
    led_red.value(1)
    oled.show("ALLARME!!!", reason)
    buzzer.play_continuous_siren(3000)
    time.sleep(2)
    buzzer.duty(0)
    led_red.value(0)
    oled.show("Inserisci", "codice")

# 3. GESTIONE WI-FI E MQTT
def connect_wifi(timeout=20):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(False); time.sleep(1); wlan.active(True)
    oled.show("Connessione...", "Attendere")
    wlan.connect(WIFI_NAME, WIFI_PASSWORD)
    start = time.time(); dots = 0
    while not wlan.isconnected():
        dots = (dots + 1) % 4
        oled.show(f"Connessione{'.' * dots}") 
        led_blue.value(not led_blue.value())
        time.sleep(0.5)
        if time.time() - start > timeout:
            oled.show("Connessione fallita!", "Riprovo...")
            return None
    oled.show("Wi-Fi OK!", "Connessione riuscita.") 
    led_blue.value(1)
    time.sleep(3) 
    return wlan

def mqtt_connect():
    client = MQTTClient(MQTT_CLIENT_ID, MQTT_BROKER)
    client.set_callback(mqtt_on_message)
    client.connect()
    client.subscribe(MQTT_TOPIC_COMMAND)
    return client

def mqtt_on_message(topic, msg):
    global sensor_active
    msg_str = msg.decode(); topic_str = topic.decode()
    print(f"MQTT CMD: {msg_str}")

    if topic_str == MQTT_TOPIC_COMMAND.decode():
        if msg_str == "apri":
            sensor_active = False 
            servo_angle(90)
            led_green.value(1); led_red.value(0)
            oled.show("REMOTO", "APERTA")
            buzzer.beep_ok()
            mqtt_client.publish(MQTT_TOPIC_STATUS, b"aperta")
            mqtt_client.publish(MQTT_TOPIC_EVENTS, b"apertura_remota")

        elif msg_str == "chiudi":
            servo_angle(0)
            sensor_active = True 
            led_green.value(0); led_red.value(0); led_blue.value(0)
            oled.show("REMOTO", "CHIUSA")
            buzzer.beep_ok()
            mqtt_client.publish(MQTT_TOPIC_STATUS, b"chiusa")
            time.sleep(1)
            oled.show("Inserisci", "codice")
            
        elif msg_str == "allarme":
            mqtt_client.publish(MQTT_TOPIC_STATUS, b"allarme")
            mqtt_client.publish(MQTT_TOPIC_EVENTS, b"allarme_remoto")
            activate_alarm("REMOTO!")

        elif msg_str == "reset":
            mqtt_client.publish(MQTT_TOPIC_STATUS, b"reset_remoto")
            system_reset()

# 4. MAIN LOOP
if __name__ == "__main__":
    wlan = connect_wifi()
    if not wlan:
        oled.show("No Wi-Fi", "Riavvio...")
        time.sleep(2); machine.reset()

    mqtt_client = mqtt_connect()
    mqtt_client.publish(MQTT_TOPIC_STATUS, b"online")
    
    mpu = None
    try:
        scan = i2c.scan()
        if 0x68 in scan: mpu = MPU6050(i2c, addr=0x68)
        elif 0x69 in scan: mpu = MPU6050(i2c, addr=0x69)
    except: pass
    
    servo_angle(0)
    attempts = MAX_ATTEMPTS
    entered = ""
    last_key_time = 0 
    sensor_active = True
    
    oled.show_logo()
    oled.show("SISTEMA", "PRONTO!")
    mqtt_client.publish(MQTT_TOPIC_STATUS, b"chiusa")
    time.sleep(1)
    oled.show("Inserisci", "codice")

    while True:
        try:
            mqtt_client.check_msg()
            
            # HEARTBEAT
            # Invia un messaggio "pulse" ogni 30 secondi
            if time.ticks_diff(time.ticks_ms(), last_heartbeat) > HEARTBEAT_INTERVAL:
                mqtt_client.publish(b"caveau/heartbeat", b"pulse")
                last_heartbeat = time.ticks_ms()

            if reset_button.value() == 0:
                time.sleep_ms(50) 
                if reset_button.value() == 1: system_reset()

            if sensor_active:
                trigger_reason = ""
                trigger_event = None
                if mpu and mpu.is_tampered(threshold=SOGLIA_SCASSO):
                    trigger_reason = "VASO MOSSO!"
                    trigger_event = b"allarme_vaso"
                elif tcrt_sensor.is_background():
                    trigger_reason = "QUADRO TOLTO!"
                    trigger_event = b"allarme_quadro"
                elif door_sensor.is_magnete_assente():
                    trigger_reason = "PORTA FORZATA!"
                    trigger_event = b"allarme_porta"
                    
                if trigger_event:
                    mqtt_client.publish(MQTT_TOPIC_EVENTS, trigger_event)
                    mqtt_client.publish(MQTT_TOPIC_STATUS, b"allarme")
                    servo_angle(0)
                    activate_alarm(trigger_reason)
                    mqtt_client.publish(MQTT_TOPIC_STATUS, b"chiusa")

            key = kp.get_key()
            if key:
                if time.ticks_diff(time.ticks_ms(), last_key_time) < 300: continue
                last_key_time = time.ticks_ms()
                if key == "#":
                    entered = ""; oled.show("Inserisci", "codice"); continue
                entered += key
                oled.show("Codice:", "*" * len(entered))

                if len(entered) == len(SECRET_CODE):
                    if entered == SECRET_CODE:
                        sensor_active = False 
                        led_green.value(1)
                        buzzer.beep_ok()
                        oled.show("ACCESSO", "CONSENTITO")
                        mqtt_client.publish(MQTT_TOPIC_EVENTS, b"accesso_consentito")
                        mqtt_client.publish(MQTT_TOPIC_STATUS, b"aperta")
                        servo_angle(90)
                        oled.countdown("PORTA APERTA", 10)
                        servo_angle(0)
                        led_green.value(0)
                        oled.show("PORTA", "CHIUSA")
                        mqtt_client.publish(MQTT_TOPIC_STATUS, b"chiusa")
                        attempts = MAX_ATTEMPTS; entered = ""; sensor_active = True
                        time.sleep(1)
                        oled.show("Inserisci", "codice")
                    else:
                        attempts -= 1
                        led_red.value(1); buzzer.beep_error(); led_red.value(0)
                        mqtt_client.publish(MQTT_TOPIC_EVENTS, b"codice_errato")
                        if attempts > 0:
                            oled.show("Codice errato!", f"Rimasti: {attempts}")
                            time.sleep(1.5); oled.show("Inserisci", "codice"); entered = ""
                        else:
                            sensor_active = False 
                            mqtt_client.publish(MQTT_TOPIC_EVENTS, b"caveau_bloccato")
                            mqtt_client.publish(MQTT_TOPIC_STATUS, b"allarme")
                            for i in range(BLOCK_TIME, 0, -1):
                                oled.show("BLOCCATO!", f"Attendi {i}s")
                                led_red.value(not led_red.value()); buzzer.beep_error(); time.sleep(0.5)
                            attempts = MAX_ATTEMPTS; entered = ""; sensor_active = True
                            mqtt_client.publish(MQTT_TOPIC_STATUS, b"chiusa")
                            oled.show("Inserisci", "codice")
            time.sleep_ms(40)
        except OSError as e:
            print("Errore MQTT/Wi-Fi, riavvio...", e)
            machine.reset()