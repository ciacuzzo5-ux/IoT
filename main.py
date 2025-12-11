# main.py - Serve ad implementare tutta la logica del programma.
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
    SERVO_PIN, RESET_BUTTON_PIN, TCRT_PIN, I2C_SDA, I2C_SCL,
    OLED_WIDTH, OLED_HEIGHT, ROWS_PINS, COLS_PINS
)



# 1. INIZIALIZZAZIONE HARDWARE


# LED 
led_red = Pin(LED_RED_PIN, Pin.OUT); led_red.value(0)
led_green = Pin(LED_GREEN_PIN, Pin.OUT); led_green.value(0)
led_blue = Pin(LED_BLUE_PIN, Pin.OUT); led_blue.value(0)

# BUZZER 
buzzer = BUZZER(BUZZER_PIN)

# SERVO
servo = PWM(Pin(SERVO_PIN), freq=50)

# INPUT
reset_button = Pin(RESET_BUTTON_PIN, Pin.IN, Pin.PULL_UP) 
kp = Keypad(ROWS_PINS, COLS_PINS)
tcrt_sensor = TCRT5000(pin=TCRT_PIN, invert=True)

# OLED (Inizializzazione con la tua classe)
i2c = SoftI2C(scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=100000)
oled = OLED(OLED_WIDTH, OLED_HEIGHT, i2c) 

# Variabili globali
mqtt_client = None
sensor_active = True

# KY003
door_sensor = KY003(KY003_PIN)

# 2. FUNZIONI DI SUPPORTO


def servo_angle(angle):
    duty = int((angle / 180) * 102 + 26)
    servo.duty(duty)
    
def led_blink(led, interval=0.2):
    led.value(1); time.sleep(interval); led.value(0); time.sleep(interval)

def system_reset():
    # Reset sistema con avviso su OLED e spegnimento LED 
    led_red.value(0); led_green.value(0); led_blue.value(0)
    servo_angle(0)
    
    # Animazione loop (faccio 8 passaggi, circa 3-4 secondi totali)
    for i in range(8):
        dots = i % 4  # Risultato: 0, 1, 2, 3, 0, 1...
        if oled:
            # Crea la stringa dinamica: "Riavvio", "Riavvio.", "Riavvio..", "Riavvio..."
            oled.show("RESET SISTEMA!", f"Riavvio{'.' * dots}")
        
        # Velocità dell'animazione
        time.sleep(0.4)

    # Esegue il reset hardware
    machine.reset()

def activate_alarm(reason):
    # Sequenza allarme 
    led_red.value(1)
    oled.show("ALLARME!!!", reason)
    
    # Sirena
    buzzer.play_continuous_siren(3000)
    
    # Attende 2 secondi mentre l'allarme suona
    time.sleep(2)
    
    # Ferma il buzzer (assumendo esista un metodo stop())
    buzzer.duty(0)
    
    # Ripristina lo stato iniziale
    led_red.value(0)
    oled.show("Inserisci", "codice")



# 3. GESTIONE WI-FI E MQTT

# FUNZIONE PER LA CONNESSIONE WI-FI
def connect_wifi(timeout=20):
    wlan = network.WLAN(network.STA_IF)
    
    # Spegniamo l'interfaccia per resettare eventuali stati bloccati
    wlan.active(False)
    time.sleep(1) 
    wlan.active(True)

    print(f"Tentativo connessione a: {WIFI_NAME}")
    oled.show("Connessione...", "Attendere")
    
    # Avvia la connessione
    wlan.connect(WIFI_NAME, WIFI_PASSWORD)
    
    start = time.time()
    dots = 0
    
    # Ciclo di attesa
    while not wlan.isconnected():
        dots = (dots + 1) % 4
        # Aggiorna OLED con animazione
        oled.show(f"Connessione{'.' * dots}") 
        
        # Facciamo lampeggiare il LED blu
        led_blue.value(not led_blue.value())
        time.sleep(0.5)
        
        # Controllo timeout
        if time.time() - start > timeout:
            print("Errore: Timeout connessione Wi-Fi")
            oled.show("Connessione fallita!", "Riprovo...")
            return None
    
    # Connessione riuscita
    ip_address = wlan.ifconfig()[0]
    print(f"CONNESSO! IP: {ip_address}")
    
    led_blue.value(1) # LED fisso acceso
    oled.show("Wi-Fi OK!", "Connessione riuscita.") 
    time.sleep(3) 
    
    return wlan

def mqtt_connect():
    client = MQTTClient(MQTT_CLIENT_ID, MQTT_BROKER)
    client.set_callback(mqtt_on_message)
    client.connect()
    client.subscribe(MQTT_TOPIC_COMMAND)
    oled.show("MQTT connesso", "al broker")
    return client

def mqtt_on_message(topic, msg):
    global sensor_active
    msg_str = msg.decode()
    topic_str = topic.decode()
    
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
    
    # 1. Rete
    wlan = connect_wifi()
    if not wlan:
        oled.show("No Wi-Fi", "Riavvio...")
        time.sleep(2); machine.reset()

    # 2. MQTT
    mqtt_client = mqtt_connect()
    mqtt_client.publish(MQTT_TOPIC_STATUS, b"online")
    
    # 3. Setup MPU
    mpu = None
    try:
        scan = i2c.scan()
        if 0x68 in scan: mpu = MPU6050(i2c, addr=0x68)
        elif 0x69 in scan: mpu = MPU6050(i2c, addr=0x69)
    except: pass
    
    # 4. Init Variabili
    servo_angle(0)
    attempts = MAX_ATTEMPTS
    entered = ""
    last_key_time = 0 
    sensor_active = True
    
    # Intro Schermo
    oled.show_logo()
    
    oled.show("Gruppo 11", "Nome progetto")
    time.sleep(2)
    
    oled.show("Iacuzzo Chiara", "Lupo Valeria") 
    time.sleep(3)
    
    
    # Inizializzo sistema
    oled.show("SISTEMA", "PRONTO!")
    mqtt_client.publish(MQTT_TOPIC_STATUS, b"chiusa")
    time.sleep(1)
    oled.show("Inserisci", "codice")

    # 5. Loop Infinito
    while True:
        # Check messaggi MQTT
        mqtt_client.check_msg()
        
        # Check Reset Fisico
        if reset_button.value() == 0:
            time.sleep_ms(50) 
            if reset_button.value() == 1: system_reset()

        # A) CONTROLLO SENSORI
        if sensor_active:
            trigger_reason = ""
            trigger_event = None

            # 1. Controllo VASO (Accelerometro)
            if mpu and mpu.is_tampered(threshold=SOGLIA_SCASSO):
                trigger_reason = "VASO MOSSO!"
                trigger_event = b"allarme_vaso"
            
            # 2. Controllo QUADRO (Sensore IR)
            elif tcrt_sensor.is_background():
                trigger_reason = "QUADRO TOLTO!"
                trigger_event = b"allarme_quadro"
                
            # 3. Controllo PORTA (Sensore Hall KY-003)
            # Se il magnete è assente, la porta è stata forzata
            elif door_sensor.is_magnete_assente():
                trigger_reason = "PORTA FORZATA!"
                trigger_event = b"allarme_porta"
                
            # Se è scattato allarme
            if trigger_event:
                print(f"ALARM: {trigger_reason}")
                mqtt_client.publish(MQTT_TOPIC_EVENTS, trigger_event)
                mqtt_client.publish(MQTT_TOPIC_STATUS, b"allarme")
                servo_angle(0)
                activate_alarm(trigger_reason)
                
                # Ripristino
                mqtt_client.publish(MQTT_TOPIC_STATUS, b"chiusa")

        # B) GESTIONE TASTIERINO
        key = kp.get_key()
        if key:
            if time.ticks_diff(time.ticks_ms(), last_key_time) < 300: continue
            last_key_time = time.ticks_ms()

            # Tasto Cancelletto
            if key == "#":
                entered = ""; oled.show("Inserisci", "codice"); continue

            entered += key
            oled.show("Codice:", "*" * len(entered))

            # Verifica codice
            if len(entered) == len(SECRET_CODE):
                if entered == SECRET_CODE:
                    # --- ACCESSO CONSENTITO ---
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
                    # --- CODICE ERRATO ---
                    attempts -= 1
                    led_red.value(1)
                    buzzer.beep_error()
                    led_red.value(0)
                    
                    mqtt_client.publish(MQTT_TOPIC_EVENTS, b"codice_errato")

                    if attempts > 0:
                        oled.show("Codice errato!", f"Rimasti: {attempts}")
                        time.sleep(1.5); oled.show("Inserisci", "codice"); entered = ""
                    else:
                        # --- BLOCCO SISTEMA ---
                        sensor_active = False 
                        mqtt_client.publish(MQTT_TOPIC_EVENTS, b"caveau_bloccato")
                        mqtt_client.publish(MQTT_TOPIC_STATUS, b"allarme")
                        
                        for i in range(BLOCK_TIME, 0, -1):
                            oled.show("BLOCCATO!", f"Attendi {i}s")
                            led_red.value(not led_red.value()) 
                            buzzer.beep_error()
                            time.sleep(0.5)
                        
                        attempts = MAX_ATTEMPTS; entered = ""; sensor_active = True
                        mqtt_client.publish(MQTT_TOPIC_STATUS, b"chiusa")
                        oled.show("Inserisci", "codice")

        time.sleep_ms(40)
