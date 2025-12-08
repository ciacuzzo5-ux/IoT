import network
import machine
import time
import math
import framebuf
import utime
import ubinascii
import sys

from machine import Pin, I2C, PWM, SoftI2C
from time import sleep_ms, sleep
from ssd1306 import SSD1306_I2C
from mpu6050 import MPU6050
from TCRT5000 import TCRT5000 
from keypad import Keypad
from umqttsimple import MQTTClient

from boot import (
    WIFI_NAME,
    WIFI_PASSWORD,
    MQTT_BROKER,
    MQTT_CLIENT_ID, 
    MQTT_TOPIC_STATUS, 
    MQTT_TOPIC_EVENTS, 
    MQTT_TOPIC_COMMAND,
    SECRET_CODE,
    MAX_ATTEMPTS,
    BLOCK_TIME,
    SOGLIA_SCASSO,
    PI,
    LED_RED_PIN,
    LED_GREEN_PIN,
    LED_BLUE_PIN,
    BUZZER_PIN,
    SERVO_PIN,
    RESET_BUTTON_PIN,
    TCRT_PIN,
    I2C_SDA,
    I2C_SCL,
    OLED_WIDTH,
    OLED_HEIGHT,
    ROWS_PINS,
    COLS_PINS,
    LOGO
    )

# 1. INIZIALIZZAZIONE HARDWARE

# PULSANTE RESET
reset_button = Pin(RESET_BUTTON_PIN, Pin.IN, Pin.PULL_UP) # Il pulsante funziona attivo basso

# OLED
i2c = SoftI2C(scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=100000)
oled = SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c)
oled_present = True

# LED
led_blue = Pin(LED_BLUE_PIN, Pin.OUT); led_blue.value(0)
led_red = Pin(LED_RED_PIN, Pin.OUT)
led_green = Pin(LED_GREEN_PIN, Pin.OUT)

# BUZZER (CONFIGURATO COME PWM)
# Qui viene inizializzato il PWM sul pin del buzzer
buzzer = PWM(Pin(BUZZER_PIN))
buzzer.duty(0) # Inizia spento (duty cycle a 0)

# SERVO
servo = PWM(Pin(SERVO_PIN), freq=50)

# INIZIALIZZAZIONE TASTIERINO (tramite Classe) 
kp = Keypad(ROWS_PINS, COLS_PINS)

# INIZIALIZZAZIONE SENSORE QUADRO (TCRT5000)
# invert=True fa sì che is_background() sia True quando NON c'è riflesso.
# Questo corrisponde alla logica: "Vedo lo sfondo" = "Quadro rimosso"
tcrt_sensor = TCRT5000(pin=TCRT_PIN, invert=True)


# 3. FUNZIONI DI SUPPORTO (Wifi, OLED, Suoni)

# Funzione per gestione del RESET
def system_reset():
    # 1. Spegni LED
    led_red.value(0)
    led_green.value(0)
    led_blue.value(0)

    # 2. Spegni buzzer
    buzzer.duty(0)

    # 3. Porta chiusa
    servo_angle(0)

    # 4. Mostra schermata di RESET usando la tua funzione oled_show()
    # Mostra schermata di reset (se OLED disponibile)
    try:
        oled.fill(0)
        oled.text("RESET!", 20, 20)
        oled.text("Riavvio...", 10, 40)
        oled.show()
    except:
        pass  # OLED non disponibile: prosegue comunque

    # Piccolo ritardo per permettere la visualizzazione
    time.sleep(0.4)

    # 6. Reset hardware totale dell’ESP32
    machine.reset()

# FUNZIONE LOGO
def oled_show_logo():
    # Creazione del buffer grafico usando la variabile globale LOGO (da boot.py)
    fb = framebuf.FrameBuffer(LOGO, 128, 64, framebuf.MONO_HLSB)
    
    oled.fill(0)        # Pulisce lo schermo
    oled.blit(fb, 0, 0) # Disegna il buffer sullo schermo
    oled.show()         # Aggiorna il display fisico
    
    time.sleep(2)       # Attende 2 secondi
    
    oled.fill(0)        # Pulisce lo schermo
    oled.show()
    
def oled_show_wifi(msg):
    oled.fill(0); oled.text(msg, 0, 20); oled.show()

def led_blink(led, interval=0.2):
    led.value(1); time.sleep(interval); led.value(0); time.sleep(interval)

def connect_wifi(timeout=15):
    # Preparazione e Avvio
    wlan = network.WLAN(network.STA_IF)  #Imposta il chip Wi-Fi in modalità Station (Stazione)
    wlan.active(True)   #Accende fisicamente la radio Wi-Fi
    wlan.connect(WIFI_NAME, WIFI_PASSWORD) # Invia le credenziali (SSID e Password) al router e avvia il processo di negoziazione.
    
    # Timer e attesa
    start_time = time.time()  #Memorizza l'orario di inizio per poter calcolare dopo quanto tempo è passato (per il timeout)
    dot_count = 0
    
    while not wlan.isconnected():  # Ciclo finchè la connessione non è stabilita
        # Animazione sull'Oled e led che lampeggia
        dot_count = (dot_count + 1) % 4
        oled_show_wifi(f"Connessione{'.' * dot_count}")
        led_blink(led_blue, interval=0.3)
        
        # Gestione fallimento (Timeout)
        if time.time() - start_time > timeout:  #Se la differenza tra l'ora attuale e l'inizio supera il timeout
            oled_show_wifi("Connessione fallita!")
            led_blue.value(0)
            return None
    
    # Gestione successo
    led_blue.value(1)
    oled_show_wifi("Wi-Fi connesso!")
    time.sleep(2)
    # Restituisce l'oggetto wlan.
    # Questo è importante perché il resto del programma userà questo oggetto per fare richieste su internet.
    return wlan


def mqtt_connect():
    client = MQTTClient(MQTT_CLIENT_ID, MQTT_BROKER)
    client.set_callback(mqtt_on_message)
    client.connect()
    client.subscribe(MQTT_TOPIC_COMMAND)
    oled_show("MQTT connesso", "al broker")
    return client

# successivamente aggiungeremo gli altri comandi
def mqtt_on_message(topic, msg):
    global accel_active
    
    topic = topic.decode()
    msg = msg.decode()

    if topic == MQTT_TOPIC_COMMAND:

        if msg == "apri":
            accel_active = False 
            servo_angle(90)
            oled_show("Porta", "aperta")

        elif msg == "chiudi":

            servo_angle(0)
            accel_active = True 
            oled_show("Porta", "chiusa")


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
    mqtt_client = mqtt_connect()
    mqtt_client.publish(MQTT_TOPIC_STATUS, b"online")
    
    # Mostro il logo, il nome del progetto e il gruppo
    oled_show_logo()
    oled_show("Nome progetto");
    time.sleep(3)
    oled.fill(0)                      # Ripulisco lo schermo
    oled_frame()                      # Disegna la cornice
    oled_center("Gruppo 11", 10)      # Titolo in alto (y=10)
    oled_center("Chiara Iacuzzo", 30) # Primo nome al centro (y=30)
    oled_center("Valeria Lupo", 45)   # Secondo nome in basso (y=45)
    oled.show()
    
    time.sleep(3) # Lascia leggere i nomi per 3 secondi
    
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
    last_key_time = 0   # Rappresenta l'istante esatto (in millisecondi) in cui l'ultimo tasto è stato accettato come valido dal programma.

    oled_show("SISTEMA", "PRONTO!")
    sleep(1)
    oled_show("Inserisci", "codice")

    # FASE 3: LOOP DI SICUREZZA 
    while True:
        
        mqtt_client.check_msg()
        
        # Controllo il pulsante reset
        if reset_button.value() == 0: # pulsante premuto
            time.sleep_ms(50) # debounce
            if reset_button.value() == 1:
                system_reset()
             
        # A) CONTROLLO SENSORI (Solo se l'allarme è ATTIVO)
        if accel_active:
            
            # 1. Controllo Vaso (Accelerometro)
            if mpu and mpu.is_tampered(threshold=SOGLIA_SCASSO):
                print("Allarme: Vibrazione rilevata")
                mqtt_client.publish(MQTT_TOPIC_EVENTS, b"allarme_movimento")
                activate_alarm("MOVIMENTO!")
                servo_angle(0) # Chiude la porta
            
            # 2. Controllo Quadro (TCRT5000)
            # Logica: Se is_background() è True, significa che vede il "vuoto" -> Quadro rimosso
            elif tcrt_sensor.is_background():
                print("Allarme: Quadro rimosso")
                mqtt_client.publish(MQTT_TOPIC_EVENTS, b"allarme_quadro")
                activate_alarm("QUADRO TOLTO!")
                servo_angle(0) # Chiude la porta

        # B) GESTIONE TASTIERINO
        # LETTURA DAL TASTIERINO
        key = kp.get_key()  # Usa la classe Keypad
        # Se key non è vuoto (quindi un tasto è stato premuto), entra nel blocco if.
        if key:
            # GESTIONE DEI RIMBALZI
            """ Il codice calcola la differenza di tempo (ticks_diff) tra l'istante attuale (ticks_ms)
                e l'ultima volta che un tasto è stato accettato (last_key_time).
                Se sono passati meno di 300 millisecondi dall'ultima pressione, il codice esegue continue.
                Questo salta tutto il resto e ricomincia il ciclo,
                ignorando la pressione (considerandola un errore o un "rimbalzo")."""
            if time.ticks_diff(time.ticks_ms(), last_key_time) < 300:
                continue
            last_key_time = time.ticks_ms()

            # TASTO DI CANCELLAZIONE
            """ La variabile entered (che contiene il codice segreto digitato finora) viene svuotata ("").
                Lo schermo viene aggiornato per mostrare il messaggio iniziale ("Inserisci codice").
                continue fa ripartire il ciclo da capo, ignorando le righe successive."""
            if key == "#":
                entered = ""; oled_show("Inserisci", "codice"); continue

            # Memorizzazione e Visualizzazione "Mascherata"
            entered += key
            oled_show("Codice:", "*" * len(entered))

            if len(entered) == len(SECRET_CODE):
                # CODICE CORRETTO
                if entered == SECRET_CODE:
                    accel_active = False # Disattiva i sensori
                    led_green.on(); beep_ok()
                    oled_show("ACCESSO", "CONSENTITO")
                    mqtt_client.publish(MQTT_TOPIC_EVENTS, b"accesso_consentito")
                    
                    servo_angle(90) # Apre la porta per 10sec
                    oled_countdown("PORTA APERTA", 10)
                    servo_angle(0)  # Chiude la porta
                    led_green.off()
                    oled_show("PORTA", "CHIUSA")
                    sleep(1)
                    
                    # Reset variabili
                    attempts = MAX_ATTEMPTS; entered = ""; accel_active = True
                    oled_show("Inserisci", "codice")

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