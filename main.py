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


# CONFIGURAZIONE HARDWARE

# Led di stato
led_red = Pin(LED_RED_PIN, Pin.OUT)       # allarme
led_green = Pin(LED_GREEN_PIN, Pin.OUT)   # codice corretto
led_blue = Pin(LED_BLUE_PIN, Pin.OUT)     # WiFi
# led spenti all'avvio
led_red.value(0)
led_green.value(0)
led_blue.value(0)

# Buzzer allarme
buzzer = Pin(BUZZER_PIN, Pin.OUT)
#buzzer spento all'avvio
buzzer.value(0)

# Tasto di RESET
reset_button = Pin(RESET_PIN, Pin.IN, Pin.PULL_UP)

# Tastierino 4x4
rows = [Pin(p, Pin.OUT) for p in ROWS_PINS]
cols = [Pin(p, Pin.IN, Pin.PULL_UP) for p in COLS_PINS]

KEYS = [
    ["1","2","3","A"],
    ["4","5","6","B"],
    ["7","8","9","C"],
    ["*","0","#","D"]
]

# Lettura da tastierino
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



# CONNESSIONE WIFI CON ANIMAZIONE SU OLED E ACCENZIONE LED BLU

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

        # led blu lampeggiante - mentre si connette il led lampeggia
        led_blue.value(1)
        time.sleep(0.2)
        led_blue.value(0)
        time.sleep(0.2)

        # timeout - se non riesce a connettersi entro tot tempo la connessione è fallita
        if time.time() - start > timeout:
            oled_show("Connessione FALLITA")
            led_blue.value(0)
            return None

    # connesso - led blu acceso e sull'oled viene mostrata la scritta "WiFi connesso!"
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
accel = None   # conterrà l’oggetto che gestisce il sensore MPU6050.
ACCEL_AVAILABLE = False   # flag che indica se il sensore è stato trovato correttamente.
baseline = None    # memorizza il valore di riferimento dell’accelerometro quando è a riposo

def init_accel():
    global accel, ACCEL_AVAILABLE
    try:
        accel = MPU6050(i2c)   # crea un’istanza del sensore collegato tramite il bus I2C
        # Se funziona, ACCEL_AVAILABLE diventa True e stampa “MPU6050 OK”.
        ACCEL_AVAILABLE = True
        print("MPU6050 OK")
    except:
        # Se il sensore non viene trovato, entra nell’except e segnala che non è disponibile.
        ACCEL_AVAILABLE = False
        print("MPU6050 non trovato")

def calibrate_accel():
    # Serve a stabilire un valore di riferimento per l’accelerometro quando il dispositivo è fermo.
    global baseline
    if not ACCEL_AVAILABLE:
        return
    vals = accel.get_values()
    # Restituisce un dizionario con i valori degli assi X, Y e Z (AcX, AcY, AcZ).
    # Questi valori vengono memorizzati in baseline
    baseline = (vals['AcX'], vals['AcY'], vals['AcZ'])
    print("Baseline:", baseline)

def movement_detected():
    # Controlla se c’è movimento confrontando i valori attuali dell’accelerometro con il baseline.
    if not ACCEL_AVAILABLE or baseline is None:
        return False
    vals = accel.get_values()
    # Calcola la differenza assoluta su ogni asse (dx, dy, dz) e le somma (total).
    dx = abs(vals['AcX'] - baseline[0])
    dy = abs(vals['AcY'] - baseline[1])
    dz = abs(vals['AcZ'] - baseline[2])
    total = dx + dy + dz
    # Se total supera una soglia definita da MOVEMENT_THRESHOLD,
    # allora ritorna True (movimento rilevato), altrimenti False.
    return total > MOVEMENT_THRESHOLD

init_accel()         # Inizializza l’accelerometro
time.sleep(0.5)      # Attende mezzo secondo per stabilizzare i valori
calibrate_accel()    # Imposta il baseline tramite calibrate_accel()




# SENSORE DI DISTANZA HC-SR04
trig = Pin(PIN_TRIG, Pin.OUT)   # pin di uscita: manda un impulso al sensore per avviare la misura.
echo = Pin(PIN_ECHO, Pin.IN)    # pin di ingresso: legge il tempo che il segnale impiega a tornare indietro.

def distance_cm():
    # Generazione impulso TRIG
    # Imposta il pin TRIG a 0 per 2 ms (garantisce che sia stabile).
    trig.value(0)
    sleep_ms(2)
    # Lo porta a 1 per 10 ms: questo è l’impulso che dice al sensore di partire.
    trig.value(1)
    sleep_ms(10)
    # Lo riporta a 0.
    trig.value(0)

    # Aspetta che echo diventi HIGH (1) e misura per quanto tempo rimane HIGH.
    # Il valore viene restituito in microsecondi (µs).
    # 30000 è un timeout: dopo 30 ms smette di aspettare -> serve per non bloccare il programma.
    duration = time_pulse_us(echo, 1, 30000)
    if duration < 0:
        return None
    # Il tempo misurato è il viaggio completo (andata + ritorno).
    # L’onda sonora nell’aria percorre circa 1 cm ogni 29.1 microsecondi.
    dist = (duration / 2) / 29.1
    return dist


# MQTT
def connect_mqtt():
    # Crea il client MQTT
    client = MQTTClient(MQTT_CLIENT_ID, MQTT_BROKER)
    client.connect()
    # Mostra sullo schermo OLED che la connessione è avvenuta.
    oled_show("MQTT connesso")
    return client

mqtt = connect_mqtt()
# Da questo momento mqtt è il canale per pubblicare messaggi.

# Funzioni di utilità per pubblicare su diversi topic
def mqtt_status(text):
    mqtt.publish(MQTT_TOPIC_STATUS, text.encode())

def mqtt_event(text):
    mqtt.publish(MQTT_TOPIC_EVENTS, text.encode())

def mqtt_command(text):
    mqtt.publish(MQTT_TOPIC_COMMAND, text.encode())
    
# Messaggio iniziale
mqtt_status("SYSTEM_START")
# Appena il dispositivo parte,
# invia sul topic di stato un messaggio che indica l’avvio del sistema.

# Funzione di RESET
def reset_system():
    global attempts, lockout_until, state

    attempts = 0
    lockout_until = 0

    # spegne allarme
    buzzer.value(0)
    led_red.value(0)

    # se era in ALARM torna in modo ARMED
    if state == STATE_ALARM:
        state = STATE_ARMED

    oled_show("Sistema resettato")
    print("RESET eseguito")


# LOGICA DEL CAVEAU
state = STATE_ARMED
entered_code = ""

servo_close()
oled_show("CAVEAU ARMATO")



# LOOP PRINCIPALE

while True:

    # 1) Movimento (accelerometro): tentativo di furto
    if state == STATE_ARMED and movement_detected():
        state = STATE_ALARM
        buzzer.value(1)
        led_red.value(1)
        mqtt_event("MOVIMENTO_RILEVATO")
        oled_show("ALLARME: MOVIMENTO RILEVATO!!!")
        continue

    # 2) Infrarossi IR (mano o oggetto vicino)
    if state == STATE_ARMED and sensor_ir.read():
        mqtt_event("IR_TRIGGER")
        oled_show("ALLARME: IR ATTIVATO!!!")

    # 3) Sensore distanza HC-SR04
    dist = distance_cm()
    if dist is not None and dist < 10:  # soglia allarme
        print("Allarme distanza! Oggetto vicino:", dist, "cm")
        mqtt_event("DISTANZA_CRITICA")

        if state == STATE_ARMED:
            state = STATE_ALARM
            buzzer.value(1)
            led_red.value(1)
            oled_show("ALLARME: DISTANZA VIOLATA!!!")
        # se è già in allarme non cambio stato

    # 4) Lettura tastierino
    MAX_ATTEMPTS = 3    # numero di volte che può provare il codice
    attempts = 0        # numerop di volte che ha inserito il codice
    lockout_until = 0   # timestamp in secondi
    LOCKOUT_TIME = 60  # 1 minuto = 60 sec

    key = read_keypad()
    current_time = time.time()   #Salva l’ora corrente

    # Blocco 2 minuti se troppi tentativi
    if current_time < lockout_until:
        oled_show("BLOCCATO: attendi...")
        led_red.value(1)
        buzzer.value(1)
        time.sleep(0.2)
        buzzer.value(0)
        time.sleep(0.5)
        continue

    if key:
        print("Premuto:", key)   #Stampa quale tasto è stato premuto

        # Reset codice: * cancella tutto il codice inserito.
        if key == "*":
            entered_code = ""
            oled_show("Codice cancellato")
            continue  #salta al prossimo ciclo e ignora il resto del codice

        # Invio codice: quando l’utente preme #, significa “verifica il codice”.
        if key == "#":
            global attempts, lockout_until
            
            #CODICE CORRETTO
            if entered_code == SECRET_CODE:
                # RESET dei tentativi dopo successo
                attempts = 0

                state = STATE_UNLOCKED
                led_green.value(1)
                buzzer.value(0)
                servo_open()
                mqtt_event("ACCESSO_AUTORIZZATO")
                oled_show("CODICE CORRETTO, ACCESSO AUTORIZZATO!")
                #dopo unlock_door sec la porta si chiude
                time.sleep(UNLOCK_DOOR)
                led_green.value(0)
                servo_close()
                state = STATE_ARMED
                entered_code = ""   #Resetta il codice inserito
                oled_show("CAVEAU ARMATO")
                
            #CODICE ERRATO
            else:
                # Se il confronto fallisce:aumenta il contatore attempts di 1
                # e pubblica via MQTT l’evento "CODICE_ERRATO".
                attempts += 1
                mqtt_event("CODICE_ERRATO")

                if attempts >= MAX_ATTEMPTS:
                    # BLOCCO DI 1 MINUTO
                    # Imposta lockout_until al timestamp futuro: fino a quel momento il tastierino sarà bloccato.
                    lockout_until = time.time() + LOCKOUT_TIME
                    state = STATE_ALARM

                    oled_show("TENTATIVI FALLITI! BLOCCATO!")
                    led_red.value(1)
                    buzzer.value(1)
                    time.sleep(2)
                    buzzer.value(0)
                    entered_code = ""
                    continue

                # ERRORE NORMALE
                oled_show("CODICE ERRATO!")
                led_red.value(1)
                buzzer.value(1)
                time.sleep(1)
                buzzer.value(0)
                entered_code = ""

            continue

    # Aggiunta caratteri (solo numeri)
    if key not in ["A", "B", "C", "D"]:
        entered_code += key
        # Sull’OLED mostra la scritta Codice: ****
        # dove il numero di * corrisponde alla lunghezza del codice inserito
        # (nasconde quindi i numeri reali per sicurezza).
        oled_show("Codice: " + "*" * len(entered_code))


    # 5) Se è in allarme
    if state == STATE_ALARM:
        buzzer.value(1)
        led_red.value(1)
        mqtt_event("ALLARME_ATTIVO")
        time.sleep(0.2)

    time.sleep(0.1)
    
    # 6) Controllo tasto reset
    if reset_button.value() == 0:   # premuto
        reset_system()
        time.sleep(0.5)  # evita rimbalzi
        continue


###########
#Funzionamento:
# - Logo all'avvio
# - Tastierino: numeri->inserimento, * cancella, # conferma
# - Codice corretto: apre servo, LED verde, evento ACCESSO_AUTORIZZATO, richiude dopo UNLOCK_DOOR, torna ARMATO
# - Accelerometro: se rileva movimento -> ALLARME
# - IR: evento IR_TRIGGER
# - HC-SR04: se distanza < 10 cm :  ALLARME DISTANZA
# - MQTT: pubblica stati ed eventi sui topic definiti nel boot
# - Stati: ARMATO, SBLOCCATO, ALLARME
###########