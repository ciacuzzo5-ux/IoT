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
    SERVO_PIN, RESET_BUTTON_PIN, TCRT_PIN, KY003_PIN, I2C_SDA, I2C_SCL,
    OLED_WIDTH, OLED_HEIGHT, ROWS_PINS, COLS_PINS,
    LOGO_PROGETTO, LOGO_WIFI, LOGO_LUCCHETTO_APERTO, LOGO_LUCCHETTO_CHIUSO, LOGO_ALLARME
)

#------------------------------------------------------------------------
# 1. INIZIALIZZAZIONE HARDWARE

# LED 
led_red = Pin(LED_RED_PIN, Pin.OUT); led_red.value(0)
led_green = Pin(LED_GREEN_PIN, Pin.OUT); led_green.value(0)
led_blue = Pin(LED_BLUE_PIN, Pin.OUT); led_blue.value(0)

# BUZZER 
buzzer = BUZZER(BUZZER_PIN)

# SERVO
servo = PWM(Pin(SERVO_PIN), freq=50)

# RESET
reset_button = Pin(RESET_BUTTON_PIN, Pin.IN, Pin.PULL_UP)

# TASTIERINO
kp = Keypad(ROWS_PINS, COLS_PINS)

# SENSORE IR
tcrt_sensor = TCRT5000(pin=TCRT_PIN, invert=True)

# KY003
door_sensor = KY003(KY003_PIN)

# OLED
i2c = SoftI2C(scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=100000)
oled = OLED(OLED_WIDTH, OLED_HEIGHT, i2c) 

# VARIABILI GLOBALI
mqtt_client = None
sensor_active = True

#------------------------------------------------------------------------
# 2. FUNZIONI DI SUPPORTO

# Funzione per aprire/chiudere la porta
def servo_angle(angle):
    duty = int((angle / 180) * 102 + 26)
    servo.duty(duty)
 
 
# Funzione per far lampeggiare il led
def led_blink(led, interval=0.2):
    led.value(1); time.sleep(interval); led.value(0); time.sleep(interval)


# Funzione per il RESET
def system_reset():
    global sensor_active
    # Invio del messaggio MQTT prima del reset
    mqtt_client.publish(MQTT_TOPIC_EVENTS, b"reset_fisico_bottone")
    mqtt_client.publish(MQTT_TOPIC_STATUS, b"resetting")
    # Reset hardware e spegnimento LED
    led_red.value(0); led_green.value(0); led_blue.value(0)
    servo_angle(0)
    sensor_active = False
    # Animazione loop (faccio 8 passaggi, circa 3-4 secondi totali)
    for i in range(8):
        dots = i % 4
        if oled:
            oled.show("RESET SISTEMA!", f"Riavvio{'.' * dots}")
        
        time.sleep(0.4)
    time.sleep(0.5)
    # Esegue il reset hardware: riavvia completamente il microcontrollore
    machine.reset()


# Funzione per attivare l'allarme
def activate_alarm(reason):
    # SEQUENZA DI ALLARME:
    # Si accende il led rosso, si chiude la porta in automatico e scatta la sirena
    led_red.value(1)
    servo_angle(0)
    oled.show_logo(LOGO_ALLARME, 1)
    oled.show("!!!ALLARME!!!", reason)
    buzzer.play_continuous_siren(3000) # Questa funzione è "bloccante". Dopo 3 secondi il buzzer si spegne da solo.
    # Ripristino dello stato iniziale: la porta rimane chiusa
    led_red.value(0)
    oled.show("Inserisci", "codice")

#------------------------------------------------------------------------
# 3. GESTIONE WI-FI E MQTT

# FUNZIONE PER LA CONNESSIONE WI-FI
def connect_wifi(timeout=20):
    # Imposto l'ESP32 come "Station", cioè come un dispositivo connesso al router.
    wlan = network.WLAN(network.STA_IF)
    
    # Spengo l'interfaccia per resettare eventuali stati bloccati
    # Serve a pulire la memoria del chip Wi-Fi da eventuali errori di connessioni precedenti fallite
    wlan.active(False)
    time.sleep(1) 
    wlan.active(True)

    print(f"Tentativo connessione a: {WIFI_NAME}")
    oled.show_status(LOGO_WIFI, "Avvio Wi-Fi!")
    
    # Avvia la connessione
    wlan.connect(WIFI_NAME, WIFI_PASSWORD)
    
    # Per vedere se la connessione fallisce, salvo in momento esatto in cui è iniziata la connessione
    start = time.time()
    dots = 0
    
    # Ciclo di attesa
    while not wlan.isconnected():
        dots = (dots + 1) % 4
        # Animazione per l'OLED
        oled.show(f"Connessione{'.' * dots}") 
        
        # Faccio lampeggiare il LED blu
        led_blue.value(not led_blue.value())
        time.sleep(0.5)
        
        # Controllo timeout: se impiega troppo tempo la connessione fallisce
        if time.time() - start > timeout:
            print("Errore: Timeout connessione Wi-Fi")
            oled.show("CONNESSIONE", "FALLITA!")
            return None
    
    # Connessione riuscita
    # ip_address = Restituisce una lista di parametri di rete. La posizione [0] è l'indirizzo IP assegnato dal router.
    ip_address = wlan.ifconfig()[0]
    print(f"CONNESSO! IP: {ip_address}")
    
    led_blue.value(1) # LED fisso acceso
    oled.show("CONNESSIONE", "RIUSCITA!") 
    time.sleep(3) 
    
    return wlan


# FUNZIONE PER LA CONNESSIONE MQTT
def mqtt_connect():
    # Creazione del Client
    client = MQTTClient(MQTT_CLIENT_ID, MQTT_BROKER, keepalive=15)
    
    # LAST WILL AND TESTAMENT 
    client.set_last_will(b"caveau/connection", b"OFFLINE", retain=True, qos=1)
    
    # Definizione della Callback:
    # Indico al programma quale funzione deve "scattare" ogni volta che arriva un comando dal broker.
    client.set_callback(mqtt_on_message)
    client.connect()
    
    # Appena connesso, dichiaro di essere ONLINE sullo STESSO topic del testamento
    client.publish(b"caveau/connection", b"ONLINE", retain=True, qos=1)
    
    # Iscrizione ai Comandi (Subscribe)
    client.subscribe(MQTT_TOPIC_COMMAND)
    return client

# FUNZIONE PER IL MONITORAGGIO DEI MESSAGGI MQTT
def mqtt_on_message(topic, msg):
    global sensor_active
    msg_str = msg.decode()  # Prende la sequenza di byte e la traduce in una Stringa di testo leggibile
    topic_str = topic.decode()
    
    if topic_str == MQTT_TOPIC_COMMAND.decode():
        if msg_str == "apri":
            # Apertura della porta (i sensori sono disattivati)
            sensor_active = False 
            servo_angle(90)
            led_green.value(1); led_red.value(0)
            oled.show("APERTURA", "DA REMOTO")
            buzzer.beep_ok()
            # Notifico il cambio di stato alla dashboard
            mqtt_client.publish(MQTT_TOPIC_STATUS, b"accesso_autorizzato")
            mqtt_client.publish(MQTT_TOPIC_EVENTS, b"apertura_remota")

        elif msg_str == "chiudi":
            # Chiusura della porta (i sensori devono attivarsi)
            servo_angle(0)
            sensor_active = True 
            led_green.value(0); led_red.value(0);
            oled.show("CHIUSURA", "DA REMOTO")
            buzzer.beep_ok()
            # Notifico il cambio di stato alla dashboard
            mqtt_client.publish(MQTT_TOPIC_STATUS, b"caveau_protetto")
            time.sleep(1)
            oled.show("Inserisci", "codice")
            
        elif msg_str == "allarme":
            # Allarme attivato 
            activate_alarm("REMOTO!")
            # Notifico il cambio di stato alla dashboard
            mqtt_client.publish(MQTT_TOPIC_STATUS, b"allarme_attivo")
            mqtt_client.publish(MQTT_TOPIC_EVENTS, b"allarme_remoto")
            
            
        elif msg_str == "disattiva":
            # Allarme disattivato 
            buzzer.stop()
            led_red.value(0)
            sensor_active = True
            oled.show("ALLARME", "RESETTATO")
            time.sleep(1)
            oled.show("Inserisci", "codice")
            # Notifico il cambio di stato alla dashboard
            mqtt_client.publish(MQTT_TOPIC_STATUS, b"caveau_protetto")
            mqtt_client.publish(MQTT_TOPIC_EVENTS, b"allarme_disattivato_remoto")

        elif msg_str == "reset":
            # Reset da remoto
            # Notifico il cambio di stato alla dashboard
            mqtt_client.publish(MQTT_TOPIC_STATUS, b"reset_remoto")
            system_reset()


#------------------------------------------------------------------------
# 4. MAIN LOOP
if __name__ == "__main__":
    
    # 1. Rete
    wlan = connect_wifi()
    if not wlan:
        oled.show("Connessione fallita!", "Riavvio...")
        time.sleep(2); machine.reset()

    # 2. MQTT
    mqtt_client = mqtt_connect()
    mqtt_client.publish(b"caveau/connection", b"ONLINE", retain=True)
    mqtt_client.publish(MQTT_TOPIC_STATUS, b"online")
    
    # 3. Setup MPU6050: serve a identificare e attivare l'accelerometro
    mpu = None
    try:
        scan = i2c.scan()   # Restituisce una lista di indirizzi (numeri) di tutti i sensori collegati al protocollo
        # L'MPU6050 può avere l'indirizzo 0x68 oppure 0x69.
        if 0x68 in scan: mpu = MPU6050(i2c, addr=0x68)
        elif 0x69 in scan: mpu = MPU6050(i2c, addr=0x69)
    except: pass
    
    # 4. Inizializzazione Variabili
    servo_angle(0)
    attempts = MAX_ATTEMPTS
    entered = ""
    last_key_time = 0 
    sensor_active = True
    
    # Intro Schermo
    oled.show_logo(LOGO_PROGETTO)
    
    oled.show("Gruppo 11", "Nome progetto")
    time.sleep(2)
    
    oled.show("Iacuzzo Chiara", "Lupo Valeria") 
    time.sleep(3)
    
    
    # Inizializzo sistema
    oled.show("SISTEMA", "PRONTO!")
    mqtt_client.publish(MQTT_TOPIC_STATUS, b"caveau_protetto")
    time.sleep(1)
    oled.show("Inserisci", "codice")

    # 5. Loop Infinito
    while True:
        # Check messaggi MQTT
        mqtt_client.check_msg()
        
       # Check Reset Fisico
        if reset_button.value() == 0:
            time.sleep_ms(100) # Debounce 
            if reset_button.value() == 0: 
                system_reset()

        # CONTROLLO SENSORI
        if sensor_active:
            trigger_reason = ""
            trigger_event = None

            # 1. Controllo VASO (Accelerometro - MPU6050)
            if mpu and mpu.is_tampered(threshold=SOGLIA_SCASSO):
                trigger_reason = "STATUA MOSSA!"
                trigger_event = b"allarme_statua"
            
            # 2. Controllo QUADRO (Sensore IR - TCRT5000)
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
                mqtt_client.publish(MQTT_TOPIC_STATUS, b"allarme_attivo")
                # Attivo l'allarme
                activate_alarm(trigger_reason)
                # Ripristino: dopo che l'allarme è scattato ritorna allo stato protetto
                mqtt_client.publish(MQTT_TOPIC_STATUS, b"caveau_protetto")

        # GESTIONE TASTIERINO
        key = kp.get_key()
        if key:
            if time.ticks_diff(time.ticks_ms(), last_key_time) < 300: continue
            last_key_time = time.ticks_ms()

            # Tasto Cancelletto
            if key == "#":
                entered = ""; oled.show("Inserisci", "codice"); continue

            entered += key
            oled.show("Codice:", "*" * len(entered))  # La password è mostrata crittografata.

            # Verifica codice
            if len(entered) == len(SECRET_CODE):
                if entered == SECRET_CODE:
                    # 1. ACCESSO CONSENTITO
                    # Quando si inserisce il codice corretto, i sensori vengono disattivati
                    sensor_active = False 
                    led_green.value(1)
                    buzzer.beep_ok()
                    oled.show_status(LOGO_LUCCHETTO_APERTO, "CODICE VALIDO!")
                    # Messaggi inviati via MQTT
                    mqtt_client.publish(MQTT_TOPIC_EVENTS, b"accesso_consentito")
                    mqtt_client.publish(MQTT_TOPIC_STATUS, b"accesso_autorizzato")
                    # La porta si apre per 10 secondi
                    servo_angle(90)
                    # Viene mostrato sull'OLED il tempo rimanente
                    oled.countdown("PORTA APERTA:", 10)
                    # La porta si chiude in automatico
                    servo_angle(0)
                    # Il led si spegne
                    led_green.value(0)
                    oled.show("PORTA", "CHIUSA")
                    oled.show("CAVEAU", "PROTETTO")
                    oled.show_logo(LOGO_LUCCHETTO_CHIUSO, 1)
                    # Si torna nello stato protetto
                    mqtt_client.publish(MQTT_TOPIC_STATUS, b"caveau_protetto")
                    # Ripristino del codice
                    attempts = MAX_ATTEMPTS; entered = ""; sensor_active = True
                    time.sleep(1)
                    oled.show("Inserisci", "codice")

                else:
                    # 2. CODICE ERRATO
                    # Si hanno 3 tentativi per inserire il codice corretto
                    attempts -= 1
                    # Allarme per un tentativo errato
                    led_red.value(1)
                    buzzer.beep_error()
                    led_red.value(0)
                    
                    mqtt_client.publish(MQTT_TOPIC_EVENTS, b"codice_errato")

                    if attempts > 0:
                        # Sull'OLED vengono mostrati i tentativi rimanenti
                        oled.show("Codice errato!", f"Tentativi:{attempts}")
                        time.sleep(1.5); oled.show("Inserisci", "codice"); entered = ""
                    else:
                        # 3. BLOCCO SISTEMA
                        # Se si sbagliano tutti e 3 i tentativi si attiva il blocco del Caveau
                        sensor_active = False
                        # Messaggio MQTT
                        mqtt_client.publish(MQTT_TOPIC_EVENTS, b"caveau_bloccato")
                        mqtt_client.publish(MQTT_TOPIC_STATUS, b"allarme_attivo")
                        
                        # Mostro il messaggio sull'OLED
                        oled.show_status(LOGO_LUCCHETTO_CHIUSO, "BLOCCATO!")
                        
                        # Si mostra sull'OLED il tempo rimanente in cui il sistema è bloccato
                        for i in range(BLOCK_TIME, 0, -1):
                            oled.show("CAVEAU BLOCCATO!", f"Attendi {i}s")
                            # Lampeggio e suono
                            led_red.value(1); buzzer.beep_error()
                            time.sleep(0.5)
                            led_red.value(0)
                            time.sleep(0.5) # Ora il ciclo dura esattamente 1 secondo
                            
                        # Una volta che il sistema è sbloccato si può inserire di nuovo il codice
                        attempts = MAX_ATTEMPTS; entered = ""; sensor_active = True
                        mqtt_client.publish(MQTT_TOPIC_STATUS, b"caveau_protetto")
                        oled.show("Inserisci", "codice")

        time.sleep_ms(40)
