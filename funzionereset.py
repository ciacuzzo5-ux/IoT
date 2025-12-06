'''
obiettivo : un pulsante fisico che quando premuto resetta tutto il sistema sia hardware che software
il reset deve:
Spegnere tutti i LED (rosso, verde, blu)
Portare il servo a 0 gradi (porta chiusa)
Spegnere anche il buzzer
reinizializzare i2c
rinizializzare oled
riavvia la connessione wifi
reinizializzare accelerometro, infrarossi, tastierino
riportare le variabili (attempts, entered, accel_active, servo_angle) allo stato inziale
fermare buzzer e servo
riportare il sistema allo schermo "inserisci codice"

'''

 
def system_reset():
    # 1. Spegni LED
    led_red.value(0)
    led_green.value(0)
    led_blue.value(0)

    # 2. Buzzer spento
    buzzer.duty(0)

    # 3. Servo a 0° (porta chiusa)
    servo_angle(0)

    # 4. Messaggio OLED
    try:
        oled.fill(0)
        oled.text("RESET -", 20, 20)
        oled.text("Riavvio...", 10, 40)
        oled.show()
    except:
        pass   # se l'OLED non funziona, ignoriamo

    # 5. Piccolo delay per mostrare il messaggio
    sleep(0.4)

    # 6. HARD RESET DELLA SCHEDA
    machine.reset()

                
'''
cosa succede quando premi il pulsante:
Servo : 0° (porta CHIUSA)
2.led rossi, verdi e blu spenti
3.Buzzer spento
4.Oled mostra “RESET – Riavvio…”
5.L’esp32 esegue reset hardware totale
6.Il main.py riparte da zero
7.I2C : reinizializzato
8.OLED : reinizializzato
9.MPU6050 :reinizializzato
10.TCRT5000 :reinizializzato
11.tastierino : reinizializzato
12.PWM servo e buzzer : reinizializzati
13.Variabili (attempts, entered, accel_active) → valori iniziali
14.Wi-Fi : ricollegato
15.schermata : Inserisci codice”

'''