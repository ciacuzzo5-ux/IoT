# questo è il main del progetto MUSEO di IOT che gestisce:
# mqqt (pub/sub)
#lettura dell'accelerometro
# controllo led del quadro
# allarme buzzer
# gestione comandi ricevuti da nodered

import time
import machine
from umqttsimple import MQTTClient
from machine import Pin, I2C

#import delle variabili definite in boot.py (ip broker
while True: