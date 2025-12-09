from machine import Pin
from time import sleep_ms

# Classe TCRT5000

class TCRT5000:
    def __init__(self, pin, invert=False):
        self.pin = Pin(pin, Pin.IN)
        self.invert = invert

    def read(self):
        val = self.pin.value()
        if self.invert:
            val = 1 - val
        return val
    
#Il sensore misura la quantità di luce riflessa.
#“SFONDO” = luce sufficiente → HIGH   =>   LED e fototransistor scoperti → luce riflessa → sensore vede superficie chiara → linea rilevata
#“LINEA” = poca luce → LOW   =>   LED coperti → poca luce riflessa → sensore vede scuro → sfondo rilevato
    
    def is_line(self):
        return self.read() == 1

    def is_background(self):
        return self.read() == 0

