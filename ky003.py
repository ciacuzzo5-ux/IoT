from machine import Pin

class KY003:
    """
    Classe driver per il sensore Hall KY-003.
    """
    def __init__(self, pin_gpio):
        """
        :param pin_gpio: Numero del pin GPIO (es. 15)
        """
        # Configurazione in Input con Pull-Up (essenziale per KY-003)
        self.pin = Pin(pin_gpio, Pin.IN, Pin.PULL_UP)

    def is_magnete_presente(self):
        """
        Ritorna True se il magnete è vicino (Porta CHIUSA).
        Ritorna False se il magnete è lontano.
        """
        # Il sensore va a 0 (LOW) quando sente il campo magnetico
        return self.pin.value() == 0

    def is_magnete_assente(self):
        """
        Ritorna True se il magnete è lontano (Porta APERTA).
        Ritorna False se il magnete è vicino.
        """
        # Il sensore sta a 1 (HIGH) quando non c'è campo magnetico
        return self.pin.value() == 1
    
    def valore_grezzo(self):
        """Ritorna 0 o 1 direttamente dal pin"""
        return self.pin.value()