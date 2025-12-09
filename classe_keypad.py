from machine import Pin
import time

class Keypad:
    def __init__(self, rows_pins, cols_pins, keys=None):
        # Configurazione PIN
        self.rows = [Pin(pin, Pin.OUT) for pin in rows_pins]
        self.cols = [Pin(pin, Pin.IN, Pin.PULL_DOWN) for pin in cols_pins]
        
        # Layout predefinito 4x4 se non viene passato diversamente
        if keys is None:
            self.keys = [
                ['1', '2', '3', 'A'],
                ['4', '5', '6', 'B'],
                ['7', '8', '9', 'C'],
                ['*', '0', '#', 'D']
            ]
        else:
            self.keys = keys

    def get_key(self):
        """Scansiona la matrice e restituisce il tasto premuto o None"""
        for r, row in enumerate(self.rows):
            # Attiva la riga corrente (HIGH)
            row.value(1)
            
            # Controlla se qualche colonna è attiva (HIGH)
            for c, col in enumerate(self.cols):
                if col.value() == 1:
                    # Tasto rilevato! Spegni la riga e restituisci il valore
                    row.value(0)
                    return self.keys[r][c]
            
            # Spegni la riga prima di passare alla prossima
            row.value(0)
            
        return None
