import ssd1306
import framebuf
import time
from boot import LOGO  # Importiamo il LOGO definito nel boot

class OLED:
    def __init__(self, width, height, i2c):
        # Inizializza il driver SSD1306 e lo salva in self.display
        self.width = width
        self.height = height
        self.display = ssd1306.SSD1306_I2C(width, height, i2c)

    # FUNZIONE LOGO
    def show_logo(self):
        # Creazione del buffer grafico usando la variabile globale LOGO
        fb = framebuf.FrameBuffer(LOGO, 128, 64, framebuf.MONO_HLSB)
        
        self.display.fill(0)         # Pulisce lo schermo
        self.display.blit(fb, 0, 0)  # Disegna il buffer sullo schermo
        self.display.show()          # Aggiorna il display fisico
        
        time.sleep(2)                # Attende 2 secondi
        
        self.display.fill(0)         # Pulisce lo schermo
        self.display.show()
        
    def show_wifi(self, msg):
        self.display.fill(0)
        self.center(msg, 20)
        self.display.show()
        
    def frame(self):
        self.display.rect(0, 0, 128, 64, 1)

    def center(self, text, y):
        # Calcola la X per centrare il testo (assumendo font 8x8)
        x = max(0, (128 - len(text) * 8) // 2)
        self.display.text(text, x, y)

    # Funzione principale per mostrare messaggi (Titolo + Corpo)
    def show(self, title, msg=""):
        self.display.fill(0)
        self.frame()            # Disegna la cornice
        self.center(title, 10)  # Titolo in alto
        if msg:
            self.center(msg, 35) # Messaggio sotto
        self.display.show()

    def countdown(self, title, sec):
        for i in range(sec, 0, -1):
            self.display.fill(0)
            self.frame()
            self.center(title, 5)
            self.center("Chiusura porta:", 25)
            self.center(str(i), 45)
            self.display.show()
            time.sleep(1)
