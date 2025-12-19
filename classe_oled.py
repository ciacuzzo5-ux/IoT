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
        
    def frame(self):
        self.display.rect(0, 0, 128, 64, 1)

    def center(self, text, y):
        # Calcola la X per centrare il testo (assumendo font 8x8)
        x = max(0, (128 - len(text) * 8) // 2)
        self.display.text(text, x, y)

    # Funzione per la visualizzazione dei loghi
    def show_logo(self, logo_data, duration=2):
        # Creazione del buffer grafico usando il logo passato alla funzione
        fb = framebuf.FrameBuffer(logo_data, 128, 64, framebuf.MONO_HLSB)
        
        self.display.fill(0)         # Pulisce lo schermo
        self.display.blit(fb, 0, 0)  # Disegna il buffer
        self.display.show()          # Aggiorna il display
        
        if duration > 0:
            time.sleep(duration)      # Attende il tempo prestabilito
            self.display.fill(0)      # Pulisce
            self.display.show()
    
    # Funzione principale per mostrare messaggi (Titolo + Corpo)
    def show(self, title, msg=""):
        self.display.fill(0)
        self.frame()            # Disegna la cornice
        self.center(title, 10)  # Titolo in alto
        if msg:
            self.center(msg, 35) # Messaggio sotto
        self.display.show()
        
    # Funzione per mostrare Icona + Messaggio sotto
    def show_status(self, logo_data, msg):
        self.display.fill(0)
        # Disegna il logo 
        fb = framebuf.FrameBuffer(logo_data, 128, 64, framebuf.MONO_HLSB)
        self.display.blit(fb, 0, 0)
        
        # Sovrascrive il messaggio in basso (Y=54 per non coprire troppo il disegno)
        # Usiamo un piccolo rettangolo nero dietro il testo se il logo è troppo pieno
        text_x = max(0, (128 - len(msg) * 8) // 2)
        self.display.fill_rect(0, 52, 128, 12, 0) # Pulisce la fascia del testo
        self.display.text(msg, text_x, 54, 1)     # Scrive il testo in bianco
        
        # Mostra lo schermo per un secondo e poi si pulisce lo schermo
        time.sleep(1)
        self.display.fill(0)
        self.display.show()
        
    # Funzione per il countdown 
    def countdown(self, title, sec):
        for i in range(sec, 0, -1):
            self.display.fill(0)
            self.frame()
            self.center(title, 5)
            self.center("Chiusura porta:", 25)
            self.center(str(i), 45)
            self.display.show()
            time.sleep(1)
            
    