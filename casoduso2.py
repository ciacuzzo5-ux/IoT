from machine import Pin, PWM, I2C
import time
from ssd1306 import SSD1306_I2C

LED_GREEN_PIN = 2 #gpio2: led di stato
SERVO_PIN     = 19 #gpio19 pin pwm stabile per il servo
IR_SENSOR_PIN = 34 #
MOTION_PIN    = 35

# valori prova per servo (duty 0–1023)
SERVO_OPEN  = 77   # circa 90°
SERVO_CLOSE = 40   # circa 0°

I2C_SDA = 21
I2C_SCL = 22

led_green = Pin(LED_GREEN_PIN, Pin.OUT)
led_green.value(0)


servo = PWM(Pin(SERVO_PIN))
servo.freq(50)  # frequenza standard servo

ir_sensor = Pin(IR_SENSOR_PIN, Pin.IN)
motion_sensor = Pin(MOTION_PIN, Pin.IN)

i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA))
oled = SSD1306_I2C(128, 64, i2c)

def unlock_door():
    time.sleep(0.5) #piccolo ritardo all'avvio
    #primo passo: accende LED verde
    led_green.value(1)

    # secondo passo: disattiva i sensori
    motion_sensor.init(mode=Pin.IN)
    ir_sensor.init(mode=Pin.IN)

    # terzo passo: apre la porta ruotando il servo
    servo.duty(SERVO_OPEN)
    time.sleep(1)

    # quarto passo:  mostra messaggio su OLED
    oled.fill(0)
    oled.text("Accesso", 10, 20)
    oled.text("autorizzato", 10, 35)
    oled.show()

# posizione iniziale del servo (chiuso)
servo.duty(SERVO_CLOSE)
time.sleep(0.5)


unlock_door()

