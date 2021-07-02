from gpiozero import Servo
from time import sleep

# terme de correction pour faire 180°
maxPW = (1.76 +0.45)/1000
minPW = (0.96-0.45)/1000

servo_drapeau_pin = 22
servo_drapeau = Servo(servo_drapeau_pin, min_pulse_width=minPW, max_pulse_width=maxPW)

while True:
    servo_drapeau.mid()
    print("mid")
    sleep(0.5)
    servo_drapeau.min()
    print("min")
    sleep(1)
    servo_drapeau.mid()
    print("mid")
    sleep(0.5)
    servo_drapeau.max()
    print("max")
    sleep(1)
