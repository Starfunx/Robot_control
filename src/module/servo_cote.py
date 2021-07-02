from gpiozero.pins.rpigpio import RPiGPIOFactory
from gpiozero import Servo, Device


class Servo_cote():
    def __init__(self, servo_pin_droite, servo_pin_gauche):
        Device.pin_factory = RPiGPIOFactory()
        self.servo_gauche = Servo(servo_pin_gauche)
        self.servo_droite = Servo(servo_pin_droite)

    def move_min(self):
        self.servo_droite.max()
        self.servo_gauche.min()
    
    def move_max(self):
        self.servo_droite.min()
        self.servo_gauche.max()
    
    def move_mirror(self, valeur):
        self.servo_droite.value = valeur
        self.servo_gauche.value = -valeur
    
    def move(self, valeur_droite, valeur_gauche):
        self.servo_droite.value = valeur_droite
        self.servo_gauche.value = valeur_gauche




if __name__ == "__main__":
    
    from time import sleep
    
    pin_servo_gauche = 17
    pin_servo_droite = 27

    servo_cote = Servo_cote(pin_servo_droite, pin_servo_gauche)

    while True:
        # servo_cote.move_max()
        servo_cote.move_mirror(1)
        sleep(2)
        servo_cote.move_mirror(-1)
        # servo_cote.move_min()
        sleep(2)

