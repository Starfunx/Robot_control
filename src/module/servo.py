
from gpiozero.pins.rpigpio import RPiGPIOFactory
from gpiozero import Servo, Device
from time import sleep

Device.pin_factory = RPiGPIOFactory()


def servo_init():
    servo = Servo(17)
    servo2 = Servo(27)

    SERVO = [servo, servo2]
    servo_move_min(SERVO)

    return SERVO


def servo_move_min(SERVO):
    servo, servo2 = SERVO[0], SERVO[1]
    servo.min()
    servo2.max()

def servo_move_max(SERVO):
    servo, servo2 = SERVO[0], SERVO[1]
    servo.max()
    servo2.min()


if __name__ == "__main__":

    SERVO = servo_init()    
    while True:
        servo_move_max(SERVO)
        sleep(2)
        servo_move_min(SERVO)
        sleep(2)

