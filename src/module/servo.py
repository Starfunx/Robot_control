
from gpiozero.pins.rpigpio import RPiGPIOFactory
from gpiozero import Servo, Device
from time import sleep

Device.pin_factory = RPiGPIOFactory()


def servo_init(pin_servo = 17, pin_servo2 = 27):
    servo = Servo(pin_servo)
    servo2 = Servo(pin_servo2)

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

