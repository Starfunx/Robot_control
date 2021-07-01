from external.API_interface.TLF_API.component.Class_Pose2D import Pose2D
from gpiozero.pins.rpigpio import RPiGPIOFactory
from gpiozero import Servo, Device
from module.servo_cote import Servo_cote


from robot_package.RobotControl import RobotControl
from task_manager import Task
import time



#########################################
# Creation de la stratégie
#########################################

def deplacer_1000():
    global robotcontrol
    goal = Pose2D(1000, 0, 0)
    robotcontrol.set_goal(goal)

    robotcontrol.robot.enable_motors()

    while not(robotcontrol.goal_reached()):
        robotcontrol.update_speed()
    
    robotcontrol.robot.set_speed(0, 0)
    robotcontrol.robot.disable_motors()


def deplacer_0():
    global robotcontrol
    goal = Pose2D(0, 0, 0)
    robotcontrol.set_goal(goal)

    robotcontrol.robot.enable_motors()

    while not(robotcontrol.goal_reached()):
        robotcontrol.update_speed()
    
    robotcontrol.robot.set_speed(0, 0)
    robotcontrol.robot.disable_motors()
    

def stop_robot():
    global robotcontrol
    robotcontrol.robot.set_speed(0, 0)
    robotcontrol.robot.disable_motors()

def deploye_servo():
    global servo_cote
    servo_cote.move_max()
    time.sleep(0.1)


def retract_servo():
    global servo_cote
    servo_cote.move_min()
    time.sleep(0.1)

def wait():
    time.sleep(0.3)

def wait2():
    time.sleep(2)


# Creation des tasks
task_deplacer_1000 = Task(deplacer_1000)
task_deplacer_0 = Task(deplacer_0)

task_deploye_servo = Task(deploye_servo)
task_retract_servo = Task(retract_servo)
task_wait = Task(wait)
task_wait_deplacement = Task(wait2)

############################################
# Création du robot
############################################

# Carte moteur
nom_fichier = './config/robot_config.yaml'
robot_port = "/dev/ttyACM0"
robot_baurate = 115200

# Carte obstacle
carte_obstacle_port = ""
carte_obstacle_bauderate = 115200

robotcontrol = RobotControl(nom_fichier, robot_port, robot_baurate, carte_obstacle_port, carte_obstacle_bauderate)


###########################################
# Creation des modules
###########################################

# Module servo sur le coté
Device.pin_factory = RPiGPIOFactory()
servo_cote = Servo_cote(27, 17)


#########################################
# Main programme
##########################################



Tmax = 15.0 #secondes
# start
t0 = time.time()
t = t0
# Main loop
while t < t0+Tmax:

    task_deplacer_1000.start()
    task_deploye_servo.start()

    if task_deplacer_1000.done:
        task_wait_deplacement.start()
    
    if task_wait_deplacement.done:
        task_deplacer_0.start()
    

    if task_deploye_servo.done:
        task_wait.start()
    
    if task_wait.done:
        task_retract_servo.start()

    t = time.time()

# End loop



task_deplacer_1000.stop()
task_deplacer_0.stop()
task_deploye_servo.stop()
task_retract_servo.stop()
task_wait.stop()
task_wait_deplacement.stop()

stop_robot() 


time.sleep(2)
print("FIN Programme")


