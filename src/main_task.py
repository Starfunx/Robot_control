from external.API_interface.TLF_API.component.Class_Pose2D import Pose2D
from gpiozero.pins.rpigpio import RPiGPIOFactory
from gpiozero import Servo, Device
from module.servo_cote import Servo_cote
from module.tirette import Selection_zone


from robot_package.RobotControl import RobotControl
from task_manager import Task
import time



#########################################
# Creation de la stratégie
#########################################

# Déplacement
def go_to(goal: Pose2D):
    global robotcontrol
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


# Attente
def wait(t):
    time.sleep(t)


# Modules
def servo(valeur):
    """ Max = 1 | Mid = 0 | Min = -1 """
    global servo_cote
    servo_cote.move_mirror(valeur)
    time.sleep(0.1)


def drapeau(valeur):
    """ Max = 1 | Mid = 0 | Min = -1 """
    global servo_drapeau
    servo_drapeau.value = valeur



###
# Creation des tasks
###
# De déplacement
task_deplacer_1000 = Task(lambda : go_to(Pose2D(1000, 0, 0)))
task_deplacer_0 = Task(lambda: go_to(Pose2D(0, 0, 0)))
task_stop_robot = Task(stop_robot)

# Attente
task_wait = Task(lambda : wait(2))
task_wait2 = Task(lambda: wait(5))
task_wait_deplacement = Task(lambda : wait(2))

### Modules
# Bras sur les côtés
task_deploye_servo = Task(lambda : servo(1))
task_retract_servo = Task(lambda : servo(-1))

# Drapeau
task_lever_drapeau = Task(lambda : drapeau(1))
task_baisser_drapeau = Task(lambda : drapeau(-1))

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

# Module pour le drapeau
maxPW = (1.7+0.45)/1000      # terme de correction pour faire 180°
minPW = (1.0-0.45)/1000      # terme de correction pour faire 180°
servo_drapeau = Servo(22, min_pulse_width=minPW, max_pulse_width=maxPW)
servo_drapeau.min()


#########################################
# Main programme
##########################################
selection_zone = Selection_zone(14, 15)
# selection_zone.wait_start_loop()


Tmax = 15.0 #secondes
# start
t0 = time.time()
t = t0
# Main loop
while t < t0+Tmax:

    task_deplacer_1000.start()
    task_wait.start()
    task_deploye_servo.start()

    if task_deplacer_1000.done:
        task_wait_deplacement.start()
        
        if task_deploye_servo.done:
            task_retract_servo.start()
    
    # if task_deplacer_0.done:
    #     task_lever_drapeau.start()
    
    if task_wait_deplacement.done:
        task_deplacer_0.start()
    
    if task_deplacer_0.done:
        task_stop_robot.start()

    # if task_deploye_servo.done:
    #     task_wait.start()
    
    if task_wait.done:
        task_lever_drapeau.start()

    if task_lever_drapeau.done:
        task_wait2.start()
    
    if task_wait2.done:
        task_baisser_drapeau.start()

    t = time.time()



# End loop



task_deplacer_1000.stop()
task_deplacer_0.stop()
task_deploye_servo.stop()
task_retract_servo.stop()
task_wait.stop()
task_wait2.stop()
task_wait_deplacement.stop()
task_baisser_drapeau.stop()
task_lever_drapeau.stop()
task_stop_robot.stop()


stop_robot() 

time.sleep(2)
print("FIN Programme")


