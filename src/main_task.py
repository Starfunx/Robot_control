from external.API_interface.TLF_API.component.Class_Pose2D import Pose2D
from gpiozero.pins.rpigpio import RPiGPIOFactory
from gpiozero import Servo, Device
from module.servo_cote import Servo_cote
from module.tirette import Selection_zone

# import matplotlib
# matplotlib.use('Agg')
# import matplotlib.pyplot as plt
# from external.affichage_ecran import Ecran


from robot_package.RobotControl import RobotControl
from task_manager import Task
import time

import numpy as np

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

# Déplacement angulaire
def go_to_angle(angle: float):
    global robotcontrol
    robotcontrol.robot.enable_motors()


    def mod(a, n):
        return (a%n+n)%n

    Kalpha = 0.05

    Beta = 10
    while abs(Beta) >= 0.1:
        pose = robotcontrol.robot.get_pose()
        theta = pose.theta

        # calcul angle Beta
        Beta = angle - theta
        Beta = mod(Beta + np.pi, 2*np.pi) - np.pi
        # print(Beta)
        ang_speed = -Kalpha * Beta

        robotcontrol.robot.set_speed(0.0, ang_speed)

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
    """ levée = 1 \ -0.5 """
    global servo_drapeau
    print("lever drapeau", valeur)
    servo_drapeau.value = valeur

def vitesse(v_lin: float, v_ang:float, temps:float):
    robotcontrol.robot.enable_motors()

    t0 = time.time()
    while time.time() < t0 + temps:
        robotcontrol.robot.set_speed(v_lin, v_ang)

    robotcontrol.robot.disable_motors()

# def display_ecran():
#     global ecran
#     global score
#     ecran.update_text(score)
#     plt.pause(1.0/30)


###
# Creation des tasks
###
# De déplacement
point_depart = Pose2D(250, 1310, np.pi/2)
task_go_point_phare = Task(lambda: go_to(Pose2D(220, 1840)))
task_tourner_vers_phare = Task(lambda: go_to_angle(np.pi/2))
task_vitesse_phare = Task(lambda: vitesse(10, 0, 5))

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
task_baisser_drapeau = Task(lambda : drapeau(-0.5))

# task_affichage_ecran = Task(display_ecran)
############################################
# Création du robot
############################################

# Carte moteur
nom_fichier = './config/robot_config.yaml'
robot_port = "/dev/ttyACM0"
robot_baurate = 115200

# Carte obstacle
carte_obstacle_port = "/dev/ttyUSB0"
carte_obstacle_bauderate = 115200

robotcontrol = RobotControl(nom_fichier, robot_port, robot_baurate, carte_obstacle_port, carte_obstacle_bauderate)
robotcontrol.robot.set_pose(point_depart.x, point_depart.y, point_depart.theta)
# robotcontrol.robot.set_pid_left( 0.162, 0.024,  0.006)
# robotcontrol.robot.set_pid_right(0.162, 0.024,  0.006)

###########################################
# Creation des modules
###########################################
# Pour les différents servos
Device.pin_factory = RPiGPIOFactory()

# Module servo sur le coté
servo_cote = Servo_cote(27, 17)


servo_drapeau = Servo(22)
servo_drapeau.value = -0.5


#########################################
# Main programme
##########################################
selection_zone = Selection_zone(14, 15)
# selection_zone.wait_start_loop()


t0 = time.time()
Tmax = 20 #secondes
T_Drapeau = Tmax-5 #secondes
# start
t = 0

# ecran = Ecran()
# score = 0
# Main loop
while t < Tmax:
    # print(t)

    task_vitesse_phare.start()




    if t > T_Drapeau:
        task_lever_drapeau.start()

    t = time.time() - t0



# End loop
task_lever_drapeau.stop()
task_vitesse_phare.stop()


for _ in range(150):
    stop_robot() 

time.sleep(2)
print("FIN Programme")


