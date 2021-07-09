from external.API_interface.TLF_API.component.Class_Pose2D import Pose2D
from gpiozero.pins.rpigpio import RPiGPIOFactory
from gpiozero import Servo, Device, LED
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
        print(robotcontrol.robotPose)

    robotcontrol.robot.set_speed(0, 0)
    robotcontrol.robot.disable_motors()

# Déplacement angulaire


def go_to_angle(angle: float):
    global robotcontrol
    robotcontrol.robot.enable_motors()

    def mod(a, n):
        return (a % n+n) % n

    Kalpha = 0.05

    tmax = time.time() + 10.0

    Beta = 10
    while (abs(Beta) >= 0.05) and time.time() < tmax:
        pose = robotcontrol.robot.get_pose()
        theta = pose.theta

        # calcul angle Beta
        Beta = angle - theta
        Beta = mod(Beta + np.pi, 2*np.pi) - np.pi
        # print(Beta)
        ang_speed = -Kalpha * Beta
        print(robotcontrol.robot.get_pose())

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
def servo_danse1():
    """ Max = 1 | Mid = 0 | Min = -1 """
    global servo_cote
    servo_cote.servo_droite.value = -1
    servo_cote.servo_gauche.value = -1
    time.sleep(1)

    servo_cote.servo_droite.value = 1
    servo_cote.servo_gauche.value = -1
    time.sleep(1)

    servo_cote.servo_droite.value = -1
    servo_cote.servo_gauche.value = -1
    time.sleep(1)
    
    servo_cote.servo_droite.value = -1
    servo_cote.servo_gauche.value = 1
    time.sleep(1)
    
    servo_cote.servo_droite.value = 1
    servo_cote.servo_gauche.value = 1
    time.sleep(1)



def drapeau(valeur):
    """ levée = 1 \ -0.5 """
    global servo_drapeau
    print("lever drapeau", valeur)
    servo_drapeau.value = valeur


def vitesse(v_lin: float, v_ang: float, temps: float):
    time.sleep(0.5)
    robotcontrol.robot.enable_motors()
    robotcontrol.robotPose = robotcontrol.robot.get_pose()  # update controller pos

    t0 = time.time()
    while time.time() < t0 + temps:
        robotcontrol.robot.set_speed(v_lin, v_ang)
        print(robotcontrol.robot.get_pose())

    robotcontrol.robot.disable_motors()


def stop_and_wait(t):
    stop_robot()
    time.sleep(t)


point_depart = Pose2D(0, 0, 0)
point_depart = Pose2D(250, 1200, np.pi/2)
###
# Creation des tasks
#################################################################################
# task_go_point_phare = Task(lambda: go_to(Pose2D(250, 1600)), "task_go_point_phare")
# task_stop1 = Task(lambda: stop_and_wait(2))

task_servo_cote = Task(servo_danse1, "task_orienter_point_inter")
# task_point_MAH = Task(lambda: go_to(Pose2D(170, 170)), "task_point_MAH")

# task_orienter_MAH = Task(lambda: go_to_angle(0), "task_orienter_MAH")
# # Bras sur les côtés

# task_deploye_servo = Task(lambda : servo(1), "task_deploye_servo")
# task_point_fin_MAH = Task(lambda: go_to(Pose2D(750, 170)), "task_point_fin_MAH")
# task_retract_servo = Task(lambda : servo(-1), "task_retract_servo")


# Drapeau
task_lever_drapeau = Task(lambda: drapeau(1), "task_lever_drapeau")
task_baisser_drapeau = Task(lambda: drapeau(-0.5), "task_baisser_drapeau")

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

robotcontrol = RobotControl(
    nom_fichier, robot_port, robot_baurate, carte_obstacle_port, carte_obstacle_bauderate)
robotcontrol.robot.reset_board()
for _ in range(10):
    robotcontrol.robot.set_pose(
        point_depart.x, point_depart.y, point_depart.theta)
robotcontrol.robot.set_pid_left(0.15, 0.024,  0.006)
robotcontrol.robot.set_pid_right(0.15, 0.024,  0.006)
print(robotcontrol.robot.get_pose())

time.sleep(0.5)


###########################################
# Creation des modules
###########################################
# Pour les différents servos
Device.pin_factory = RPiGPIOFactory()

# Module servo sur le coté
servo_cote = Servo_cote(27, 17)
servo_cote.move_mirror(-1)


servo_drapeau = Servo(22)
servo_drapeau.value = -0.5


#########################################
# Main programme
##########################################
selection_zone = Selection_zone(14, 15)
selection_zone.wait_start_loop()  # TIRETTE

time.sleep(5)

t0 = time.time()
Tmax = 100  # secondes
T_Drapeau = Tmax-5  # secondes
# start
t = 0
# ecran = Ecran()

tmp_task = Task(lambda: go_to(Pose2D(1000, 0)))
tmp_task = Task(lambda: go_to(Pose2D(1000, 0)))
# score = 0
# Main loop
while True:
    servo_danse1()
    drapeau(1)
    servo_danse1()
    drapeau(-0.5)
    vitesse(0, (0.5-np.random.rand())*10, 2)

    t = time.time() - t0

print("FIN Programme")
time.sleep(10)
