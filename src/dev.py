import time
import numpy as np

from external.API_interface.TLF_API.component.Class_Pose2D import Pose2D
from gpiozero.pins.rpigpio import RPiGPIOFactory
from gpiozero import Servo, Device, LED

from module.servo_cote import Servo_cote
from module.tirette import Selection_zone
from robot_package.RobotControl import RobotControl
from task_manager import Task
from config import *
from tasks import *

#########################################
# Creation de la stratégie
#########################################
point_depart = Pose2D(0, 0, 0)
# point_depart = Pose2D(250, 1200, np.pi/2)
###
# Creation des tasks
#################################################################################
# servo cote
task_deploye_servo = Task(lambda : servo(servo_cote, 1), "task_deploye_servo")
task_retract_servo = Task(lambda : servo(servo_cote, -1), "task_retract_servo")
# Drapeau
task_lever_drapeau = Task(lambda : drapeau(servo_drapeau, 1), "task_lever_drapeau")
task_baisser_drapeau = Task(lambda : drapeau(servo_drapeau, -0.5), "task_baisser_drapeau")

############################################
# Création du robot
############################################

robotcontrol = RobotControl(nom_fichier, robot_port, robot_baurate, carte_obstacle_port, carte_obstacle_bauderate)
robotcontrol.robot.reset_board()
for _ in range (10):
    robotcontrol.robot.set_pose(point_depart.x, point_depart.y, point_depart.theta)
robotcontrol.robot.set_pid_left( 0.15, 0.027,  0.006)
robotcontrol.robot.set_pid_right(0.15, 0.025,  0.006)
print(robotcontrol.robot.get_pose())


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
# selection_zone.wait_start_loop() # TIRETTE
print(selection_zone.zone)

if selection_zone.zone == "bleu" :
    pass
else:
    pass

t0 = time.time()
Tmax = 100 #secondes
T_Drapeau = Tmax-5 #secondes
# start
t = 0
# ecran = Ecran()

task1 = Task(lambda: move_to_pose(robotcontrol, 1000, 0.0, np.pi))

#############################################
# Main loop
while t < Tmax:
    task1.start()






#############################################
    if t > T_Drapeau:
        task_lever_drapeau.start()


    t = time.time() - t0


for _ in range(150):
    stop_robot() 

print("FIN Programme")
time.sleep(10)


