from external.API_interface.TLF_API.component.Class_Pose2D import Pose2D
from module.servo import servo_init, servo_move_max, servo_move_min


from robot_package.RobotControl import RobotControl
from task_manager.task_manager import Task
import time

def deplacer1():
    global robotcontrol
    goal = [Pose2D(1000, 0, 0)]
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
    global SERVO
    servo_move_max(SERVO)
    time.sleep(2)


def retract_servo():
    global SERVO
    servo_move_min(SERVO)
    time.sleep(2)


nom_fichier = './config/robot_config.yaml'
robot_port = "/dev/ttyACM0"
robot_baurate = 115200

carte_obstacle_port = ""
carte_obstacle_bauderate = 115200

robotcontrol = RobotControl(nom_fichier, robot_port, robot_baurate, carte_obstacle_port, carte_obstacle_bauderate)
SERVO = servo_init()

task_deplacer1 = Task(deplacer1)
task_servo1 = Task(deploye_servo)
task_servo2 = Task(retract_servo)

Tmax = 20.0 #secondes
# start
t0 = time.time()
t = t0
# Main loop
while t < t0+Tmax:

    task_deplacer1.start()
    

    if task_deplacer1.done:
        task_servo1.start()
    
    if task_servo1.done:
        task_servo2.start()

    t = time.time()

task_deplacer1.stop()
task_servo1.stop()
task_servo2.stop()

stop_robot() 


time.sleep(2)
print("FIN Programme")


