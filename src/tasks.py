import time
import numpy as np
from re import S
from numpy.core.numeric import False_
from external.API_interface import Robot
from external.API_interface.TLF_API.component.Class_Pose2D import Pose2D

from robot_package.data_robot_creator import data_robot_creator
from external.sensor_board import CarteDetecteurObstacle

##############################################################
# Temps
def wait(t):
    time.sleep(t)


#################################################################
# servos
def servo(servo_cote, valeur):
    """ Max = 1 | Mid = 0 | Min = -1 """
    servo_cote.move_mirror(valeur)
    print("deployer servo done")


def drapeau(servo_drapeau, valeur):
    """ levée = 1 \ -0.5 """
    print("lever drapeau", valeur)
    servo_drapeau.value = valeur


#################################################################
# Deplacements


def stop_robot(robotcontrol):
    robotcontrol.robot.set_speed(0, 0)
    robotcontrol.robot.disable_motors()


def vitesse(robotcontrol, v_lin: float, v_ang:float, temps:float):
    robotcontrol.robot.enable_motors()
    robotcontrol.robotPose = robotcontrol.robot.get_pose() # update controller pos

    t0 = time.time()
    while time.time() < t0 + temps:
        robotcontrol.robot.set_speed(v_lin, v_ang)
        print(robotcontrol.robot.get_pose())

    robotcontrol.robot.disable_motors()

def stop_and_wait(t):
    stop_robot()
    time.sleep(t)


def constrainAngle(x):
    x = np.fmod(x + np.pi, 2*np.pi)
    if (x < 0):
        x += 2*np.pi

    return x - np.pi


def move_to_pose(robotcontrol, x_goal, y_goal, theta_goal):
    """
    rho is the distance between the robot and the goal position
    alpha is the angle to the goal relative to the heading of the robot
    beta is the angle between the robot's position and the goal position plus the goal angle

    Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards the goal
    Kp_beta*beta rotates the line so that it is parallel to the goal angle
    """
    Kp_rho = 0.0055
    Kp_alpha = 0.005
    Kp_beta = -0.0003

    pose =  robotcontrol.robot.get_pose() 

    x = pose.x
    y = pose.y
    theta = pose.theta

    x_diff = x_goal - x
    y_diff = y_goal - y

    x_traj, y_traj = [], []

    rho = np.hypot(x_diff, y_diff)



    robotcontrol.robot.enable_motors()


    while rho > 5:
        x_diff = x_goal - x
        y_diff = y_goal - y

        # x_diff, y_diff = y_diff, x_diff

        # Restrict alpha and beta (angle differences) to the range
        # [-pi, pi] to prevent unstable behavior e.g. difference going
        # from 0 rad to 2*pi rad with slight turn
        # print(f"x_diff:{x_diff} y_diff:{y_diff}")

        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff) - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi
        # print(f"alpha:{alpha}")
        v = Kp_rho * rho
        w = Kp_alpha * alpha + Kp_beta * beta
        print(f"rho:{rho} \t alpha:{alpha} beta:{beta}\t v:{v} \t w:{w}")

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v

        #update robot
        robotcontrol.robot.set_speed(v, -w)
        
        pose =  robotcontrol.robot.get_pose() 
        x = pose.x
        y = pose.y
        theta = pose.theta
    
    robotcontrol.robot.disable_motors()
