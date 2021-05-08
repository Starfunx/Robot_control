from external.API_interface import Robot
from external.API_interface.TLF_API.component.Class_Pose2D import Pose2D

from robot_package.data_robot_creator import data_robot_creator


import time
import numpy as np

class RobotControl():
    def __init__(self, nom_fichier, robot_port, robot_bauderate):
        # creation des capteurs virtuels avec le yaml --> permet de convertir distacne en position
        self.robot = Robot(robot_port, robot_bauderate)
        _, self.dist_sensor = data_robot_creator(nom_fichier) # On ne récupère pas les points du robot
        
        self.goal = None
        self.robotPose = self.robot.get_pose()
        
        self.vitesse_angulaire = []
        self.vitesse_lineaire = []
        self.vitesse_roue_gauche = []
        self.vitesse_roue_droite = []

        self.time = time.time()


    def update_speed(self):
        # lecture
        # traitement
        # ecriture
        #######################
        # Recupe données
        #######################
        # update vitesse vitesse angulaire en fonction des obstacles
        # Recupére position --> driver robot
        self.robotPose = self.robot.get_pose()


        # Calculs
        dist_robot2goal = self.distance_robot2goal(self.goal)
        angle_robot2goal = self.angle_robot2goal(self.goal)
        angle_goal = self.goal.theta - self.robotPose.theta

        # Constantes
        krho = 0.2
        kalpha = 2

        dt = (time.time()- self.time)
        linear_speed = krho* dist_robot2goal / dt
        angular_speed = kalpha* angle_robot2goal / dt
        self.robot.set_speed(linear_speed, angular_speed)
        
        self.time = time.time()
        
        self.vitesse_angulaire.append(angular_speed)
        self.vitesse_lineaire.append(linear_speed)

        RL = self.robot.get_left_wheel_data()
        self.vitesse_roue_gauche.append(RL.measure)
        RR = self.robot.get_right_wheel_data()
        self.vitesse_roue_droite.append(RR.measure)
        # recup obstacle --> driver carte obstacle
        # Dist_sensir(obstacle) -> repère robot
        
        ##  # gestionnaire capteur virtuel
        ##  # robot transforme point dans le repère du robot (optionnel à garder en t^te)
 
        #
        ############################
        # Stratégie en fonction des données
        #############################
        # Calcul vitesse, vitesse angulaire pour aller au point B

        # update la vitesse avec le driver
        pass

    def set_goal(self, pose : Pose2D):
        self.goal = pose
        
    def distance_robot2goal(self, goal : Pose2D):
        dist_robot2goal = np.sqrt(
            np.power((self.robotPose.x - goal.x), 2) + np.power((self.robotPose.y - goal.y), 2))
        
        return dist_robot2goal

    def angle_robot2goal(self, goal: Pose2D):
        X = goal.x - self.robotPose.x
        Y = goal.y - self.robotPose.y
        
        angle_robot2goal = np.arctan2(Y, X) - self.robotPose.theta
        
        return angle_robot2goal

    def goal_reached(self):
        # critère dans un yaml
        distance = self.distance_robot2goal(self.goal)
        # print(distance)

        if distance < 20:
            return 1
        
        else:
            return 0
    





if __name__ == "__main__":
    nom_fichier = './config/robot_config.yaml'
    robot_port = "/dev/ttyACM0"
    robot_baurate = 115200
    robotcontrol = RobotControl(nom_fichier, robot_port, robot_baurate)

    robotcontrol.robot.set_pose(0, 0, 0)
    pose = Pose2D(1000, 1000, 0)
    robotcontrol.set_goal(pose)   

    robotcontrol.robot.enable_motors()
    try:
        while not(robotcontrol.goal_reached()):
            # Mettre 10 Hz
            robotcontrol.update_speed()

            

    except KeyboardInterrupt:
        print("stop")
        pass

    np.save("vitesse_angular", robotcontrol.vitesse_angulaire)
    np.save("vitesse_linear", robotcontrol.vitesse_lineaire)
    np.save("RL", robotcontrol.vitesse_roue_gauche)
    np.save("RR", robotcontrol.vitesse_roue_droite)

    robotcontrol.robot.set_speed(0, 0)
    robotcontrol.robot.disable_motors()
    

