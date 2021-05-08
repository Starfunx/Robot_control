from external.API_interface import Robot
from external.API_interface.TLF_API.component.Class_Pose2D import Pose2D

from robot_package.data_robot_creator import data_robot_creator


import time
import numpy as np

class RobotControl(Robot):
    def __init__(self, nom_fichier, robot_port, robot_bauderate):
        # creation des capteurs virtuels avec le yaml --> permet de convertir distacne en position
        Robot.__init__(self, robot_port, robot_bauderate)
        _, self.dist_sensor = data_robot_creator(nom_fichier) # On ne récupère pas les points du robot
        self.goal = Pose2D()
        self.dist_robot2goal = 0
        
        self.time = time.time()
        pass


    def update_speed(self):
        # lecture
        # traitement
        # ecriture
        #######################
        # Recupe données
        #######################
        # update vitesse vitesse angulaire en fonction des obstacles
        # Recupére position --> driver robot
        self.robotPose = self.get_pose()
        dist_robot2goal = self.distance_robot2goal()

        dt = time.time()- self.time
        speed = dist_robot2goal / dt
        self.set_speed(speed, 0)

        self.time = time.time()
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

    def set_goal(self, x, y, theta):
        self.goal.x = x
        self.goal.y = y
        self.goal.theta = theta
    
    def distance_robot2goal(self):
        self.dist_robot2goal = np.sqrt(
            np.power((self.robotPose.x - self.goal.x), 2) + np.power((self.robotPose.y - self.goal.y), 2))
        
        return self.dist_robot2goal

    def goal_reached(self):
        # critère dans un yaml
        distance = self.distance_robot2goal()
        print(distance)

        if distance < 5:
            return 1
        
        else:
            return 0
    





if __name__ == "__main__":
    nom_fichier = './config/robot_config.yaml'
    robot_port = "/dev/ttyACM0"
    robot_baurate = 115200
    robotcontrol = RobotControl(nom_fichier, robot_port, robot_baurate)

    robotcontrol.set_pose(0, 0, 0)
    robotcontrol.set_goal(300, 0, 0)

    robotcontrol.enable_motors()
    try:
        while not(robotcontrol.goal_reached()):
            robotcontrol.update_speed()

    except KeyboardInterrupt:
        print("stop")
        pass

    robotcontrol.disable_motors()
    

