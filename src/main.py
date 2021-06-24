from numpy.core.numeric import False_
from external.API_interface import Robot
from external.API_interface.TLF_API.component.Class_Pose2D import Pose2D

from robot_package.data_robot_creator import data_robot_creator
from external.sensor_board import CarteDetecteurObstacle


import time
import numpy as np
from shapely.geometry import MultiPoint, Polygon


class RobotControl():
    def __init__(self, nom_fichier, robot_port, robot_bauderate, carte_obstacle_port, carte_obstacle_bauderate):
        # creation des capteurs virtuels avec le yaml --> permet de convertir distacne en position
        _, self.dist_sensor, self.point_area_obstacle = data_robot_creator(nom_fichier) # On ne récupère pas les points du robot
        
        # création des zones de detections d'obstacle
        self.aire_avant_gauche = Polygon(self.point_area_obstacle[0])
        self.aire_avant_droite = Polygon(self.point_area_obstacle[1])

        # Création des cartes de communications
        self.robot = Robot(robot_port, robot_bauderate)

        if carte_obstacle_port != "":
            self.carteobstacle = CarteDetecteurObstacle(carte_obstacle_port, carte_obstacle_bauderate)
        
        # COnsigne
        self.consign_linear_speed = 0
        self.consign_angular_speed = 0


        # Positions utiles pour le robot
        self.goal = None
        self.liste_goal = None
        self.goal_save = []
        self.robotPose = self.robot.get_pose()
        
        # Sauvegarde de données pour les tracer
        self.consigne_vitesse_angulaire = []
        self.consigne_vitesse_lineaire = []
        self.mesure_vitesse_roue_gauche = []
        self.mesure_vitesse_roue_droite = []

        self.time = time.time()

        

    def update_speed(self):
        # lecture
        # traitement
        # ecriture
        #######################
        # Recupe données
        # update vitesse vitesse angulaire en fonction des obstacles
        # Recupére position --> driver robot
        self.robotPose = self.robot.get_pose()



        # Constantes
        krho = 0.2
        kalpha = 2
        flag_obstacle_droite = 0
        flag_obstacle_gauche = 0

        
        # recup obstacle --> driver carte obstacle
        # Transformation dans le repère du robot en même temps
        # liste_obstacle = self.carteobstacle.get_distance('A')
        liste_obstacle = []
        
        # Vérification des distance des capteurs
        for sensor, distance in zip(self.dist_sensor, liste_obstacle):
            # Liaison des distances réelles aux capteurs virtuels --> Connaitre position de l'obstacle
            sensor.set_dist(distance, 0)
            # Projection dans le repère du robot
            point_repère_robot = MultiPoint(np.transpose(sensor.get_obstacle_pose()))
            
            ### Vérification de la distance en fonctions des zones (droite ou gauche)
            if self.aire_avant_gauche.contains(point_repère_robot):
                print("Point à l'avant gauche")
                # flag_obstacle_gauche = True

            if self.aire_avant_droite.contains(point_repère_robot):
                print("Point à l'avant droite")
                # flag_obstacle_droite = True

            ## Si on a un obstalce dans les 2 zones, on passe directement à la suite 
            if (flag_obstacle_gauche and flag_obstacle_droite):
                # Emergency
                break

        ##################################
        ## Stratégie d'évitement vers la gauche ou la droite en focntion de l'obstacle

        # Gestion de la vitesse pour un obstacle dans les 2 zones --> Arret immédiat
        if (flag_obstacle_gauche and flag_obstacle_droite):
            print("------- LES DEUX CAPTEURS ----------")
            # Calculs
            self.consign_linear_speed = 0
            self.consign_angular_speed = 0

        # Gestion de 1 obstacle dans 1 seule zone (avance)
        elif (flag_obstacle_gauche or flag_obstacle_droite):
            self.consign_linear_speed = 0

            if (flag_obstacle_droite):
                print("change goal vers la gauche")
                self.consign_angular_speed += 0.1

            elif (flag_obstacle_gauche):
                print("change goal vers la droite")
                self.consign_angular_speed -= 0.1

        # Cas où il n'y a aucun obstacle, on calcule l'objectif
        else:
            # Calculs si il n'y a aucun obstacle
            dist_robot2goal = self.distance_robot2goal(self.goal)
            angle_robot2goal = self.angle_robot2goal(self.goal)
            angle_goal = self.goal.theta - self.robotPose.theta
            # angle_robot2goal -= angle_goal
    
            # Dist_sensir(obstacle) -> repère robot
            ############################
            # Stratégie en fonction des données
            # Calcul vitesse, vitesse angulaire pour aller au point B
            dt = (time.time() - self.time)
            
            self.consign_linear_speed = krho * dist_robot2goal / dt
            self.consign_angular_speed = kalpha * constrainAngle(angle_robot2goal) / dt
        
        
        ############################
        # update la vitesse avec le driver

        # on définit un polygone et on vérifie si les obstacles sont dans cet espcae
        # --> vitesse lineaire/angulaire à 0 --> strategie arret, plus tard strategie d'évitement
        #
        # Si la vitesse lineaire est positif : poly test devant le robot
        #
        # Si la vitesse lineaire est negative, poly derrère le robot
        #
        # FAIRE SCH2MA
        #
        
        self.robot.set_speed(self.consign_linear_speed, self.consign_angular_speed)
        self.time = time.time()


        # ############################
        # ## Affichage - sauvegarde des données
        # self.consigne_vitesse_angulaire.append(consign_angular_speed)
        # self.consigne_vitesse_lineaire.append(consign_linear_speed)

        # left_wheel = self.robot.get_left_wheel_data()
        # right_wheel = self.robot.get_right_wheel_data()
        # self.mesure_vitesse_roue_gauche.append(left_wheel.measure)
        # self.mesure_vitesse_roue_droite.append(right_wheel.measure)
    # END FUNCTION UPDATE SPEED


    def set_goal(self, pose ):
        self.liste_goal = pose
        self.goal = self.liste_goal.pop()
        
    def distance_robot2goal(self, goal : Pose2D):
        dist_robot2goal = np.sqrt(
            np.power((self.robotPose.x - goal.x), 2) + np.power((self.robotPose.y - goal.y), 2))
        
        return dist_robot2goal

    def angle_robot2goal(self, goal: Pose2D):
        X = goal.x - self.robotPose.x
        Y = goal.y - self.robotPose.y
        
        angle_robot2goal = constrainAngle( np.arctan2(Y, X) - self.robotPose.theta )
        # print("X =", X, "\tY =", Y, "\tAtan =", angle_robot2goal )  

        return angle_robot2goal

    def goal_reached(self):
        # critère dans un yaml
        distance = self.distance_robot2goal(self.goal)
        # print(distance)

        if ( (distance < 50) ):
            if ( len(self.liste_goal) ):
                print(distance)
                print("goal", self.goal.x, "; ", self.goal.y)
                self.goal = self.liste_goal.pop()
                print("changement goal")

                return 0

            else :
                print(distance)
                print("fin")
                return 1
        
        else:
            return 0
    

def constrainAngle(x):
    x = np.fmod(x + np.pi, 2*np.pi)
    if (x < 0):
        x += 2*np.pi

    return x - np.pi


if __name__ == "__main__":
    from module.tirette import Selection_zone

    nom_fichier = './config/robot_config.yaml'
    robot_port = "/dev/ttyACM0"
    robot_baurate = 115200

    carte_obstacle_port = ""
    carte_obstacle_bauderate = 115200

    robotcontrol = RobotControl(nom_fichier, robot_port, robot_baurate, carte_obstacle_port, carte_obstacle_bauderate)

    robotcontrol.robot.set_pose(0, 0, 0)
    pose = [Pose2D(1000, 0, 0),
            Pose2D(-1000, 0, 0)]
    
    robotcontrol.set_goal(pose[::-1])
    
    selection_zone = Selection_zone(14, 15)
    # selection_zone.wait_start_loop()

    print("Zone de départ :", selection_zone.zone)

    robotcontrol.robot.enable_motors()
    try:
        while not(robotcontrol.goal_reached()):
            # Mettre 10 Hz
            robotcontrol.update_speed()

            

    except KeyboardInterrupt:
        print("stop")
        pass

    # np.save("vitesse_angular", robotcontrol.consigne_vitesse_angulaire)
    # np.save("vitesse_linear", robotcontrol.consigne_vitesse_lineaire)
    # np.save("left_wheel", robotcontrol.mesure_vitesse_roue_gauche)
    # np.save("right_wheel", robotcontrol.mesure_vitesse_roue_droite)

    robotcontrol.robot.set_speed(0, 0)
    robotcontrol.robot.disable_motors()
    

