from re import S
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

        # if carte_obstacle_port != "":
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
        self.dist_obstacle = 500


    def update_speed(self):
        # lecture
        # traitement
        # ecriture
        #######################
        # Recupe données
        # update vitesse vitesse angulaire en fonction des obstacles
        # Recupére position --> driver robot
        self.robotPose = self.robot.get_pose()
        # print(self.robotPose)



        # Constantes
        krho = 0.015
        kalpha = 0.07
        flag_obstacle_droite = 0
        flag_obstacle_gauche = 0

        
        # recup obstacle --> driver carte obstacle
        # Transformation dans le repère du robot en même temps
        liste_obstacle = self.carteobstacle.get_distance('A')
        # print(liste_obstacle)
        # liste_obstacle = []
        
        # # Vérification des distance des capteurs
        # for sensor, distance in zip(self.dist_sensor, liste_obstacle):
        #     # Liaison des distances réelles aux capteurs virtuels --> Connaitre position de l'obstacle
        #     sensor.set_dist(distance, 0)
        #     # Projection dans le repère du robot
        #     point_repère_robot = MultiPoint(np.transpose(sensor.get_obstacle_pose()))
            
        #     ### Vérification de la distance en fonctions des zones (droite ou gauche)
        #     if self.aire_avant_gauche.contains(point_repère_robot):
        #         print("Point à l'avant gauche")
        #         flag_obstacle_gauche = True

        #     if self.aire_avant_droite.contains(point_repère_robot):
        #         print("Point à l'avant droite")
        #         flag_obstacle_droite = True

        #     ## Si on a un obstalce dans les 2 zones, on passe directement à la suite 
        #     if (flag_obstacle_gauche or flag_obstacle_droite):
        #         # Emergency
        #         break

        if (liste_obstacle[0] < self.dist_obstacle or liste_obstacle[1] < self.dist_obstacle):
            # print("Obtacle detecté")
            self.consign_linear_speed = 0
            self.consign_angular_speed = 0
            self.robot.disable_motors()
            print("obstacle detecté")



        ##################################
        ## Stratégie d'évitement vers la gauche ou la droite en focntion de l'obstacle

        # Gestion de la vitesse pour un obstacle dans les 2 zones --> Arret immédiat
        # if (flag_obstacle_gauche or flag_obstacle_droite):
        #     print("------- LES DEUX CAPTEURS ----------")
        #     # Calculs

        # # Gestion de 1 obstacle dans 1 seule zone (avance)
        # elif (flag_obstacle_gauche or flag_obstacle_droite):
        #     self.consign_linear_speed = 0

        #     if (flag_obstacle_droite):
        #         print("change goal vers la gauche")
        #         self.consign_angular_speed += 0.1

        #     elif (flag_obstacle_gauche):
        #         print("change goal vers la droite")
        #         self.consign_angular_speed -= 0.1

        # Cas où il n'y a aucun obstacle, on calcule l'objectif
        else:
            # Calculs si il n'y a aucun obstacle
            self.robot.enable_motors()
            dist_robot2goal = self.distance_robot2goal(self.goal)
            alpha = self.angle_robot2goal(self.goal) # alpha
            # angle_robot2goal -= angle_goal
    
            # Dist_sensir(obstacle) -> repère robot
            ############################
            # Stratégie en fonction des données
            # Calcul vitesse, vitesse angulaire pour aller au point B
            # if abs(alpha) > 1.0:
            #     self.consign_linear_speed = 0
            #     self.consign_angular_speed = kalpha * alpha
            # else:
            self.consign_linear_speed = krho * dist_robot2goal * np.cos(alpha)
            # self.consign_linear_speed = 0.0
            
            self.consign_angular_speed = -kalpha * alpha
        
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
        # print(self.consign_linear_speed, self.consign_angular_speed)
        self.robot.set_speed(self.consign_linear_speed, self.consign_angular_speed)


        # ############################
        # ## Affichage - sauvegarde des données
        # self.consigne_vitesse_angulaire.append(consign_angular_speed)
        # self.consigne_vitesse_lineaire.append(consign_linear_speed)

        # left_wheel = self.robot.get_left_wheel_data()
        # right_wheel = self.robot.get_right_wheel_data()
        # self.mesure_vitesse_roue_gauche.append(left_wheel.measure)
        # self.mesure_vitesse_roue_droite.append(right_wheel.measure)
    # END FUNCTION UPDATE SPEED

    def set_angle(self, angle):
        angle = constrainAngle(self.robotPose.theta + angle)

        angle_robot2angle = angle - self.robotPose.theta

        #0.85 measure 0.00 command 0.35

        while (abs(angle_robot2angle) > 0.7):
            kalpha = angle_robot2angle *0.5
            # print(kalpha)

            self.consign_linear_speed = 0
            self.consign_angular_speed = kalpha 

            self.robot.set_speed(self.consign_linear_speed, self.consign_angular_speed)

            self.robotPose = self.robot.get_pose()
            angle_robot2angle = angle - self.robotPose.theta

        self.consign_linear_speed, self.consign_angular_speed = 0, 0
        self.robot.set_speed(self.consign_linear_speed, self.consign_angular_speed)


    def set_goal(self, pose : Pose2D):
        self.liste_goal = []
        self.goal = pose

    def set_liste_goal(self, pose ):
        self.liste_goal = pose
        self.goal = self.liste_goal.pop()
        
    def distance_robot2goal(self, goal : Pose2D):
        dist_robot2goal = np.sqrt(
            np.power((self.robotPose.x - goal.x), 2) + np.power((self.robotPose.y - goal.y), 2))
        
        return dist_robot2goal

    def angle_robot2goal(self, goal: Pose2D):
        X = goal.x - self.robotPose.x
        Y = goal.y - self.robotPose.y
        
        tmp_angle = np.arctan2(Y, X) - self.robotPose.theta
        alpha = constrainAngle( tmp_angle )

        print(alpha)
        print(alpha)
        print(alpha)

        return alpha

    def goal_reached(self):
        # critère dans un yaml
        if self.goal == None:
            return 1

        distance = self.distance_robot2goal(self.goal)
        # print(distance)

        if ( (distance < 50) ):
            if ( len(self.liste_goal) > 0 ):
                print(distance)
                print("goal", self.goal.x, "; ", self.goal.y)
                self.goal = self.liste_goal.pop()
                print("changement goal")
                print("goal", self.goal.x, "; ", self.goal.y)

                return 0

            else :
                print(distance)
                print("Déplacement terminé")
                return 1
        
        else:
            return 0
    

def constrainAngle(x):
    x = np.fmod(x + np.pi, 2*np.pi)
    if (x < 0):
        x += 2*np.pi

    return x - np.pi