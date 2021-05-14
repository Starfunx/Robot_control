if __name__ == "__main__":
    # Pour executer le code en local et avoir les bon import
    import sys, os
    sys.path.append(os.getcwd() + "\\src")

from utils.yaml_utils import yaml_data_import
from robot_package.DistSensor import DistSensor
import numpy as np

def data_robot_creator(nom_fichier):
    """Fonction pour retourner les points du robot et les données des capteurs extrait de la configuration yaml

    Args:
        nom_fichier (string): nom du fichier de configuration yaml

    Returns:
        dict: dictionnaire contenant les configurations
    """
    dict_data = yaml_data_import(nom_fichier)


    dist_sensors = capteur_creator(dict_data.get('capteurs'))
    point_robot = robot_point_creator(dict_data.get('point_robot'))
    point_area_obstacle = area_obstacle_creator(dict_data.get('area_detection_obstacle'))

    return point_robot, dist_sensors, point_area_obstacle



def capteur_creator(dict_capteur):
    """Fonction permettant d'extraire les données des capteurs du dictionnaire générer par l'importation du yaml

    Args:
        dict_capteur (dict): diictionnaire de configuration des capteurs

    Returns:
        liste Distsensor: liste contenant les objets distsensor des capteurs configurés. L'angle est en RADIAN
    """
    # initialisation de la liste de capteurs
    dist_sensors = []
    
    # Ajout des capteurs en fonction du fichier capteur_config.yaml
    for capteur in dict_capteur:
        data = dict_capteur.get(capteur)
        dist_sensors.append(DistSensor(data['x'], data['y'], np.deg2rad(data['theta'])))
    
    return dist_sensors


def robot_point_creator(dict_point):
    """Fonction qui permet de convertir les données dictionnaires de configuration de la forme du robot en une liste de point

    Args:
        dict_point (dict): dictionnaire de configuration de la forme du robot

    Returns:
        liste: liste des coordonnées dans le repère du robot de sa forme
    """
    # initialisation de la liste de capteurs
    robot_pointX = []
    robot_pointY = []

    # Ajout des capteurs en fonction du fichier capteur_config.yaml
    for point in dict_point:
        data = dict_point.get(point)
        robot_pointX.append(data['x'])
        robot_pointY.append(data['y'])
    
    # Ajout de la ligne pour fermer le robot
    robot_pointX.append(robot_pointX[0])
    robot_pointY.append(robot_pointY[0])

    return [robot_pointX, robot_pointY]

def area_obstacle_creator(dict_area):
    """FOnction qui permet de convertir le dictionnaire de points représentant les aires de detection en liste python de points 

    Args:
        dict_area (dict): dictionnaire de configuration des zones de detections d'obstacle

    Returns:
        liste: liste des points des différentes aires
    """
    point_area = []

    for area in dict_area:
        area_point = []
        dict_point = dict_area.get(area)
        for point in dict_point:
            data = dict_point.get(point)
            area_point.append([data['x'], data['y']])
        point_area.append(area_point)
    
    return point_area



if __name__ == "__main__":     
    nom_fichier = './src/config/robot_config.yaml'
    point_robot, dist_sensor, point_area_obstacle = data_robot_creator(nom_fichier)

    print("\n------ Main -----")
    print(point_area_obstacle)

