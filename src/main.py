from external.API_interface_python.TLF_API.Class_Robot import Robot

from robot_package.data_robot_creator import data_robot_creator



import numpy as np

class RobotControl():
    def __init__(self, nom_fichier):
        # creation des capteurs virtuels avec le yaml --> permet de convertir distacne en position
        
        _, self.dist_sensor = data_robot_creator(nom_fichier) # On ne récupère pas les points du robot

        pass


    def update_speed():
        # lecture
        # traitement
        # ecriture
        #######################
        # Recupe données
        #######################
        # update vitesse vitesse angulaire en fonction des obstacles
        # Recupére position --> driver robot
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

    def set_goal(position):
        pass
    
    def goal_reached():
        # critère dans un yaml
        pass
    





if __name__ == "__main__":
    nom_fichier = './src/config/robot_config.yaml'
    robotcontrol = RobotControl(nom_fichier)
