import numpy as np

class DistSensor(object):
    """
    classe des capteurs de distance
    """
    def __init__(self, x=0.0, y=0.0, theta=0.0) -> None:
        """focntion de construction de la classe des capteurs de distance

        Args:
            x (float): position en x en mm du capteur
            y (float): position en y en mm du capteur
            t (float): angle en radian du capteur par rapport à l'axe x du robot (devant)
        """
        self.position = np.array([x, y, theta])
        self.obstacle_pose = [None, None] # Position de l'obstacle dans le repère du capteur

    def set_sensor_pose(self, x, y, theta):
        """fonction pour définir l'emplacement du capteur
        Args:
            x (float): position en x en mm du capteur
            y (float): position en y en mm du capteur
            t (float): angle en radian du capteur par rapport à l'axe x du robot (devant)
        """       
        self.position = np.array([x, y, theta])

    def get_sensor_pose(self):
        """Fonction pour obtenir l'emplacement du capteur
        Returns:
            [tuple]: (x, y, t) pose of the sensor in the robot coordinates
        """        
        return self.position

    def set_dist(self, distance_x, distance_y):
        """Fonction de démo. Permet de donner une mesure au capteur
        Args:
            dist (float): distance en mm de l'obstacle par rapport au capteur
        """
        self.obstacle_pose = np.array([distance_x, distance_y])
        return self.obstacle_pose

    def get_transformation_matrix(self):
        """Fonction pour créer les matrices de projections du repère du capteur au repère du robot
        Returns:
            np.array (3,3): Matrice de projetion du repère du capteur au repère du robot
        """
        # Projection dans le repère du robot mais positionné sur le capteur
        # x (obstacle->capteur)/robot = - distance * sin(theta)
        # y (obstacle->capteur)/robot = distance * cos(theta)
        x = self.position[0]
        y = self.position[1]
        theta = self.position[2]
        m_robot_capteur_rotation = np.array([[-np.sin(theta),  -np.cos(theta), 0],
                                             [np.cos(theta), -np.sin(theta), 0],
                                             [0, 0, 1]])

        # Déplacement de ce rèpere au centre du robot
        # x (capteur ->centre)/robot
        # y (capteur ->centre)/robot
        M_robot_centre_translation = np.array([[1, 0, x],
                                               [0, 1, y],
                                               [0, 0, 1]])
        M_robot_centre = np.dot(M_robot_centre_translation,
                                m_robot_capteur_rotation)
        
        return M_robot_centre


    def get_obstacle_pose(self):
        """Fonction pour preojeter les coordonnées du repère du capteur dans le repère du robot.
        On utilise des matrices de taille 3x3 pour porjeter des coordonnées plan afin de faire la tranformation par produit matriciel.
        Pour plus d'information : https://www2.ift.ulaval.ca/~pgiguere/cours/IntroRobotique2017/notes/03-VisionIII.pdf
        Returns:
            [np.array]: position in mm of the obstacle in the robot space (cartesian coordinate)
        """
        if any(elem is None for elem in self.obstacle_pose):
            return [[], []]

        M_robot_centre = self.get_transformation_matrix()

        obstace_pose = self.obstacle_pose
        obstace_pose = np.append(obstace_pose, [1], axis=0 )

        obstace_pose = np.dot(M_robot_centre, obstace_pose)
        
        # Projection dans le repère du robot avec translation vers le centre du robot
        return np.array([[obstace_pose[0]], [obstace_pose[1]]])
