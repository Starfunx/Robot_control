import numpy as np


class Robot():
    """
    Classe robot
    """
    def __init__(self, x, y, t) -> None:
        """Initialisateur de la classe avec les coordonnées du robot

        Args:
            x (float): coordonnée X du robot
            y (float): coordonnée Y du robot
            t (float): angle theta du robot
        """
        self.position = np.array([x, y, t])
        self.shape = [] # Création d'une forme vide pour le robot. A définir avec une méthode

    def update_position(self, x, y, t):
        self.position = np.array([x, y, t])

    def get_transformation_matrix(self):
        """Création des matrices de transformation pour passer d'un capteur au repère du robot
        On utilise des matrices 3,3 pour les cooronnée (x, y) afin de faire la transformation avec un produit matriciel.
        Plus d'info : https://www2.ift.ulaval.ca/~pgiguere/cours/IntroRobotique2017/notes/03-VisionIII.pdf
        Returns:
            np.array (3,3): Matrice de transformation rotation et translation pour passer d'une coordonnée (x,y) du robot à une coordonnée (x,y) dans le repère de la table.
        """
        x = self.position[0]
        y = self.position[1]
        t = self.position[2]

        M_robot_table_rotation = np.array([[np.sin(t), np.cos(t), 0],
                                           [-np.cos(t), np.sin(t), 0],
                                           [0, 0, 1]])

        M_robot_table_translation = np.array([[1, 0, x],
                                              [0, 1, y],
                                              [0, 0, 1]])

        M_robot_table = np.dot(M_robot_table_translation,
                               M_robot_table_rotation)

        return M_robot_table

    def get_robot_point(self):
        """Fonction pour retourner facilement les coordonnées de la forme du robot dans le repère de la table
        Returns:
            np.array (2,n): coordonnées (x, y) de la forme du robot dans le repère de la table
        """
        robot = self.get_robot_shape()
        # return la transformation des point du robot dans le repère de la table
        return self.transform_robot2table(robot)
    
    def define_robot_shape(self, points_of_shape):
        """Fonction pour définir la forme du robot
        Args:
            points_of_shape (np.array (n, 2)): coordonnée des points de la forme du robot
        """
        self.shape = points_of_shape

    def get_robot_shape(self):
        return self.shape

    def transform_robot2table(self, coordonnee_point):
        """Fonction pour transformer les points du repère du robot dans le repère de la table

        Args:
            coordonnee_point (np.array): point du robot à mettre dans le repère de la table

        Returns:
            np.array: point dans le repère de la table
        """
        # Gestion des coordonnées vide. Apparaissent lors de l'initialisation.
        if coordonnee_point == [[], []]:
            return np.array([[], []])
        
        # Transformation du vecteur (2,n) en vecteur (3,n) pour faire le changement de repère par produit matriciel
        coordonnee_point = np.append(coordonnee_point,  [[1]*coordonnee_point.shape[1]], axis=0)
        # récupération de la matrice de transformation
        M_robot_table = self.get_transformation_matrix()
        # Changement de repère
        result = np.dot(M_robot_table, coordonnee_point)
        # On retorune seulement un vecteur (2, n). La 3e coordonnée est utile seulement pour les calculs
        return np.array([result[0], result[1]])
