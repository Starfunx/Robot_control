# coding: utf-8
# @author S6ril & Starfunx

"""API pour controler facilement le robot

Returns:
    [type]: [description]
"""
from external.serial_driver import BetterSerial
from .component.Class_Pose2D import Pose2D
from .component.Class_Wheel import Wheel

#Joystik control pour tester facilement le robot
#Sharp GCODE

#faire main. 
#1. Attendre debut du match avec le bordel tirette
#2. actions du match une a une. Action = objectif == boucle -> asservissement en position.

class Robot(BetterSerial):
    """Api for using the robot.

    Args:
        BetterSerial inherite from serial class
    """
    def __init__(self, portserial, bauderate) -> None:
        BetterSerial.__init__(self, portserial, bauderate)
        self.robotPose = Pose2D()
        self.wheel_droite = Wheel()
        self.wheel_gauche = Wheel()


    def fast_move_to(self, x : float = 0, y : float = 0, theta : float = 0 ):
        """G00 Le robot se déplace au plus vite vers la cible et prend l’angle d’arrivée.

        Args:
            x (float): Position en X absolue sur le terrain
            y (float): Position en Y absolue sur le terrain
            theta (float): Angle d'arrivé à prendre lorsque la position est atteinte
        """
        self.write(b'G00 X')
        self.write(str.encode(str(x)))
        self.write(b' Y')
        self.write(str.encode(str(y)))
        self.write(b' A')
        self.write(str.encode(str(theta)))
        self.write(b'\n')

        self.cleanSerial()


    def slow_move_to(self, x: float = 0, y: float = 0, theta: float = 0):
        """G01 Le robot se déplace vers la cible de façon linéaire: il s’oriente d’abord vers sa cible
        avant d’avancer jusqu’à atteindre la cible. Il prend l’angle d’arrivée une fois sur la cible.

        Args:
            x (float): Position en X absolue sur le terrain
            y (float): Position en Y absolue sur le terrain
            theta (float): Angle d'arrivé à prendre lorsque la position est atteinte
        """
        self.write(b'G01 X')
        self.write(str.encode(str(x)))
        self.write(b' Y')
        self.write(str.encode(str(y)))
        self.write(b' A')
        self.write(str.encode(str(theta)))
        self.write(b'\n')

        self.cleanSerial()

    def set_speed(self, linearSpeed: float = 0, angularSpeed: float = 0 ):
        """G10 Le robot se déplace à la vitesse et vitesse angulaire précisée.

        Args:
            linearSpeed (float): Consigne en vitesse linéaire du robot
            angularSpeed (float): Consigne en vitesse angulaire du robot
        """
        self.write(b'G10 I')
        self.write(str.encode(str(linearSpeed)))
        self.write(b' J')
        self.write(str.encode(str(angularSpeed)))
        self.write(b'\n')

        self.cleanSerial()

    def set_wheel_speed(self, leftWheelSpeed: float, RightWeelSpeed: float):
        """G11 Les roues du robot se déplacent aux vitesses précisées.

        Args:
            leftWheelSpeed (float): Consigne de vitesse de la roue gauche
            RightWeelSpeed (float): Consigne de vitesse de la roue droite
        """
        self.write(b'G11 I')
        self.write(str.encode(str(leftWheelSpeed)))
        self.write(b' J')
        self.write(str.encode(str(RightWeelSpeed)))
        self.write(b'\n')

        self.cleanSerial()

    # position control

    def get_pose(self):
        """Fonction qui met à jour la position actuelle du robot dans la classe.

        Returns:
            Pose2D: Position actuelle du robot.
        """
        if (self.is_open):
            self.write(b'M114\n')  # Demande la position au robot

            message = self.decode_serial()
            # print(message)
            # Répartition des valeurs dans la variable position.
            if (len(message) >= 6):
                self.robotPose.x = float(message[1])
                self.robotPose.y = float(message[3])
                self.robotPose.theta = float(message[5])
            

        return self.robotPose

    def set_pose(self, x: float = 0, y: float = 0, theta: float = 0):
        """G92 Met à jour la position du robot, et change place la consigne de position du robot
        à la même valeur.

        Args:
            x (float): Nouvelle valeur de position suivant l'axe x
            y (float): Nouvelle valeur de postion suivant l'axe y
            theta (float): Nouvelle valeur de position autour de l'axe z
        """
        self.write(b'G92 X')
        self.write(str.encode(str(x)))
        self.write(b' Y')
        self.write(str.encode(str(y)))
        self.write(b' A')
        self.write(str.encode(str(theta)))
        self.write(b'\n')

        self.cleanSerial()

    # motor enable control

    def enable_motors(self):
        """
        M19 Active les moteurs et remet à zéro les intégrateurs des PID de contrôle moteur
        """
        if (self.is_open):
            self.write(b'M19\n')  # Mise en route des moteurs

            self.cleanSerial()

    def disable_motors(self):
        """
        M18 Désactive les déplacements moteurs stoppant les mouvements jusqu’à réactivation 
        via la commande M19
        """
        if (self.is_open):
            self.write(b'M18\n')  # Stop tous les moteurs

            self.cleanSerial()

    # tests commands

    def get_left_wheel_data(self):
        """M115 permet d'obtenir les information de la roue gauche, en consigne, mesure, commande 

        Returns:
            Wheel : Classe contenant la consigne, mesure et commande de la roue gauche du robot.
        """
        # to do ( besoin pour calibrer les pid avec zigler nichhols etc..)
        if (self.is_open):
            self.write(b'M115\n')  # Demande la position au robot

            message = self.decode_serial()
            # print(message)
            # Répartition des valeurs dans la variable position.
            if (len(message) >= 6):
                self.wheel_gauche.consigne = float(message[1])
                self.wheel_gauche.measure = float(message[3][1:])
                self.wheel_gauche.commande = float(message[5][1:])

        return self.wheel_gauche

    def get_right_wheel_data(self):
        """M116 permet d'obtenir les information de la roue droite, en consigne, mesure, commande

        Returns:
            Wheel : Classe contenant la consigne, mesure et commande de la roue droite du robot.
        """
        # to do ( besoin pour calibrer les pid avec zigler nichhols etc..)
        if (self.is_open):
            self.write(b'M116\n')  # Demande la position au robot

            message = self.decode_serial()
            # print(message)
            # Répartition des valeurs dans la variable position.
            if (len(message) >= 6):
                self.wheel_droite.consigne = float(message[1])
                self.wheel_droite.measure = float(message[3][1:])
                self.wheel_droite.commande = float(message[5][1:])

        return self.wheel_droite

    #reglages paramètres
    def set_pid_left(self, p, i, d):  # set pid vitesse gauche
        """M301 permet de configurer le PID sur la roue gauche

        Args:
            p (float): Valeur du coefficient proportionnel
            i (float): Valeur du coefficient intégral
            d (float): Valueur du coefficient dérivé
        """
        self.write(b'M301 P')
        self.write(str.encode(str(p)))
        self.write(b' I')
        self.write(str.encode(str(i)))
        self.write(b' D')
        self.write(str.encode(str(d)))
        self.write(b'\n')

        self.cleanSerial()

    def set_pid_right(self, p, i, d):  # set pid vitesse droite
        """M302 permet de configurer le PID sur la roue droite

        Args:
            p (float): Valeur du coefficient proportionnel
            i (float): Valeur du coefficient intégral
            d (float): Valueur du coefficient dérivé
        """
        self.write(b'M302 P')
        self.write(str.encode(str(p)))
        self.write(b' I')
        self.write(str.encode(str(i)))
        self.write(b' D')
        self.write(str.encode(str(d)))
        self.write(b'\n')

        self.cleanSerial()

    def set_left_odom_wheel_diameter(self, r):
        """M311 Permet de mettre à jour le diamètre de la roue codeuse gauche

        Args:
            r (float): Nouveau diamètre de la roue gauche encodeuse
        """
        #TODO Mettre le D
        self.write(b'M311 ')
        self.write(str.encode(str(r)))
        self.write(b'\n')

        self.cleanSerial()

    def set_right_odom_wheel_diameter(self, r):
        """M312 Permet de mettre à jour le diamètre de la roue codeuse droite

        Args:
            r (float): Nouveau diamètre de la roue droite encodeuse
        """
        self.write(b'M312 ')
        self.write(str.encode(str(r)))
        self.write(b'\n')

        self.cleanSerial()

    def set_odom_wheel_dist(self, d):
        """M313 Mettre à jour la distance entre les deux roues codeuses

        Args:
            d (flaot): distance entre les 2 roues
        """
        self.write(b'M313 ')
        self.write(str.encode(str(d)))
        self.write(b'\n')

        self.cleanSerial()


    def set_wheel_dist(self, d):
        """M314 Mettre à jour la distance entre les roues motrices

        Args:
            d (float): distance entre les roues motrices
        """
        self.write(b'M314 ')
        self.write(str.encode(str(d)))
        self.write(b'\n')

        self.cleanSerial()


    def set_linear_precision(self, value):
        """M321 Distance à partir de laquelle le robot s'oriente vers l'angle final

        Args:
            value (float): distance orientation angle final
        """
        self.write(b'M321 ')
        self.write(str.encode(str(value)))
        self.write(b'\n')

        self.cleanSerial()

    def set_angular_precision(self, value):
        """M322 Met à jour l'angle à partir duquel le robot arêtre de s'orienter pour se mettre en position

        Args:
            value (float): angle stop orientation
        """
        self.write(b'M322 ')
        self.write(str.encode(str(value)))
        self.write(b'\n')

        self.cleanSerial()

    def set_linear_speed_coefficient(self, value):
        """M323 Met à jour le coefficient de vitesse lineaire.
        Plus celui ci est élevé plus les vitesses linéaires seront élevée

        Args:
            value (float): Coefficient de vitesse lineaire
        """
        self.write(b'M323 ')
        self.write(str.encode(str(value)))
        self.write(b'\n')

        self.cleanSerial()

    def set_angular_speed_coefficient(self, value):
        """M324 Met à jour le coefficient de vitesse angulaire.
        Plus celui ci est élevé plus les vitesses angulaire seront élevée

        Args:
            value (float): coefficient vitesse angulaire
        """
        self.write(b'M324 ')
        self.write(str.encode(str(value)))
        self.write(b'\n')

        self.cleanSerial()

    def set_linear_speed_max(self, value):
        """M331 Met à jour la vitesse linéaire maximale que le robot est autorisé à prendre

        Args:
            value (float): vitesse linéaire max
        """
        self.write(b'M331 ')
        self.write(str.encode(str(value)))
        self.write(b'\n')

        self.cleanSerial()

    def set_angular_speed_max(self, value):
        """M332 Met à jour la vitesse angulaire maximae que le robot est autorisé à prendre

        Args:
            value (float): vitesse angulaire max
        """
        self.write(b'M332 ')
        self.write(str.encode(str(value)))
        self.write(b'\n')

        self.cleanSerial()

    def set_maximal_linear_acceleration(self, value):
        """M333 Met à jour l'accélération linéaire maximale

        Args:
            value (float): accélération linéaire maximale
        """
        self.write(b'M333 ')
        self.write(str.encode(str(value)))
        self.write(b'\n')

        self.cleanSerial()
    
    def set_maximal_angular_acceleration(self, value):
        """M334 Met à jour l'accélération angulaire maxiamle

        Args:
            value (float): accélération angulaire maximale
        """
        self.write(b'M334 ')
        self.write(str.encode(str(value)))
        self.write(b'\n')

        self.cleanSerial()

    def set_maximal_linear_deceleration(self, value):
        """M335 Met à jour l'accélération linéaire maximale lors des phases de décélération

        Args:
            value (flaot): décélération linéaire maximale
        """
        self.write(b'M335 ')
        self.write(str.encode(str(value)))
        self.write(b'\n')

        self.cleanSerial()

    def set_maximal_angular_deceleration(self, value):
        """M336 Met à jour l'accélération angulaire maximale lors des phases de décélération

        Args:
            value (float): décélération angulaire maximale
        """
        self.write(b'M336 ')
        self.write(str.encode(str(value)))
        self.write(b'\n')

        self.cleanSerial()

    # aliases

    def miseEnRouteRobot(self):
        self.enable_motors()

    def stopRobot(self):
        self.disable_motors()

