# coding: utf-8
# @author S6ril & Starfunx

"""
Cette classe permet de gérer la communication Gcode entre un robot et un ordinateur.
Pour trouver le port :

- Avec l'arduino débranchée
ls /dev/tty*

- Avec l'arduino branchée
ls /dev/tty*

- On recherche le nouveau port connecté ;)
"""
import serial


class BetterSerial(serial.Serial):
    """
    Classe communication sur le port serial.
    """

    def __init__(self, portserial, bauderate, timeout = 1):
        """Initialisation des variables internes de la classe.

        Args:
            portserial (char): Port USB de l'arduino
            bauderate (float): bauderate de la carte
        """
        super(BetterSerial, self).__init__()
        serial.Serial.__init__(self)
        self.port = portserial
        self.baudrate = bauderate
        self.timeout = timeout
        self.open()

    def __del__(self):
        """
        Destructeur de la classe.
        Permet de fermer le port Serial.
        """
        self.close()
        print("Serial close")

    def cleanSerial(self):
        """
        Nettoyage du buffer pour éviter une saturation.
        On appelle cette fonction après chaques commandes Gcode executées.
        """
        self.flushInput()
        self.flushOutput()
    
    def decode_serial(self, separator = " "):
        """
        Fonction pour lire, décoder, et mettre les données de la sérial dans une liste
        """
        message = self.readline()  # Lecture du port serial

        message = message.decode()  # Conversion bytes en str
        message = message.rstrip()  # Enlève \n
        message = message.split(separator)  # Conversion str en list

        return message
