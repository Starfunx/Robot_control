if __name__ == "__main__":
    # Pour executer le code en local et avoir les bon import
    import sys
    import os
    sys.path.append(os.getcwd() + "\\src")

import sys
import os
sys.path.append(os.getcwd() + "\\")

from external.serial_driver import BetterSerial
import numpy as np

class CarteDetecteurObstacle(BetterSerial):
    """Api for using the robot.

    Args:
        Communication_Gcode inherite from serial class
    """

    def __init__(self, portserial, bauderate) -> None:
        BetterSerial.__init__(self, portserial, bauderate)
        self.get_distance("A") # Lecture une première fois de la distance pour l'initialisation


    def get_distance(self, capteur):
        """Fonction qui met à jour la position actuelle du robot dans la classe.
        """
        if (self.is_open):
            self.write(b'S')
            self.write(str(capteur).encode())
            self.write(b'\n')

            message = self.decode_serial(separator="; ")
            # print(message)

            if message[0] != '':
                if capteur == "A":
                    return np.array(message, dtype=float)
                else:
                    return float(message[0])

            return None


if __name__ == "__main__":
    import time

    carte = CarteDetecteurObstacle("COM8", 9600)

    while True:
        # print(carte.get_distance(0))
        A = carte.get_distance("A")
        print(A)
        # time.sleep(0.2)
