# coding: utf-8
# @author S6ril & Starfunx

"""
CLasse pour les contenir les coordonnées. 
Inspiration des coordonnées de ROS. 
"""

class Pose2D():
    def __init__(self, x :float = 0, y :float = 0, theta :float = 0) -> None:
        self.x = x
        self.y = y
        self.theta = theta
    """
    Classe pour contenir les positions du robot sur un plan.
    """
