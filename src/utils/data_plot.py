from matplotlib.collections import PolyCollection
if __name__ == "__main__":
    # Pour executer le code en local et avoir les bon import
    import sys
    import os
    sys.path.append(os.getcwd() + "\\src")
    from robot_package.data_robot_creator import data_robot_creator

from matplotlib.patches import Polygon
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.axisartist.angle_helper as angle_helper
from matplotlib.projections import PolarAxes
from matplotlib.transforms import Affine2D
from mpl_toolkits.axisartist import HostAxes, GridHelperCurveLinear


def curvelinear_plot(rayon_max = 1000):
    """Fonction pour créer un repère cartésien avec en décoration les coordonnées polaire pour mieux repérer les angles
    Cette fonction est basée sur un exemple matplotlib : see demo_curvelinear_grid.py for details

    Args:
        rayon_max (float, optional): Rayon amximal du plot dans lequel afficher le robot. Defaults to 1000.

    Returns:
        matplotlib.fig, matplotlib.ax : figure et axe matplotlib dans lequel on fait l'affichage
    """
    tr_rotate = Affine2D().translate(90, 0)
    tr_scale = Affine2D().scale(np.pi/180., 1.)
    tr = tr_rotate + tr_scale + PolarAxes.PolarTransform()
    
    extreme_finder = angle_helper.ExtremeFinderCycle(20,
                                                     20,
                                                     lon_cycle=360,
                                                     lat_cycle=None,
                                                     lon_minmax=None,
                                                     lat_minmax=(-np.inf,
                                                                 np.inf),
                                                     )

    grid_locator1 = angle_helper.LocatorDMS(8)
    # tick_formatter1 = angle_helper.FormatterDMS()

    grid_helper = GridHelperCurveLinear(tr,
                                        extreme_finder=extreme_finder,
                                        grid_locator1=grid_locator1,
                                        # tick_formatter1=tick_formatter1
                                        )

    fig = plt.figure()
    ax1 = fig.add_subplot(axes_class=HostAxes, grid_helper=grid_helper)


    # Now creates floating axis
    # ax1.scatter(5, 5)

    # floating axis whose first coordinate (theta) is fixed at 60
    ax1.axis["ax"] = axis = ax1.new_floating_axis(0, 0)
    axis.set_axis_direction("top")
    axis.major_ticklabels.set_axis_direction("left")
    ax1.axis["ax1"] = axis = ax1.new_floating_axis(0, -90)
    axis.set_axis_direction("left")
    axis.major_ticklabels.set_axis_direction("top")
    # axis.label.set_text(r"$\theta = 60^{\circ}$")
    # axis.label.set_visible(True)

    # floating axis whose second coordinate (r) is fixed at 6
    ax1.axis["lon"] = axis = ax1.new_floating_axis(1, 150)
    axis.label.set_pad(10)
    
    # axis.label.set_text(r"$r = 1$")

    ax1.set_aspect(1.)
    ax1.set_xlim(-rayon_max, rayon_max)
    ax1.set_ylim(-rayon_max, rayon_max)

    ax1.grid(True)
    return fig, ax1




if __name__ == "__main__":
    import random as random
    fig, ax = curvelinear_plot(500)
    nom_fichier = './src/config/robot_config.yaml'
    point_robot, dist_sensors, area_obstacle = data_robot_creator(nom_fichier)

    
    ax.plot(point_robot[0], point_robot[1])
    
    for sensor in dist_sensors:
        #Affichage capteur
        sensor_pose = sensor.get_sensor_pose()
        ax.plot([sensor_pose[0]], sensor_pose[1], 'ko')
        ax.plot([sensor_pose[0], sensor_pose[0] + 50*np.sin(-sensor_pose[2])], [sensor_pose[1], sensor_pose[1] + 50*np.cos(-sensor_pose[2])], 'k')
    # ax.plot(dist_sensors[0], dist_sensors[1], 'o')

    color = ["b", "r", "g", "m", "y"]
    for area, c in zip(area_obstacle, color):
        poly = plt.Polygon(area, facecolor=c,
                           edgecolor=c, alpha=0.25, zorder=2)
        ax.add_patch(poly)
    


    plt.show()
