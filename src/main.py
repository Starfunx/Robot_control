from module.tirette import Selection_zone
from module.servo import servo_init, servo_move_max, servo_move_min
from external.API_interface.TLF_API.component.Class_Pose2D import Pose2D

from robot_package.RobotControl import RobotControl

nom_fichier = './config/robot_config.yaml'
robot_port = "/dev/ttyACM0"
robot_baurate = 115200

carte_obstacle_port = ""
carte_obstacle_bauderate = 115200

robotcontrol = RobotControl(nom_fichier, robot_port, robot_baurate, carte_obstacle_port, carte_obstacle_bauderate)

# robotcontrol.robot.set_pose(0, 0, 0)
pose = [Pose2D(1000, 0, 0),
        Pose2D(0, 0, 0)]

robotcontrol.set_goal(pose[::-1])

selection_zone = Selection_zone(14, 15)
SERVO = servo_init()
 # selection_zone.wait_start_loop()

print("Zone de départ :", selection_zone.zone)

robotcontrol.robot.enable_motors()
try:
    while not(robotcontrol.goal_reached()):
        # Mettre 10 Hz
        if (robotcontrol.distance_robot2goal(Pose2D(1000, 0, 0)) < 500):
            servo_move_max(SERVO)
        else:
            servo_move_min(SERVO)
        robotcontrol.update_speed()

except KeyboardInterrupt:
    print("stop")
    pass

# np.save("vitesse_angular", robotcontrol.consigne_vitesse_angulaire)
# np.save("vitesse_linear", robotcontrol.consigne_vitesse_lineaire)
# np.save("left_wheel", robotcontrol.mesure_vitesse_roue_gauche)
# np.save("right_wheel", robotcontrol.mesure_vitesse_roue_droite)

robotcontrol.robot.set_speed(0, 0)
robotcontrol.robot.disable_motors()
