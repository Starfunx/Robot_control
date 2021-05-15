import time
from TLF_API.Class_Robot import Robot

robot = Robot("/dev/ttyACM0", 112500)
distance = 500

robot.set_pose(0, 0, 0)

print("-------------------------")
A = robot.get_pose()
P = [A.x, A.y, A.theta]
print(P, time.strftime("%H:%M:%S", time.gmtime()))
time.sleep(1)

print("-------------------------")
robot.enable_motors()
robot.slow_move_to(distance, 0, 0)
print(time.strftime("%H:%M:%S", time.gmtime()))

try:
    while P[0] < distance - 2:
        print("-------------------------")
        A = robot.get_pose()
        P = [A.x, A.y, A.theta]
        print(P, time.strftime("%H:%M:%S", time.gmtime()))
        time.sleep(1)
except KeyboardInterrupt:
    # break the while loop
    pass

robot.disable_motors()
