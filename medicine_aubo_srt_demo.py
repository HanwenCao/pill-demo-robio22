from auboi5_controller import AuboController
import time
import math
from copy import deepcopy

if __name__ == '__main__':
    auboi5_controller = AuboController('192.168.1.115')

    angle = math.pi/6
    p1 = auboi5_controller.assign_pick_point(0.2387239779144876, -0.25429673419425003,angle,0.3106401367492817)
    auboi5_controller.moveL(p1)
    print("im here!!!!!!")

