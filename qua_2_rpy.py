import libpyauboi5
import time
from copy import deepcopy
import math
 
def euler_from_quaternion(quaternion):
        x= quaternion[0]
        y= quaternion[1]
        z= quaternion[2]
        w= quaternion[3]

        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return [roll_x, pitch_y, yaw_z] # in radians

if __name__ == '__main__':
    rpy = euler_from_quaternion([0.12200129680275658, -0.9924559092858365, 0.01183003353330129, -0.0026461308868564923])
    print(rpy)


