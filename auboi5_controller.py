import numpy as np
# from ikfastpy import ikfastpy
import robotcontrol
import time
from scipy.spatial.transform import Rotation as Rot
from multiprocessing import shared_memory
import threading
from srt_serial import PythonSerialDriver
from copy import deepcopy
import math
import serial


class AuboController (threading.Thread):
    def __init__(self, robot_ip_):
        self.robot = robotcontrol.Auboi5Robot()
        self.robot.initialize()
        handle = self.robot.create_context()
        port = 8899
        result = self.robot.connect(robot_ip_,port)
        if result != robotcontrol.RobotErrorType.RobotError_SUCC:
            print("connection failed!!")
        self.robot.robot_startup()
        self.robot.init_profile()
        self.speed_scale_ = 0.4
        joint_maxvelc = (self.speed_scale_*2.0, self.speed_scale_*2.0, self.speed_scale_*2.0, self.speed_scale_*2.0, self.speed_scale_*2.0, self.speed_scale_*2.0)
        joint_maxacc  = (self.speed_scale_*3.0, self.speed_scale_*3.0, self.speed_scale_*3.0, self.speed_scale_*3.0, self.speed_scale_*3.0, self.speed_scale_*3.0)
        line_maxvelc  = self.speed_scale_*1.0
        line_maxvelc  = self.speed_scale_*1.0

        self.robot.set_joint_maxvelc(joint_maxvelc)
        self.robot.set_joint_maxacc(joint_maxacc)
        self.robot.set_end_max_line_velc(line_maxvelc)
        self.robot.set_end_max_line_acc(line_maxvelc)

        self.srt = PythonSerialDriver()
        
        self.cTo_z_ = 0.326
        self.pick_z_ = 0.134 + 0.018
        self.sweep_z_ = 0.140 + 0.012

        self.medicine_box_position_origin_ = [-0.11568, -0.458656, 0.151801+0.005+0.01] # -0.11568, -0.457156, 0.151801

        # self.wms = np.full((7, 4, 6), False)
        p = 0.5
        self.wms = np.random.choice(a=[False, True], size=(7, 4, 6), p=[p, 1-p])  
        # days_per_week=7, times_per_day=4, pills_per_taking=6, grid_occupied=False

        # self.led = serial.Serial('/dev/ttyUSB1', 9600)


    def moveJ(self, joints):
        self.robot.move_joint(joints)

    def moveL(self, pose):
        xyz = (pose[0],pose[1],pose[2])
        ori = (pose[3],pose[4],pose[5],pose[6])
        self.robot.move_to_target_in_cartesian(xyz,ori)

    def getCurrentWaypoint(self,):
        current_status = self.robot.get_current_waypoint()
        # print(current_status)
        return current_status

    def pos_srt(self, pressure = 70):
        self.srt.move3Fingers(True, pressure)

    def neg_srt(self, pressure = 60):
        self.srt.move3Fingers(False, pressure)        

    def zero_srt(self, pressure = 0):
        self.srt.move3Fingers(True, pressure)

    def opening_width_mapping(self,opening):
        if opening >= 60:
            width = 4
        elif opening >= 53:
            width = 3
        elif opening >= 40:
            width = 2
        elif opening >= 10:
            width = 1
        else:
            width = 0
        return width

    def pick_one_time(self,point,width,zoffset=0.05):
        self.neg_srt(0)
        if width >= 4:
            pressure = 50
        elif width == 3:
            pressure = 40
        elif width == 2:
            pressure = 30
        elif width == 1:
            pressure = 20
        else:
            pressure = 0
        self.pos_srt(pressure)

        point_pre = deepcopy(point)
        point_pre[2] += zoffset
        self.moveL(point_pre)
        # self.pos_srt()
        print("im in point_pre")
        self.moveL(point)
        # self.pos_srt(0)
        # time.sleep(0.1)
        self.neg_srt(60)
        print("im in point")

        self.moveL(point_pre)
        print("im in point_pre")

    def place_one_time(self,point,zoffset=0.08,pressure=60):
        point_pre = deepcopy(point)
        point_pre[2] += zoffset
        self.moveL(point_pre)
        self.moveL(point)
        self.neg_srt(0)
        self.pos_srt(pressure)
        self.moveL(point_pre)
        self.neg_srt(0)

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

    def assign_pick_point(self,x,y,rad,z):
        #the degree from left to right is pi/2 to -pi/2.    0 point to the front.
        quat = self.get_quaternion_from_euler(math.pi,0,rad)
        # print(quat)
        return [x,y,z,0.0,quat[0],quat[1],0.0]
    
    def set_aside(self,start_p_xy,finish_p_xy):
        #neg at firstc
        self.neg_srt(60)
        #compute the degree
        k = (start_p_xy[1]-finish_p_xy[1])/(start_p_xy[0]-finish_p_xy[0])
        x_theta = math.atan(abs(k))
        if k>0:
            angle = -(math.pi/2 - x_theta)
        elif k<0:
            angle = (math.pi/2 - x_theta)

        start_p = self.assign_pick_point(start_p_xy[0],start_p_xy[1],angle,self.sweep_z_)
        finish_p = self.assign_pick_point(finish_p_xy[0],finish_p_xy[1],angle,self.sweep_z_)
        self.moveL(start_p)
        self.moveL(finish_p)
        self.pos_srt(60)
        self.moveJ(self.photo_joints_)
        self.neg_srt(0)

    def set_sweep(self,start_p_xy,finish_p_xy):
        #neg at first
        self.neg_srt(60)
        #compute the degree
        k = (start_p_xy[1]-finish_p_xy[1])/(start_p_xy[0]-finish_p_xy[0])
        x_theta = math.atan(abs(k))
        if k>0:
            angle = x_theta
        elif k<0:
            angle = -x_theta

        start_p = self.assign_pick_point(start_p_xy[0],start_p_xy[1],angle,self.sweep_z_)
        finish_p = self.assign_pick_point(finish_p_xy[0],finish_p_xy[1],angle,self.sweep_z_)
        self.moveL(start_p)
        self.moveL(finish_p)
        self.moveJ(self.photo_joints_)
        self.pos_srt(0)

    def quaternion_to_Tmatrix(self, quaternion):
        """Return homogeneous rotation matrix from quaternion.
        >>> R = quaternion_to_Tmatrix([x,y,z,ow,ox,oy,oz])

        """
        _EPS = np.finfo(float).eps * 4.0
        quat = [quaternion[4],quaternion[5],quaternion[6],quaternion[3]]
        q = np.array(quat, dtype=np.float64, copy=True)
        nq = np.dot(q, q)
        if nq < _EPS:
            return np.identity(4)
        q *= math.sqrt(2.0 / nq)
        q = np.outer(q, q)
        return np.array((
            (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], quaternion[0]),
            (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], quaternion[1]),
            (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], quaternion[2]),
            (                0.0,                 0.0,                 0.0, 1.0)
            ), dtype=np.float64)

    def eye_hand_transfer(self,cTo_xy):    
        # The following matrix should be modified
        # bTw = np.array((
        #     (  1.0,0.0,0.0,0.2387239779144876),
        #     (  0.0,-1.0,0.0,-0.25429673419425003),
        #     (  0.0,0.0,-1.0, 0.3106401367492817),
        #     (                0.0,                 0.0,                 0.0, 1.0)
        #     ), dtype=np.float64)
        
        current_status = self.getCurrentWaypoint()
 
        bTw = np.array((
            (  1.0,0.0,0.0,current_status['pos'][0]),
            (  0.0,-1.0,0.0,current_status['pos'][1]),
            (  0.0,0.0,-1.0, current_status['pos'][2]),
            (0.0,0.0,0.0, 1.0)
            ), dtype=np.float64)

        wTc = np.array((
            (   0.999732 , -0.0182786 , -0.0142087  ,-0.0153094),
            (  0.0182479 ,   0.999831 ,-0.00228665  ,  0.066011),
            (  0.0142481 , 0.00202676 ,   0.999896  , 0.0101006),
            (                0.0,                 0.0,                 0.0, 1.0)
            ), dtype=np.float64)            
        bTc = np.dot(bTw,wTc)

        cTo = np.array((
            (  1.0,0.0,0.0,cTo_xy[0]),
            (  0.0,1.0,0.0,cTo_xy[1]),
            (  0.0,0.0,1.0, self.cTo_z_),
            (                0.0,                 0.0,                 0.0, 1.0)
            ), dtype=np.float64)
        bTo = np.dot(bTc,cTo)
        # print('the transformed bTo is: ')
        # print(bTo)
        return [bTo[0,3],bTo[1,3]]

    def rad_transfer(self,rad):
        # print('rad is: ',rad)
        if rad >= 0 and rad <= math.pi:
            # rad_transfered = -(math.pi/2 - rad)
            rad_transfered = math.pi/2 - rad
        elif rad < 0 and rad >= -math.pi:
            # rad_transfered = math.pi/2 + rad
            rad_transfered = - (math.pi/2 + rad)
        # print('rad_transfered is: ',rad_transfered)
        return rad_transfered

    def medicine_box_position(self,row,column):
        # starts from row,column=0,0
        row_gap = 0.03
        column_gap = 0.029
        x = self.medicine_box_position_origin_[0] + (column) * column_gap
        y = self.medicine_box_position_origin_[1] + (row) * row_gap
        z = self.medicine_box_position_origin_[2]
        return [x, y, z, 0.0, 1.0, 0.0, 0.0]

    def decide_place_row_col(self, pill_class):
        # info of each pill is stroed in 3rd axis of self.wms
        wms = self.wms[:,:,pill_class-2]
        
        # Find the first unoccupied grid
        print(wms, '\n', [np.where(wms==False)[0][0],np.where(wms==False)[1][0]])
        search = np.where(wms==False)
        if search[0].size == 0 or search[1].size == 0:
            print("No available grid for placing this kind of pill.")
            return -1, -1  # could be something else
        else:
            row = search[0][0]
            col = search[1][0]
            self.wms[row, col, pill_class-2] = True  # update
            return row, col

    # def led_control(self, row, col):
    #     # Input: row, col of wms
    #     strip_id = col
    #     led_id = row

    #     num_pills = np.sum(self.wms, axis=2)[led_id, strip_id]  # 0,1,2,3
    #     print('num_pills of this cell:', num_pills)
        
    #     # Color Map 1:blue  2:yellow 3: green 0:red
    #     if num_pills >= 1:
    #         # num_pills = 5
    #         led_color = 1  # Blue
    #     else:
    #         led_color = 0  

    #     # Led Control
    #     input_str = str(strip_id) + str(led_id) + str(led_color)
    #     self.led.write(bytes(input_str, encoding = "utf8"))
    def find_capture_position(self, pill_id):
        photo_joints_1 = [1.4730256795883179, -0.34653106331825256, -2.125231981277466, -0.20790015161037445, -1.5707817077636719, -0.09777495265007019]
        photo_joints_2 = [1.7618705034255981, -0.23622873425483704, -2.050191640853882, -0.24317924678325653, -1.5707817077636719, 0.19106757640838623]
        photo_joints_3 = [2.004626750946045, -0.07717428356409073, -1.918959379196167, -0.2709865868091583, -1.5707778930664062, 0.43383970856666565]
        photo_joints_4 = [2.220240831375122, 0.1384366750717163, -1.6959328651428223, -0.2635734975337982, -1.5707743167877197, 0.6494536995887756]
        photo_joints_5 = [1.3257313966751099, -0.8535616993904114, -2.3232288360595703, 0.10113120079040527, -1.5707778930664062, -0.24510496854782104]
        photo_joints_6 = [1.8765143156051636, -0.6176575422286987, -2.2589383125305176, -0.07048115879297256, -1.5707778930664062, 0.3056274354457855]
        photo_joints_7 = [2.2107129096984863, -0.34688928723335266, -2.125455379486084, -0.20776444673538208, -1.5707706212997437, 0.6398691534996033]
        photo_joints_8 = [2.4327595233917236, -0.06297697871923447, -1.9058825969696045, -0.27210167050361633, -1.5707743167877197, 0.8619462847709656]
        photo_joints_9 = [2.560816764831543, -0.48621082305908203, -2.202678680419922, -0.14568306505680084, -1.5707889795303345, 0.989931046962738]
        photo_joints_10 = [2.707529306411743, -0.18590900301933289, -2.011690139770508, -0.25495728850364685, -1.5707889795303345, 1.1366411447525024]

        if pill_id == 1:
            return photo_joints_1
        elif pill_id == 2:
            return photo_joints_2
        elif pill_id == 3:
            return photo_joints_3
        elif pill_id == 4:
            return photo_joints_4
        elif pill_id == 5:
            return photo_joints_5
        elif pill_id == 6:
            return photo_joints_6
        elif pill_id == 7:
            return photo_joints_7
        elif pill_id == 8:
            return photo_joints_8
        elif pill_id == 9:
            return photo_joints_9
        elif pill_id == 10:
            return photo_joints_10
        else:
            print('Pill ID error!')
            return photo_joints_1

if __name__ == '__main__':
    auboi5_controller = AuboController('192.168.1.115')
