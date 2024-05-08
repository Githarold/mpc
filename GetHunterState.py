import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from hunter_msgs.msg import HunterStatus
from math import sqrt, atan2, sin, tan, cos, log, floor, pi
from GetSensorSate import GPS, VehicleState
import GetPathParams
# from GetWaypoint import PathPlanner
from autocarz.msg import CtrlCmd, WaypointInfo  #고쳐야함
import numpy as np
path_model = "0.1m_path"

class HunterState:
    def __init__(self):
        # localization
        self.waypointInfo = WaypointInfo()
        # self.waypointInfoPub = rospy.Publisher('/waypointInfo', WaypointInfo, queue_size=1)
        # self.distance_log = rospy.Publisher('distance_error', Float32, queue_size=1)

        # path data
        self.pathmodel = GetPathParams.ModelConverter.getPathParams(path_model)
        self.globalPath = self.pathmodel.path
        # self.path_planner = PathPlanner(self.globalPath)

        # centroid
        self.centorid_back_ratio = 0.208
        self.centorid_front_ratio = 0.341
        self.gps_data = GPS()
        self.pre_gps_inBack = None
        self.pre_gps_inFront = None
        self.pre_Back_X , self.pre_Back_Y = None, None
        self.pre_Front_X , self.pre_Front_Y = None, None
        self.pre_phi = None

        # vehicle data
        self.vehiclestate = VehicleState()

        self.V_mat = np.zeros((1,2))
        self.phi_mat = np.zeros((2,2))

    def find_nearest_waypoint(self, path, curX, curY): #완료
        minDist = float("inf")
        nearest_idx = 0
        for i, pose in enumerate(path.poses):
            dx = curX - pose.pose.position.x
            dy = curY - pose.pose.position.y
            dist = sqrt(dx*dx + dy*dy)
            if dist < minDist:
                nearest_idx = i
                minDist = dist
        return nearest_idx, minDist

    def localization(self, odom):
        """
        Publish distance errors for debugging and monitoring.
        """
     
        curX = odom.pose.pose.position.x
        curY = odom.pose.pose.position.y
        
        # 최근접 점 탐색 및 거리 계산
        curWaypointIndx, minDist = self.find_nearest_waypoint(self.globalPath, curX, curY)
       
        return curWaypointIndx, minDist  

    def update_hunter_state(self, odomBack, odomFront, last_gps_inBack, last_gps_inFront):
        # odomBack, heading, gps_statusBack, gps_covarianceBack, checkGPSBack, checkHeading, last_gps_inBack = self.gps_data.gps_data()
        # odomFront, gps_statusFront, gps_covarianceFront, checkGPSFront, last_gps_inFront = self.gps_data.gps_dataFront()
        last_gps_inBack_to_nsec = last_gps_inBack.to_nsec()
        last_gps_inBack_to_nsec_int = float(last_gps_inBack_to_nsec)
        last_gps_inBack_to_sec_int = last_gps_inBack_to_nsec_int / 10**9

        last_gps_inFront_to_nsec = last_gps_inFront.to_nsec()
        last_gps_inFront_to_nsec_int = float(last_gps_inFront_to_nsec)
        last_gps_inFront_to_sec_int = last_gps_inFront_to_nsec_int / 10**9
        # print(last_gps_inFront_to_sec_int)

        Back_X = odomBack.pose.pose.position.x
        Back_Y = odomBack.pose.pose.position.y

        Front_X = odomFront.pose.pose.position.x
        Front_Y = odomFront.pose.pose.position.y

        # X, Y
        Centroid_x = (self.centorid_front_ratio * Back_X + self.centorid_back_ratio * Front_X) / (self.centorid_back_ratio + self.centorid_front_ratio)
        Centroid_y = (self.centorid_front_ratio * Back_Y + self.centorid_back_ratio * Front_Y) / (self.centorid_back_ratio + self.centorid_front_ratio)
        # print(Centroid_x, Centroid_y)

        heading_X = Front_X - Back_X
        heading_Y = Front_Y - Back_Y
        phi = atan2(heading_Y, heading_X)

        # # phi
        # phi = atan2(Centroid_y, Centroid_x)

        # self.Vx
        if self.pre_gps_inFront == None:
            self.Vx = 0.0
            self.Vy = 0.0
        elif self.pre_Front_X == 0.0:
            self.Vx = 0.0
            self.Vy = 0.0
        elif last_gps_inFront_to_sec_int - self.pre_gps_inFront == 0.0:
            # print("!!!!!!!!!!!")
            a=1
            # pass
        else:
            self.Vx = (Front_X - self.pre_Front_X)/ (last_gps_inFront_to_sec_int - self.pre_gps_inFront)
            self.Vy = (Front_Y - self.pre_Front_Y) / (last_gps_inFront_to_sec_int - self.pre_gps_inFront)
            # print(Front_X - self.pre_Front_X)
            # print(last_gps_inFront_to_sec_int - self.pre_gps_inFront)

        # print(self.Vx, self.Vy)
        if self.Vx == 0 and self.Vy == 0:
            # print("V = 0")
            a=1
        elif np.sqrt(self.Vx**2 + self.Vy**2) < 1:
            # print(self.Vx)
            theta = np.arccos(self.Vx/np.sqrt(self.Vx**2 + self.Vy**2))
            min_v = 1
            self.Vx = abs(min_v*np.cos(theta))
            self.Vy = min_v*np.sin(theta)
            
        #     print("min V!!!")
        
        # omega
        if self.pre_phi == None:
            self.yawrate = 0.0
        elif last_gps_inFront_to_sec_int - self.pre_gps_inFront == 0.0:
            pass
        else:
            self.yawrate = (phi - self.pre_phi) / (last_gps_inFront_to_sec_int - self.pre_gps_inFront)

        # self.vx, vy
        self.V_mat[0,0], self.V_mat[0,1] = self.Vx, self.Vy
        self.phi_mat[0,0], self.phi_mat[1,1] = np.cos(phi), np.cos(phi)
        self.phi_mat[0,1], self.phi_mat[1,0] = -np.sin(phi), np.sin(phi)
        car_frame_v = self.V_mat@np.linalg.inv(self.phi_mat)
        car_frame_v = car_frame_v.reshape(-1)

        # near point, minimum distance error
        curWaypointIndx, distance= self.localization(odomFront)

        self.pre_gps_inFront = last_gps_inFront_to_sec_int
        self.pre_gps_inBack = last_gps_inBack
        self.pre_Back_X, self.pre_Back_Y = Back_X, Back_Y
        self.pre_Front_X , self.pre_Front_Y = Front_X, Front_Y
        self.pre_phi = phi

        
        # print("Vx : ", abs(car_frame_v[0]))
        # return [Centroid_x, Centroid_y, phi, car_frame_v[0], car_frame_v[1], self.yawrate], curWaypointIndx, distance, checkGPSBack, checkGPSFront
        return [Centroid_x, Centroid_y, phi, abs(car_frame_v[0]), car_frame_v[1], self.yawrate], curWaypointIndx, distance
    
    