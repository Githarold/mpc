import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from hunter_msgs.msg import HunterStatus
from math import sqrt, atan2, sin, tan, cos, log, floor, pi


class GPS:
    """
    Handles GPS data.
    
    """
    

    def __init__(self):
        # rospy.init_node("gps_connector", anonymous=False)
        # Subscribers for GPS data
        rospy.Subscriber("/ublox1/fix", NavSatFix, self.getGPSBack)
        rospy.Subscriber("/ublox2/fix", NavSatFix, self.getGPSFront)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        # GPS data variables
        self.odom = Odometry()
        self.odomFront = Odometry()
        self.heading = 0.0
        self.gps_status = 0
        self.gps_statusFront = 0
        self.gps_covariance = 0.0
        self.gps_covarianceFront = 0.0
        self.pastPos = [0.0, 0.0]
        self.checkGPS = False
        self.checkGPSFront = False
        self.checkHeading = False
        self.last_gps_in = rospy.Time.now()
        self.last_gps_inFront = rospy.Time.now()

        # # for debug
        # self.i = 0
        # self.j = 0

    def getGPSBack(self, data):
        gpsX, gpsY = self.latlong2xy(data.latitude, data.longitude)
        

        self.odom.header.frame_id = "map"
        self.odom.pose.pose.position.x = gpsX
        self.odom.pose.pose.position.y = gpsY
        self.odom.pose.pose.position.z = 0.0 
        self.last_gps_in = rospy.Time.now()
        self.gps_status = data.status.status
        self.gps_covariance = data.position_covariance[0]
        self.odom_pub.publish(self.odom)
        if self.checkGPS:
            if self.checkGPSFront and self.last_gps_in - self.last_gps_inFront <= rospy.Duration(0.11) and \
            (self.gps_statusFront == 2 or self.gps_covarianceFront < 0.005):
                

                dx = self.odomFront.pose.pose.position.x - gpsX
                dy = self.odomFront.pose.pose.position.y - gpsY
                self.heading = atan2(dy,dx)
                self.odom.pose.pose.orientation.x = 0.0
                self.odom.pose.pose.orientation.y = 0.0
                self.odom.pose.pose.orientation.z = sin(self.heading*0.5)
                self.odom.pose.pose.orientation.w = cos(self.heading*0.5)
                self.checkGPS = False
                self.checkHeading = True                

            # elif -4800 < velocity and velocity < 4800:
            else:
                dx = gpsX - self.pastPos[0]
                dy = gpsY - self.pastPos[1]
                dist = sqrt(dx**2+dy**2)

                # print(dist)
                
                if dist > 0.1:                      
                    self.heading=atan2(dy, dx)
                    self.odom.pose.pose.orientation.x = 0.0
                    self.odom.pose.pose.orientation.y = 0.0
                    self.odom.pose.pose.orientation.z = sin(self.heading*0.5)
                    self.odom.pose.pose.orientation.w = cos(self.heading*0.5)
                    self.checkGPS = False
                    self.checkHeading = True

        if not self.checkGPS:
            self.pastPos = [gpsX, gpsY]
            self.checkGPS = True
            
        if not self.checkHeading:
            print("No heading Back")

        # print(self.odom)

    def getGPSFront(self, data):
        gpsX, gpsY = self.latlong2xy(data.latitude, data.longitude)
        self.odomFront.header.frame_id="map"
        self.odomFront.pose.pose.position.x=gpsX
        self.odomFront.pose.pose.position.y=gpsY
        self.odomFront.pose.pose.position.z=0.0 
        self.last_gps_inFront = rospy.Time.now()
        self.gps_statusFront = data.status.status
        self.gps_covarianceFront = data.position_covariance[0]
        self.checkGPSFront = True
        self.temp  = True      


        if not self.temp:
            print("No heading Frant")

    def latlong2xy(self, lat, long):
        RE = 6371.00877     # 지구 반경(km)
        GRID = 0.0000005    # 격자 간격(km), 0.5mm단위 
        SLAT1 = 30.0
        SLAT2 = 60.0
        OLON = 126.0
        OLAT = 38.0
        XO = 43
        YO = 136

        DEGRAD = pi / 180.0
        re = RE / GRID
        slat1 = SLAT1 * DEGRAD
        slat2 = SLAT2 * DEGRAD
        olon = OLON * DEGRAD
        olat = OLAT * DEGRAD

        sn = tan(pi * 0.25 + slat2 * 0.5) / tan(pi * 0.25 + slat1 * 0.5)
        sn = log(cos(slat1) / cos(slat2)) / log(sn)
        sf = tan(pi * 0.25 + slat1 * 0.5)
        sf = pow(sf, sn) * cos(slat1) / sn
        ro = tan(pi * 0.25 + olat * 0.5)
        ro = re * sf / pow(ro, sn)

        ra = tan(pi * 0.25 + (lat) * DEGRAD * 0.5)
        ra = re * sf / pow(ra, sn)

        theta = long * DEGRAD - olon
        if theta > pi:
            theta -= 2.0 * pi
        if theta < -pi:
            theta += 2.0 * pi
        theta *= sn
        rs_x = floor(ra * sin(theta) + XO + 0.5)
        rs_y = floor(ro - ra * cos(theta) + YO + 0.5)
        first_xx = -93464 
        first_yy = 130046 
        rs_x = rs_x/2000 + first_xx
        rs_y = rs_y/2000 + first_yy

        return rs_x, rs_y

    def gps_data(self):
        return self.odom, self.heading, self.gps_status, self.gps_covariance, self.checkGPS, self.checkHeading, self.last_gps_in

    def gps_dataFront(self):
        return self.odomFront, self.gps_statusFront, self.gps_covarianceFront, self.checkGPSFront, self.last_gps_inFront
    
class VehicleState:
    """
    Handles vehicle state data like velocity and steering.
    """
    def __init__(self):
        # Subscribers for vehicle state data
        rospy.Subscriber("/hunter_status", HunterStatus, self.get_vehicle_state)

        # Vehicle state variables
        self.velocity = 0.0
        self.steering = 0.0
        self.checkState = False

    def get_vehicle_state(self, data): 

        velocity = data.linear_velocity    # data.velocity 0~20/3.6 m/s  --> self.velocity 0~20 km/h, self.target_velocity 0~20 km/h
        # print(f"velocity {velocity}")
      
        if velocity > 4800 or velocity < -4800:
            self.velocity = 0.0
        else:
            self.velocity = velocity
        
        self.steering = data.steering_angle
        
        self.checkState = True
        
    def vehicle_data(self):
        return self.velocity, self.steering, self.checkState

   
    
if __name__ == '__main__':
    rospy.init_node('gps_vehicle_node')
    gps = GPS()
    vehicle = VehicleState()
    rospy.spin()
