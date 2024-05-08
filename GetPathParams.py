from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathModelParams:
    def __init__(self):
        self.path_name = None
        self.Model = None
        self.length_index = None # length between index
        self.num_index = None # number of index
        self.path = None
        self.pose = None
        self.list_x = None
        self.list_y = None
        self.list_phi = None
        self.list_vx = None
        self.list_vy = None
        self.list_yawrate = None

class ModelConverter:
    @staticmethod
    def getPathParams(Model):

        path_params = PathModelParams()

        if Model == '0.1m_path':

            path_params.path_name = "/home/scout/catkin_ws/src/hunter_mpc/path/"
            path_params.Model = 'waypoint_final_final_phi.txt'
            path_params.path = Path()       # global path
            path_params.path.header.frame_id = "/map"

            list_phi = []

            with open(path_params.path_name +path_params.Model, "r") as file:
                lines = file.readlines()
                for line in lines:
                    # path_params로 꼭 넣어야하나?
                    data = line.split()
                    path_params.pose = PoseStamped()
                    path_params.pose.pose.position.x = float(data[0])
                    path_params.pose.pose.position.y = float(data[1])
                    path_params.pose.pose.position.z = 0.0
                    # pose.pose.orientation.w = float(data[2])
                    path_params.path.poses.append(path_params.pose)
                    list_phi.append(data[2])

            list_x = [] # x list 
            list_y = [] # y list
            list_vx = []
            list_vy = []
            list_yawrate = []
            for i in range(len(path_params.path.poses)):
                list_x.append(path_params.path.poses[i].pose.position.x)
                list_y.append(path_params.path.poses[i].pose.position.y)
                list_vx.append(4)
                list_vy.append(0)
                list_yawrate.append(0)
            list_size = len(list_x)
            # print(len(list_x), len(list_y), len(list_phi), len(list_vx), len(list_vy), len(list_yawrate))

            

            path_params.list_x = list_x     # x좌표 리스트
            path_params.list_y = list_y     # y좌표 리스트
            path_params.length_index = 0.2      # 점 사이 거리
            path_params.num_index = list_size   # path 개수
            path_params.list_vx = list_vx
            path_params.list_vy = list_vy
            path_params.list_phi = list_phi
            path_params.list_yawrate = list_yawrate
        else:
            raise ValueError('Model invalid')
        
        return path_params


# ########## Delete############
# if __name__ == "__main__":
#     Model = '0.1m_path'
#     params = ModelConverter.getPathParams(Model)
#     print(params.path)