import math
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from slam_toolbox.srv import SaveMap
class Map_tracking(Node):
    def __init__(self):
        super().__init__('map_tracking')
        self.sub_scan = self.create_subscription(LaserScan,'/scan',self.callback_scan_val,10) # /scan
        #self.sub_scan = self.create_subscription(OccupancyGrid,'/map',self.callback_map_val,10) # /map
        self.sub_odom = self.create_subscription(Odometry,'/odom',self.callbach_odom_val,qos_profile=qos_profile_sensor_data)
        
        self.pub_timer = self.create_timer(0.5,self.callback_timer)
        self.pub_vel = self.create_publisher(Twist,'/cmd_vel',10)
        self.cli_map_saver = self.create_client(SaveMap,'/slam_toolbox/save_map')


        self.cmd_vel=Twist()
        self.start_point = True # odom 저장을 위한
        self.start = True # 센서값 받아오는것을 위함
        self.front_avg = 0.0
        self.right_avg = 0.0
        self.left_avg = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.tolerance_x = 0.3
        self.tolerance_y = 0.3
        self.cnt = 0 # x,y 좌표 비교 전 일정 텀을 두기 위해 사용
    
    def callback_scan_val(self,data):
        
        val=data.ranges
        front = val[181:360] 
        left = val[361:540] 
        right = val[0:180] 
        right2 = val[1000:1079]
        
        ## inf 값을 제거하기 위한 과정
        front = list(filter(lambda x: x != math.inf and x != -math.inf, front))
        right = list(filter(lambda x: x != math.inf and x != -math.inf, right))
        right2 = list(filter(lambda x: x != math.inf and x != -math.inf, right))
        left = list(filter(lambda x: x != math.inf and x != -math.inf, left))

        self.front_avg = sum(front) / len(front)
        self.right_avg = (sum(right)+sum(right2) )/ (len(right)+len(right2))
        self.left_avg = sum(left) / len(left)
        print('-----------------------------')
        print('front_avg= {0}'.format(self.front_avg)) # /scan 개수
        print('right_avg= {0}'.format(self.right_avg)) # /scan 개수
        print('left_avg= {0}'.format(self.left_avg)) # /scan 개수
        print('-----------------------------')

    def callback_timer(self):
        if self.start:
            self.cnt += 1 # callback_odom에서 사용할 변수 -> 일정 시간의 텀을 두기 위해 생성
            if self.right_avg > 0.46 and self.right_avg <= 0.60:
                ## 오른쪽 벽이 가까이 존재
                if self.front_avg >= 0.55: #0.65  # 0.6이 안되는 경우 늘려서 해보기
                    # >= 0.5, 0.6 (0.6 승)
                    ## 앞이 비어있는 경우
                    self.forward(0.2) #0.1
                    print('직진')
                else:
                    ## 오른쪽과 앞이 전부 막혀 있는 경우
                    self.forward(0.0)
                    print('좌회전')
                    self.turn_left(0.65) # 0.79

            elif self.right_avg <= 0.46: #0.50
                self.turn_left(0.2) #0.25
                print('오른쪽 보정 작업')

            else:
                if self.front_avg <= 0.45:
                    self.forward(0.0) #0.1
                    print('오른쪽은 넓은데 정면이 가까움')
                    self.turn_left(0.65) # 0.79
                else:
                    ## 오른쪽 벽이 멀리 존재하는 경우
                    self.forward(0.0)
                    print('우회전')
                    self.turn_right(0.41) #0.5

    def forward(self,x):
        self.cmd_vel.linear.x=x #적당히 빨라야 함
        self.cmd_vel.angular.z=0.0
        self.pub_vel.publish(self.cmd_vel)
        

    def turn_left(self,z):
        self.forward(0.0)# 정지

        self.cmd_vel.angular.z=z
        self.pub_vel.publish(self.cmd_vel)


    def turn_right(self,z):
        self.forward(0.2)# 정지

        self.cmd_vel.angular.z=-z
        self.pub_vel.publish(self.cmd_vel)

    def callbach_odom_val(self,data):
        val=data.pose.pose.position
        if self.start_point:
            self.start_x = round(val.x,3)
            self.start_y = round(val.y,3)
            self.start_point = False
        cur_x=round(val.x,3)
        cur_y=round(val.y,3)
        if self.cnt > 30 and self.start:
            if cur_x>=self.start_x-self.tolerance_x and cur_x<=self.start_x+self.tolerance_x:
                # 현재 x좌표가 시작지점의 x좌표의 오차 내에 있는 경우
                print('x좌표는 처음 위치 근처에 존재')
                if cur_y>=self.start_y-self.tolerance_y and cur_y<=self.start_y+self.tolerance_y:
                    # 현재 y좌표가 시작지점의 y좌표의 오차 내에 있는 경우
                    self.start=False
                    self.forward(0.0)
                    self.save_map() ## 맵을 저장하는 함수
                    print('시작 위치 도달!!!!!!!!!!!')
    
    def save_map(self):
        ## 맵을 저장 하기 위한 코드
        if not self.cli_map_saver.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('error sevice server')
            return
        req = SaveMap.Request()
        req.name.data='map' #str(self.get_clock().now().to_msg())

        future = self.cli_map_saver.call_async(req)
        future.add_done_callback(self.map_save_done)

    def map_save_done(self,future):
        response = future.result()
        if response.success:
            self.get_logger().info('map save done!!')
        else:
            self.get_logger().info('map save fail....')

def main():
    rclpy.init()
    node = Map_tracking()
    rclpy.spin(node)
    