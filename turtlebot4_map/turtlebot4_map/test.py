import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid,Odometry
from geometry_msgs.msg import Twist

class Map_tracking(Node):
    def __init__(self):
        super().__init__('map_tracking')
        self.sub_scan = self.create_subscription(LaserScan,'/scan',self.callback_scan_val,10) # /scan
        #self.sub_scan = self.create_subscription(OccupancyGrid,'/map',self.callback_map_val,10) # /map
        self.sub_odom = self.create_subscription(Odometry,'/odom',self.callbach_odom_val,10)
        self.timer = self.create_timer(1,self.callback_timer)
        self.pub_vel = self.create_publisher(Twist,'/cmd_vel',10)

        self.cmd_vel=Twist()
        self.tolerance_x = 0.1
        self.tolerance_x = 0.1
        self.cnt = 0

    
    def callback_scan_val(self,data):
        pass
    def callback_timer(self):

        self.cnt+=1
        print('cnt = {0}'.format(self.cnt))

        # if self.right_avg > 0.3 and self.right_avg <= 0.6:
        #     ## 오른쪽 벽이 가까이 존재
        #     if self.front_avg >= 0.7:
        #         ## 앞이 비어있는 경우
        #         self.forward(0.1)
        #         print('직진')
        #     else:
        #         ## 오른쪽과 앞이 전부 막혀 있는 경우
        #         self.forward(0.0)
        #         print('좌회전')
        #         self.turn_left(0.79)
                
        # elif self.right_avg <= 0.3:
        #     self.turn_left(0.20)
        #     print('오른쪽 보정 작업')

        # else:
        #     ## 오른쪽 벽이 멀리 존재하는 경우
        #     self.forward(0.0)
        #     print('우회전')
        #     self.turn_right(0.79)


    def forward(self,x):
        self.cmd_vel.linear.x=x #적당히 빨라야 함
        self.cmd_vel.angular.z=0.0
        self.pub_vel.publish(self.cmd_vel)
        

    def turn_left(self,z):
        self.forward(0.0)# 정지

        self.cmd_vel.angular.z=z
        self.pub_vel.publish(self.cmd_vel)


    def turn_right(self,z):
        self.forward(0.0)# 정지

        self.cmd_vel.angular.z=-z
        self.pub_vel.publish(self.cmd_vel)

    def callback_map_val(self,data):
        val=data.data
        width=data.info.width
        height=data.info.height
        # val_90 = val[30:90]
        # print(len(val),width,height)

    def callbach_odom_val(self,data):
        val=data.pose.pose.position
        cur_x=round(val.x,3)
        cur_y=round(val.y,3)
        # print('x= {0}, y= {1}'.format(x,y))

def main():
    rclpy.init()
    node = Map_tracking()
    rclpy.spin(node)