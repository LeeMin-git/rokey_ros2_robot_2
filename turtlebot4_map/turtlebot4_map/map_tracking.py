import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid,Odometry
from geometry_msgs.msg import Twist
import cv2

class Map_tracking(Node):
    def __init__(self):
        super().__init__('map_tracking')
        self.sub_scan = self.create_subscription(LaserScan,'/scan',self.callback_scan_val,10) # /scan
        self.sub_map = self.create_subscription(OccupancyGrid,'/map',self.callback_map_val,10) # /map
        self.sub_odom = self.create_subscription(Odometry,'/odom',self.callbach_odom_val,10)
        self.timer = self.create_timer(1,self.callback_timer)
        self.pub_vel = self.create_publisher(Twist,'/cmd_vel',10)

        self.cmd_vel=Twist()
        self.tolerance_x = 0.1
        self.tolerance_x = 0.1
        self.cnt = 0
        self.cnt1 = 0
        self.to_map_cur_x = 0.0
        self.to_map_cur_y = 0.0
        self.point_list =[]

    
    def callback_scan_val(self,data):
        pass
    
    def callback_map_val(self,data):
        self.to_map_cur_x=data.info.origin.position.x
        self.to_map_cur_y=data.info.origin.position.y
        resolution=data.info.resolution
        print('to_map_cur_x={0}'.format(self.to_map_cur_x)) 
        print('to_map_cur_y={0}'.format(self.to_map_cur_y)) 
        
        pixel_x = int(self.to_map_cur_x / resolution)
        pixel_y = int(self.to_map_cur_y / resolution)
        

    def callback_timer(self):
        self.cnt1 +=1
        if self.cnt1 > 10:
            if self.cnt < 11:
                if self.cnt%2 == 0:
                    self.forward(1.0)
                    self.point_list.append([self.to_map_cur_x,self.to_map_cur_y])
                elif self.cnt == 1 or self.cnt == 7 or self.cnt == 9 :
                    self.turn_right(1.57)
                    self.point_list.append([self.to_map_cur_x,self.to_map_cur_y])
                elif self.cnt == 3 or self.cnt == 5 or self.cnt == 11:
                    self.turn_left(1.57)
                    self.point_list.append([self.to_map_cur_x,self.to_map_cur_y])
                
                print('{0}번째 저장'.format(self.cnt)) 
                print('{0}번째 저장된 값은 {1}입니다'.format(self.cnt,self.point_list[self.cnt])) 
                self.cnt+=1
            if self.cnt == 11:
                print('list는 {0} 입니다.'.format(self.point_list))

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

    def callbach_odom_val(self,data):
        val=data.pose.pose.position
        self.robot_x=round(val.x,3)
        self.robot_y=round(val.y,3)
        # print('x= {0}, y= {1}'.format(x,y))

def main():
    rclpy.init()
    node = Map_tracking()
    rclpy.spin(node)
    # img=cv2.imread('/home/min/map.pgm')
    # cv2.imshow('test',img)