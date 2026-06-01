#!/usr/bin/env python
# coding=utf-8

import os
import select
import sys
import rclpy

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from step_motor.msg import Motor
if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1


msg = """
Control Your motor!
---------------------------
   q    w        r
   a    s    d   f
   z    x        v

q/a : increase/decrease resolution_ratio
r/v : increase/decrease id 
w/s/x : w/x increase/decrease speed  s stop 
z: change direction
f: send motor state request
z: sub-divide
CTRL-C to quit
"""
e = """
Communications Failed
"""

#获取键值函数
def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    motor=Motor()
    motor.speed=0
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('wheeltec_keyboard')
    pub = node.create_publisher(Motor, 'motor_control', qos)
    resolution_ratio=1
    temp_speed=0
    pow=1
    motor.id=1
    try:
        print(msg)
        while(1):
            key = get_key(settings)
            #切换是否为全向移动模式，全向轮/麦轮小车可以加入全向移动模式
            if key=='w':
                motor.mode=1
                motor.speed+=resolution_ratio
                print("current speed:",motor.speed,end="\n")
                pub.publish(motor)
            if key=='s':
                temp_speed=motor.speed
                motor.mode=1
                motor.speed=0
                print("stop\n")
                pub.publish(motor)
                motor.speed=temp_speed
            if key=='x':
                motor.mode=1
                if motor.speed>=resolution_ratio:
                    motor.speed-=resolution_ratio
                print("current speed:",motor.speed,end="\n")
                pub.publish(motor)
            
            if key=='d':               
                motor.dir=not motor.dir
                if motor.dir==0:
                    print ("foreward rotation\n")
                else:
                    print ("reverse rotation\n")
                pub.publish(motor)

            if key=='f':
                motor.state=1
                pub.publish(motor)
                print ("send motor state request\n")
                motor.state=0

            if key=='q':
                resolution_ratio+=1
                print("current resolution_ratio:",resolution_ratio,end="\n")
                pub.publish(motor)
            if key=='a':
                if resolution_ratio>1:
                    resolution_ratio-=1
                print("current resolution_ratio:",resolution_ratio,end="\n")
                pub.publish(motor)
            if key=='z':
                motor.sub_divide=2**pow
                pub.publish(motor)
                if pow<5:
                    pow+=1
                else:
                    pow=0
                print("current sub-divide:",motor.sub_divide)

            if key=='r':
                motor.id+=1
                print("current motor id:",motor.id,end="\n")
                pub.publish(motor)
            if key=='v':
                if motor.id>1:
                    motor.id-=1
                print("current motor id:",motor.id,end="\n")
                pub.publish(motor)
                

            #长期识别到不明键值，相关变量置0
            if (key == '\x03'):
                    break
         
            

    except Exception as e:
        print(e)

    finally:
        # twist = Twist()
        # twist.linear.x = 0.0
        # twist.linear.y = 0.0
        # twist.linear.z = 0.0

        # twist.angular.x = 0.0
        # twist.angular.y = 0.0
        # twist.angular.z = 0.0

        # pub.publish(twist)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
