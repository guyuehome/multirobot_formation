#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Ares!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

p : change move style
a : open/close laser
s : start/stop gun shooting
d/f : increase/decrease PTZ pitch angle
g/h : increase/decrease PTZ yaw angle
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0),
        'o':(1,-1,-1),
        'j':(0,1,1),
        'l':(0,-1,-1),
        'u':(1,1,1),
        ',':(-1,0,0),
        '.':(-1,-1,1),
        'm':(-1,1,-1),
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 0.2
turn = 4

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('ares_teleop')
    vel_pub = rospy.Publisher('cmd_vel',  Twist,   queue_size=1)

    x = 0
    y = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed_x = 0
    control_speed_y = 0
    control_turn = 0
    move_style = 0
    laser_open = 0
    gun_fire = 0
    gun_pitch_angle = 185
    gun_yaw_angle=185
    mask = 0
    try:
        print msg
        print vels(speed,turn)
        while(1):
            key = getKey()
            # 运动控制方向键（1：正方向，-1负方向）
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                th = moveBindings[key][2]
                count = 0
            # 速度修改键
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]  # 线速度增加0.1倍
                turn = turn * speedBindings[key][1]    # 角速度增加0.1倍
                count = 0

                print vels(speed,turn)
                if (status == 14):
                    print msg
                status = (status + 1) % 15
            # 停止键
            elif key == ' ' or key == 'k' :
                x = 0
                y = 0
                th = 0
                control_speed_x = 0
                control_speed_y = 0
                control_turn = 0
            # 切换全向移动模式
            elif key == 'p': 
                if move_style:
                    move_style = 0
                else:
                    move_style = 1
                print "Change move style"
            # 瞄准器
            elif key == 'a':
                mask = 2
                if laser_open:
                    laser_open = 0
                    print "Close laser"
                else:
                    laser_open = 1
                    print "Open laser"
            # 开枪
            elif key == 's':
                mask = 1
                if gun_fire:
                    gun_fire = 0
                    print "Gun stop shooting"
                else:
                    gun_fire = 1
                    print "Gun shooting"

                count = 0
            # 调整角度
            #pitch
            elif key == 'd':
                mask = 4
                gun_pitch_angle = gun_pitch_angle + 5
                gun_pitch_angle = min(gun_pitch_angle, 195)   
                print "Increase PTZ pitch angle"  
            elif key == 'f':
                mask = 4
                gun_pitch_angle = gun_pitch_angle - 5
                gun_pitch_angle = max(gun_pitch_angle, 175)   
                print "Decrease PTZ pitch angle"   
            #yaw 
            elif key == 'g':
                mask = 8
                gun_yaw_angle = gun_yaw_angle + 5
                gun_yaw_angle = min(gun_yaw_angle, 195)   
                print "Increase PTZ yaw angle"  
            elif key == 'h':
                mask = 8
                gun_yaw_angle = gun_yaw_angle - 5
                gun_yaw_angle = max(gun_yaw_angle, 175)   
                print "Decrease PTZ yaw angle"
            else:
                '''
                count = count + 1
                if count > 30:
                    x = 0
                    y = 0
                    th = 0
                    gun_fire = 0
                '''
                if (key == '\x03'):
                    break

            # 目标速度=速度值*方向值
            if move_style:
                target_speed_x = speed * x
                target_speed_y = speed * y
                target_turn = 0
            else:
                target_speed_x = speed * x
                target_speed_y = 0
                target_turn = turn * th
            
            # 速度限位，防止速度增减过快
            if target_speed_x > control_speed_x:
                control_speed_x = min( target_speed_x, control_speed_x + 10 )
            elif target_speed_x < control_speed_x:
                control_speed_x = max( target_speed_x, control_speed_x - 10 )
            else:
                control_speed_x = target_speed_x
        
            if target_speed_y > control_speed_y:
                control_speed_y = min( target_speed_y, control_speed_y + 10 )
            elif target_speed_y < control_speed_y:
                control_speed_y = max( target_speed_y, control_speed_y - 10 )
            else:
                control_speed_y = target_speed_y

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 10 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 10 )
            else:
                control_turn = target_turn
            
            # 创建并发布twist消息
            twist = Twist()
            twist.linear.x = control_speed_x; 
            twist.linear.y = control_speed_y; 
            twist.linear.z = 0
            twist.angular.x = 0; 
            twist.angular.y = 0; 
            twist.angular.z = control_turn
            vel_pub.publish(twist)


    except:
        print("error")

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        vel_pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
