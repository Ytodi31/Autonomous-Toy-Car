#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import sys, select, os

if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

MAX_LIN_VEL = 10
MAX_ANG_VEL = 0.5

LIN_VEL_STEP_SIZE = 0.5
ANG_POS_STEP_SIZE = 0.1

msg = """
Control the Toy Car!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity [step size 0.5 m/s]
a/d : turn left/right [step size 0.1 rad]
space key, s : force stop
CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_pos):
    return "currently:\tlinear vel %s\t steering angle (rad) %s " % (target_linear_vel,target_angular_pos)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)
    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -MAX_ANG_VEL, MAX_ANG_VEL)
    return vel

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("toycar_teleop")
    leftwheel_vel = rospy.Publisher("/toycar/leftwheel_velocity_controller/command", Float64, queue_size = 100)
    rightwheel_vel = rospy.Publisher("/toycar/rightwheel_velocity_controller/command", Float64, queue_size = 100)
    left_steer = rospy.Publisher("/toycar/leftjoint_position_controller/command", Float64, queue_size = 100)
    right_steer = rospy.Publisher("/toycar/rightjoint_position_controller/command", Float64, queue_size = 100)

    status = 0
    target_linear_vel   = 0.0
    target_angular_pos  = 0.0
    control_linear_vel  = 0.0
    control_angular_pos = 0.0

    velocity = Float64(0)

    try:
        print(msg)
        while(1):
            key = getKey()
            if key == 'w' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_pos))
            elif key == 'x' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_pos))
            elif key == 'a' :
                target_angular_pos = checkAngularLimitVelocity(target_angular_pos - ANG_POS_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_pos))
            elif key == 'd' :
                target_angular_pos = checkAngularLimitVelocity(target_angular_pos + ANG_POS_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_pos))
            elif key == ' ' or key == 's' :
                target_linear_vel   = 0.0
                target_angular_pos  = 0.0
                print(vels(target_linear_vel,target_angular_pos))
            else:
                if (key == '\x03'):
                    break
            if status == 20 :
                print(msg)
                status = 0

            # twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            velocity = Float64(control_linear_vel)

            control_angular_pos = makeSimpleProfile(control_angular_pos, target_angular_pos, (ANG_POS_STEP_SIZE/2.0))
            steer_angle = Float64(control_angular_pos)

            leftwheel_vel.publish(velocity)
            rightwheel_vel.publish(velocity)
            left_steer.publish(steer_angle)
            right_steer.publish(steer_angle)


    except:
        print(e)

    finally:
        velocity.data = 0
        leftwheel_vel.publish(velocity)
        rightwheel_vel.publish(velocity)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
