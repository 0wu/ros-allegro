#!/usr/bin/env python
import rclpy
import math
from allegro_hand_controllers.allegro_robot import AllegroRobot
import threading

TORQUE_SCALE = -0.1

rclpy.init()

hand = AllegroRobot(hand_topic_prefix='/allegroHand')

threading.Thread(target=rclpy.spin, args=[hand]).start()
hz = 20
ros_rate = hand.create_rate(hz)
desired_torques = [TORQUE_SCALE]*16

print('Holding home configuration for 5 seconds...')
for i in range(100):
  hand.command_joint_torques(desired_torques)
  ros_rate.sleep()
  
home_pos, _ = hand.poll_joint_position(wait=True)
print('Home joint configuration:')
print(home_pos)

hand.disconnect()
hand.destroy_node()
rclpy.shutdown()
