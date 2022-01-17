#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math
import time
import signal
import argparse
import asyncio
import pygazebo
import numpy as np
import cv2

from PIL import Image
from pynput import keyboard
from datetime import datetime
from navigation_model import NavigationModel

'''
gz topic -l
gz joint -m 'simple_diff' -j right_wheel_hinge --vel-t 0
gz world --reset-all

renice -n 15 $(pgrep gzclient)
'''

parser = argparse.ArgumentParser()

parser.add_argument('--host', default='localhost', type=str)
parser.add_argument('--port', default=11346, type=int)
parser.add_argument('--retry', default=30, type=int)

parser.add_argument('--robot', default='simple_diff', type=str)
parser.add_argument('--camera', default='camera_link', type=str)
parser.add_argument('--left-wheel', default='left_wheel_hinge', type=str)
parser.add_argument('--right-wheel', default='right_wheel_hinge', type=str)

parser.add_argument('--speed-gain', default=1.0, type=float)
parser.add_argument('--steering-gain', default=0.2, type=float)
parser.add_argument('--steering-kd', default=0.0, type=float)
parser.add_argument('--steering-bias', default=0.0, type=float)

parser.add_argument('--model', default='resnet18', type=str)
parser.add_argument('--dataset', default='data/dataset', type=str)
parser.add_argument('--epochs', default=10, type=int)
parser.add_argument('--batch-size', default=1, type=int)

args = parser.parse_args()
print(args)


# 
# program modes:
#   'collect' (V)  save data while driving with WASD/arrow keys
#   'train'   (T)  train model on collected data
#   'test'    (U)  run model inference, but under user teleop control
#   'infer'   (I)  run the model in autonomous inference mode
#   'reset'   (R)  reset the simulation
#
mode = None

# current robot wheel velocities
wheel_speed = {'left': None, 'right': None}
drive_dir = 'stop'

# load navigation model
nav_model = NavigationModel(args.model, type='regression')

os.makedirs(args.dataset, exist_ok=True)


#
# gazebo connection
#
def gazebo_connect(host='localhost', port=11346, retry=30):
    async def _connect(host, port, retry):
        connected = False
        for i in range(retry):
            try:
                print(f'waiting for gazebo connection {host}:{port} (attempt={i+1})')
                manager = await pygazebo.connect((host, port))
                connected = True
                print(f'connected to gazebo server {host}:{port}')
                break
            except Exception as e:
                print(e)
                pass
            await asyncio.sleep(1)

        if not connected: 
            raise Exception("Timeout connecting to Gazebo.")
            
        return manager

    return asyncio.get_event_loop().run_until_complete(
            _connect(host, port, retry))

def gazebo_advertise(manager, topic_name, msg_type):
    async def _advertise(manager, topic_name, msg_type):
        print(f'advertising {topic_name} ({msg_type})')
        return await manager.advertise(topic_name, msg_type)
        
    return asyncio.get_event_loop().run_until_complete(
        _advertise(manager, topic_name, msg_type))
        
def gazebo_subscribe(manager, topic_name, msg_type, callback):
    async def _subscribe(manager, topic_name, msg_type, callback):
        print(f'subscribing to {topic_name} ({msg_type})')
        subscriber = manager.subscribe(topic_name, msg_type, callback)
        await subscriber.wait_for_connection()
        return subscriber
    
    return asyncio.get_event_loop().run_until_complete(
            _subscribe(manager, topic_name, msg_type, callback))

# connect to gazebo server    
manager = gazebo_connect(args.host, args.port, args.retry)

print('namespaces')
print('  ', manager.namespaces())

print('publications')
for topic in manager.publications():
    print('  ', topic)

# cv window handler
xy_label = None

def cv_click_event(event, x, y, flags, param):
    global xy_label
    
    if event == cv2.EVENT_LBUTTONDOWN:
        print(x,",",y)
        xy_label = (x,y)
        cv2.destroyAllWindows()

# image handler
last_img = None

def on_image(data):
    global mode
    global xy_label
    
    if mode != 'collect' and mode != 'infer' and mode != 'test':
        return
        
    msg = pygazebo.msg.image_stamped_pb2.ImageStamped()
    msg.ParseFromString(data)
    
    img = np.frombuffer(msg.image.data, dtype=np.uint8)
    img = np.reshape(img, (msg.image.height, msg.image.width, 3))
    
    #print(msg.time)
    #print(f'width={msg.image.width} height={msg.image.height} pixel_format={msg.image.pixel_format} step={msg.image.step}')
    #print(img.shape)
    #print('')
    
    if mode == 'collect':
        title = 'Click on path center point'
        cv2.imshow(title, img)
        cv2.setMouseCallback(title, cv_click_event)
        cv2.waitKey(0)
        
        if xy_label is not None:
            img_path = os.path.join(args.dataset, f"xy_{xy_label[0]:03d}_{xy_label[1]:03d}_{datetime.now().strftime('%Y%m%d-%H%M%S-%f')}.jpg")
            Image.fromarray(img).save(img_path)
            print(f"saved {msg.image.width}x{msg.image.height} image to '{img_path}'")
            
        mode = None
        xy_label = None

    elif mode == 'infer' or mode == 'test':
        global last_img
        last_img = Image.fromarray(img)
        
image_subscriber = gazebo_subscribe(manager, f'/gazebo/default/{args.robot}/{args.camera}/camera/image', 'gazebo.msgs.ImageStamped', on_image)
 
# advertise topics
joint_publisher = gazebo_advertise(manager, f'/gazebo/default/{args.robot}/joint_cmd', 'gazebo.msgs.JointCmd')

    
#
# keyboard handler
#
key_states = {}

def on_press(key):
    global key_states

    try:
        key_states[key.char] = True
    except AttributeError:
        key_states[key] = True


def on_release(key):
    global key_states
    global mode
    
    try:
        key_states[key.char] = False
        
        if key.char == 'v':
            mode = 'collect' if mode != 'collect' else None
            print(f"setting mode '{mode}'")
        elif key.char == 't':
            mode = 'train' if mode != 'train' else None
            print(f"setting mode '{mode}'")
        elif key.char == 'u':
            mode = 'test' if mode != 'test' else None
            print(f"setting mode '{mode}'")
        elif key.char == 'i':
            mode = 'infer' if mode != 'infer' else None
            print(f"setting mode '{mode}'")
            
    except AttributeError:
        key_states[key] = False

keyboard_listener = keyboard.Listener(on_press=on_press, on_release=on_release)
keyboard_listener.start()


#
# robot control
#
def set_wheel_speed(left, right):
    global wheel_speed
    changed_speed = False
    
    if wheel_speed['left'] != left:
        left_msg = pygazebo.msg.joint_cmd_pb2.JointCmd()
        left_msg.name = f'{args.robot}::{args.left_wheel}'
        left_msg.velocity.target = left
        joint_publisher.publish(left_msg)
        wheel_speed['left'] = left
        changed_speed = True
        
    if wheel_speed['right'] != right:
        right_msg = pygazebo.msg.joint_cmd_pb2.JointCmd()
        right_msg.name = f'{args.robot}::{args.right_wheel}'
        right_msg.velocity.target = right
        joint_publisher.publish(right_msg)
        wheel_speed['right'] = right
        changed_speed = True
        
    if changed_speed:
        print(f"set_wheel_speed({left:.2f}, {right:.2f})")
 
def set_drive_direction(dir, speed=1.0):
    global drive_dir 
    
    if not isinstance(dir, str):
        dir = data_classes[dir]
    
    if dir == 'stop':
        set_wheel_speed(0,0)
    elif dir == 'forward':
        set_wheel_speed(speed, speed)
    elif dir == 'backward':
        set_wheel_speed(-speed, -speed)
    elif dir == 'left':
        set_wheel_speed(-speed, speed)
    elif dir == 'right':
        set_wheel_speed(speed, -speed)
    else:
        raise ValueError(f"invalid drive direction: {dir}")
 
    drive_dir = dir
    
def teleop(speed=1.0):
    dir = 'stop'
    
    if key_states.get(keyboard.Key.left) or key_states.get('a'):
        dir = 'left'
    elif key_states.get(keyboard.Key.right) or key_states.get('d'):
        dir = 'right'
    elif key_states.get(keyboard.Key.up) or key_states.get('w'):
        dir = 'forward'
    elif key_states.get(keyboard.Key.down) or key_states.get('s'):
        dir = 'backward'
     
    set_drive_direction(dir, speed)     

#
# main loop
#
run_signal = True

def signal_handler(sig, frame):
    global run_signal
    run_signal = False
    print('pressed Ctrl+C, exiting...')

signal.signal(signal.SIGINT, signal_handler)

# main loop
angle_last = 0.0

async def run():
    global mode
    global last_img
    global angle_last
    
    while run_signal:
        if mode == 'train':
            nav_model.train(args.dataset, epochs=args.epochs, batch_size=args.batch_size)
            mode = None
        elif mode == 'infer' or mode == 'test':
            if last_img is not None:
                # predict the center point
                xy = nav_model.infer(last_img)
                
                x = xy[0]
                y = (0.5 - xy[1]) / 2.0
                
                # convert to steering angle
                angle = np.arctan2(x, y)
                print(f'center={xy}  x={x:.2f}  y={y:.2f}  angle={math.degrees(angle):.1f}')
                
                pid = angle * args.steering_gain + (angle - angle_last) * args.steering_kd + args.steering_bias
                
                left = args.speed_gain + pid #max(min(args.speed_gain + pid, 1.0), 0.0)
                right = args.speed_gain - pid #max(min(args.speed_gain - pid, 1.0), 0.0)
    
                print(f'left={left:.2f}  right={right:.2f}  pid={pid}')
                
                if mode == 'infer':
                    set_wheel_speed(left, right)
                
                angle_last = angle

                # visualization
                px = min(max(((xy[0] + 1) / 2) * last_img.size[0], 0), last_img.size[0])
                py = min(max(((xy[1] + 1) / 2) * last_img.size[1], 0), last_img.size[1])
                
                cv_img = cv2.cvtColor(np.array(last_img), cv2.COLOR_BGR2RGB)
                cv_img = cv2.circle(cv_img, (int(px),int(py)), radius=5, color=(50,155,255), thickness=2)
                cv2.imshow("Inference", cv_img)
                cv2.waitKey(1)
                
                last_img = None
                
            if mode == 'test':
                teleop(args.speed_gain)
        else:
            teleop(args.speed_gain)
            
        await asyncio.sleep(0.01)    # TODO should this be run faster?
 
    print('shutting down, stopping robot...')
    set_wheel_speed(0,0)
    await asyncio.sleep(1.0)
    
asyncio.get_event_loop().run_until_complete(run())

    