#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Publish a video as ROS messages.
use expample python3 pub_video.py -v GS3_top_L_0521_1_2024-05-21-14-26-58.mp4 -y GS3_top_L_0521_1_2024-05-21-14-26-58.yaml
"""
import numpy as np
import argparse  
import sys
import cv2
import av 
import yaml 
import rospy
import time
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import copy

bridge = CvBridge()

def main():
    """Publish a video as ROS messages.
    """
    parser = argparse.ArgumentParser(description='publish a video as ROS messages')  
    parser.add_argument('-v', '--video', type=str, required=True, help='Video file to play')  
    parser.add_argument('-y', '--yaml', type=str, required=True, help='YAML configuration file')  
    parser.add_argument('-r', '--rate', type=float, default=1.0, help='Playback mulispeed (default: 1.0)')  
    parser.add_argument('-d', '--delay', type=float, default=0.0, help='Playback delay')  
    parser.add_argument('-l', '--loop', type=bool, default=False, help='Playback by loop') 
    parser.add_argument('-s', '--sim', type=bool, default=False, help='use sim time') 
    args = parser.parse_args()  
    print(args.loop)
    videoname = args.video
    yamlname = args.yaml
    delay_time = args.delay
    use_sim_time = args.sim
    with open(yamlname, 'r') as file :
        data = yaml.safe_load(file)
    bagname = data['bagname']
    if rospy.has_param("/use_sim_time"):
        if use_sim_time :
            rospy.set_param("/use_sim_time",True)
        #use_sim_time = rospy.get_param("/use_sim_time")
    else :
        rospy.set_param("/use_sim_time",use_sim_time)

    # Set up node.
    rospy.init_node("video_publisher", anonymous=True)
    img_pub = rospy.Publisher(data['topic_name'][1], Image,
                                queue_size=10)
    info_pub = rospy.Publisher(data['topic_name'][0], CameraInfo,
                                queue_size=10)
      
    

   
    print(f'use_sim_time = {use_sim_time}')

    if use_sim_time :
        print("use sim time play")
        print(f"Publishing {videoname}.")  

        framenum = len(data['img_timestamp'])
        print(len(data['img_timestamp']))
        print(data['img_timestamp'][framenum-1] - data['img_timestamp'][0])
        count = 0
        container = av.open(videoname)
        video_stream= next(s for s in container.streams if s.type == 'video')
        try:  
            for frame in container.decode(video_stream): 
                print('----------------------')
                img_msg = bridge.cv2_to_imgmsg(frame.to_ndarray(format='bgr24'), "bgr8")  
                img_msg.header.stamp = rospy.Time.from_sec(data['img_timestamp'][count])  
                img_msg.header.frame_id = data['camera_info']['frameid']  

                camera_info_message = CameraInfo()  
                camera_info_message.header.stamp = rospy.Time.from_sec(data['img_timestamp'][count])  
                camera_info_message.header.frame_id = data['camera_info']['frameid']  
                camera_info_message.width = data['camera_info']['width']  
                camera_info_message.height = data['camera_info']['height']  
                camera_info_message.distortion_model = data['camera_info']['distortion_model']  
                camera_info_message.K = list(data['camera_info']['K'])  
                camera_info_message.D = list(data['camera_info']['D'])  
                camera_info_message.R = list(data['camera_info']['R'])  
                camera_info_message.P = list(data['camera_info']['P'])  
                
                while True :
                    if rospy.Time.now() > rospy.Time.from_sec(data['img_timestamp'][count]): break
                    if count >= framenum : break
                
                # if rospy.Time.now() < rospy.Time.from_sec(data['img_timestamp'][count]) :
                #     rospy.sleep(rospy.Time.from_sec(data['img_timestamp'][count])-rospy.Time.now())
                print(f"Published timestamp {rospy.Time.now()}") 
                img_pub.publish(img_msg)  
                info_pub.publish(camera_info_message)  
                print(f"Published frame {count} finsh")  

                count = count+1
        except Exception as e:
            print(f"An error occurred: {e}")  


    else :
        print(f"Publishing {videoname}.")
         

        framenum = len(data['img_timestamp'])
        print(f'framenum {framenum}')
        frametime = (data['img_timestamp'][framenum-1]- data['img_timestamp'][0] ) / (framenum-1)
        print(len(data['camera_info']))
        print(data['img_timestamp'][framenum-1] - data['img_timestamp'][0])
        print(frametime)
        
        fps = args.rate * (1/frametime)
        print(f'fps: {fps}')
        rate = rospy.Rate(fps)

        print(f"delay {delay_time} sec")
        #input("press any key to continue")
        time.sleep(delay_time)
        all_time = 0
        s_time = time.time()
        temp_loop_sin = True
        try:  
            while temp_loop_sin :
                #
                temp_loop_sin = args.loop
                count = 0 
                container = av.open(videoname)
                video_stream= next(s for s in container.streams if s.type == 'video')
                try: 
                    for frame in container.decode(video_stream): 
                        print('----------------------')
                        loop_start_time = time.time()
                        #decode_start = time.time()
                        #frame = next(container.decode(video_stream))
                        #decode_end = time.time()
                        #decode_time = decode_end - decode_start
                        #print(f'decode_time : {decode_time}')
                        img_msg = bridge.cv2_to_imgmsg(frame.to_ndarray(format='bgr24'), "bgr8")  
                        img_msg.header.stamp = rospy.Time.from_sec(data['img_timestamp'][count])  
                        img_msg.header.frame_id = data['camera_info']['frameid']  
                        img_pub.publish(img_msg)  
                        print(f"Published frame {count}")  
                
                        camera_info_message = CameraInfo()  
                        camera_info_message.header.stamp = rospy.Time.from_sec(data['img_timestamp'][count])  
                        camera_info_message.header.frame_id = data['camera_info']['frameid']  
                        camera_info_message.width = data['camera_info']['width']  
                        camera_info_message.height = data['camera_info']['height']  
                        camera_info_message.distortion_model = data['camera_info']['distortion_model']  
                        camera_info_message.K = list(data['camera_info']['K'])  
                        camera_info_message.D = list(data['camera_info']['D'])  
                        camera_info_message.R = list(data['camera_info']['R'])  
                        camera_info_message.P = list(data['camera_info']['P'])  
                        info_pub.publish(camera_info_message)  

                        count = count+1
                        # if count >= framenum : 
                        #     print(f'next frame is {count}')
                        #     break

                        rate.sleep()  
                        loop_end_time = time.time()  
                        loop_run_time = loop_end_time - loop_start_time  
                        print(f"loop time is {loop_run_time} seconds") 
                        all_time = all_time + loop_run_time
                        print(f"Video frame runing is  {all_time} seconds") 
                except Exception as e:
                    print("error")
                    continue
                    
        except Exception as e:
            print(f"An error occurred: {e}")  
        
        finally:  
            e_time = time.time()
            r_time = e_time - s_time  
            print(f"Video finished in {r_time} seconds") 
            container.close()  
            rospy.signal_shutdown("Video playback finished.")
            return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
