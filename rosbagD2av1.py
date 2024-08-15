import rosbag  
import cv2  
from cv_bridge import CvBridge 
import subprocess  
import numpy as np  
import yaml  
import os  
import json 

bridge = CvBridge()

# bag_file_path = '../bagdocument/GS3_left_L_0409_1_2024-04-09-15-18-27.bag'
# topic_name = '/GS3_left_L/image_raw'
# output_video = 'GS3leftL0409.mp4' 
# camera_topic = '/GS3_left_L/camera_info'
#compression = False

with open('baglist.json', 'r') as file:
    datas = json.load(file)
# 
# 
# [{"bag_name":"","bag_path":"","camera_topic":"","img_topic":"","output_name":"","fps":""}]
# 

def start_ffmpeg_process(output_video, width, height, framerate):  
    command = [  
        '/home/jian/Documents/ffmpeg-master-latest-linux64-gpl/bin/ffmpeg',  
        '-y',  
        '-framerate',framerate,
        '-f', 'rawvideo', 
        '-pixel_format', 'bgr24',  
        '-s', '{}x{}'.format(width, height), 
        '-i', '-',  
        '-c:v', 'libsvtav1',  
        '-crf', '22',
        '-preset','6',
        '-r',framerate,
        output_video 
    ]  
    return subprocess.Popen(command, stdin=subprocess.PIPE) 

def process_bag(bag_name,bag_file_path,camera_topic, img_topic, ffmpeg_process,output_name,framerate,compression):  
    dataset_info = {
    'bagname':f'{bag_name}.bag',
    'topic_name':[camera_topic,img_topic],
    'camera_info':{},  
    'img_timestamp':[]
}
    with rosbag.Bag(bag_file_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[camera_topic]):
            camera_info = {  
                "frameid": msg.header.frame_id,
                #"timestamp": t.to_sec(),
                "height": msg.height,
                "width": msg.width,
                "distortion_model": msg.distortion_model,
                "D": list(msg.D),
                "K": list(msg.K),
                "R": list(msg.R),
                "P": list(msg.P)
            }
            
            dataset_info['camera_info'] = camera_info
            height = msg.height
            width = msg.width
            break
            

        for topic, msg, t in bag.read_messages(topics=[img_topic]):
                dataset_info['img_timestamp'].append( t.to_sec())
                if compression == 0 :
                    bayer_np = np.frombuffer(msg.data, np.uint8).reshape(height, width) 
                    bgr_image = cv2.cvtColor(bayer_np, cv2.COLOR_BayerGBRG2BGR)
                else:
                    np_arr = np.frombuffer(msg.data, dtype=np.uint8)
                    compressed_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)  
                    bgr_image = cv2.cvtColor(compressed_image, cv2.COLOR_BayerGBRG2BGR)     
                #height, width = bgr_image.shape[:2]  
                if ffmpeg_process is None:  
                    #framerate = bag.get_message_count(topic_name) / bag.get_duration()  
                    output_video = f'{output_name}.mp4'
                    ffmpeg_process = start_ffmpeg_process(output_video, width, height, framerate)
                bgr_image_raw = bgr_image.tobytes() 
                ffmpeg_process.stdin.write(bgr_image_raw) 

        with open(f'{output_name}.yaml', 'w') as f:  
            yaml.dump(dataset_info, f, default_flow_style=False, sort_keys=False)
        
    if ffmpeg_process:  
        ffmpeg_process.stdin.close()  
        ffmpeg_process.wait()  


for data in datas:
    process_bag(data['bag_name'],data['bag_path'],data['camera_topic'],data['img_topic'], None,data['output_name'],data['fps'],data['compression'])
