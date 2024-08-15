import argparse
import threading
import time

import av
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.clock import Clock
from rclpy.clock import ROSClock
from rclpy.time import Time
from rclpy.time_source import TimeSource
#from rclpy.clock_type import ClockType
from rclpy.parameter import Parameter




bridge = CvBridge()


class publishvideo_nomal:

    def __init__(self, rate,loop,videoname,data,img_pub,info_pub):
        self.videoname = videoname
        self.img_pub = img_pub
        self.info_pub = info_pub
        # self.data = data
        self.done = False
        self.loop = loop
        self.temp_loop_sin = True
        self._thread = threading.Thread(target=self._run, args=(rate,data), daemon=True)
        self._thread.start()

    def _run(self,rate,data):
        all_time = 0
        try:
            while self.temp_loop_sin :
                self.temp_loop_sin = self.loop
                count = 0 
                container = av.open(self.videoname)
                video_stream= next(s for s in container.streams if s.type == 'video')
                try: 
                    for frame in container.decode(video_stream): 
                            print('----------------------')
                            loop_start_time = time.time()

                            img_msg = bridge.cv2_to_imgmsg(frame.to_ndarray(format='bgr24'), "bgr8")  
                            img_msg.header.stamp = Time(seconds=data['img_timestamp'][count]).to_msg()
                            img_msg.header.frame_id = data['camera_info']['frameid'] 
                            
                    
                            camera_info_message = CameraInfo()  
                            camera_info_message.header.stamp = Time(seconds=data['img_timestamp'][count]).to_msg()
                            camera_info_message.header.frame_id = data['camera_info']['frameid']  
                            camera_info_message.width = data['camera_info']['width']  
                            camera_info_message.height = data['camera_info']['height']  
                            camera_info_message.distortion_model = data['camera_info']['distortion_model']  
                            camera_info_message.k = list(data['camera_info']['K'])  
                            camera_info_message.d = list(data['camera_info']['D'])  
                            camera_info_message.r = list(data['camera_info']['R'])  
                            camera_info_message.p = list(data['camera_info']['P'])  
                            
                            self.img_pub.publish(img_msg)  
                            self.info_pub.publish(camera_info_message)  
                            print(f"Published frame {count}") 
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
                    print(e)
                    continue
        finally:
            self.done = True

    



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

    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('videopublish', context=context)

    node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL,use_sim_time)])

    useSimTime = node.get_parameter("use_sim_time")
    print(f'use_sim_time value {useSimTime.value}')

    img_pub = node.create_publisher(Image,data['topic_name'][1], 0)
    info_pub = node.create_publisher( CameraInfo,data['topic_name'][0],10)
    clock = Clock()
    rosclock = ROSClock()
    if use_sim_time :
        time_source = TimeSource(node=node)
        time_source.attach_clock(rosclock)
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
                img_msg.header.stamp = Time(seconds=data['img_timestamp'][count]).to_msg()
                img_msg.header.frame_id = data['camera_info']['frameid']  

                camera_info_message = CameraInfo()  
                camera_info_message.header.stamp = Time(seconds=data['img_timestamp'][count]).to_msg()
                camera_info_message.header.frame_id = data['camera_info']['frameid']  
                camera_info_message.width = data['camera_info']['width']  
                camera_info_message.height = data['camera_info']['height']  
                camera_info_message.distortion_model = data['camera_info']['distortion_model']  
                camera_info_message.k = list(data['camera_info']['K'])  
                camera_info_message.d = list(data['camera_info']['D'])  
                camera_info_message.r = list(data['camera_info']['R'])  
                camera_info_message.p = list(data['camera_info']['P'])  
                # print(f'当前ros时间 {rosclock.now()}')
                while True :
                    #print(f'ROS2 time stamp {rosclock.now().nanoseconds }')
                    if rosclock.now().nanoseconds > Time(seconds=data['img_timestamp'][count]).nanoseconds: break
                    if count >= framenum : break
                
                print(f"Published timestamp {rosclock.now()}") 
                img_pub.publish(img_msg)  
                info_pub.publish(camera_info_message)  
                print(f"Published frame {count} finsh")  

                count = count+1
        except Exception as e:
            print(f"An error occurred: {e}")  

    else:

        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        print(f"Publishing {videoname}.")
         

        framenum = len(data['img_timestamp'])
        print(f'framenum {framenum}')
        frametime = (data['img_timestamp'][framenum-1]- data['img_timestamp'][0] ) / (framenum-1)
        print(len(data['camera_info']))
        print(data['img_timestamp'][framenum-1] - data['img_timestamp'][0])
        print(frametime)
        
        fps = args.rate * (1/frametime)
        rate = node.create_rate(fps)
        print(f'fps: {fps}')
        print(f"delay {delay_time} sec")
        #input("press any key to continue")
        time.sleep(delay_time)
        publishvideo_nomal_thread = publishvideo_nomal(rate,args.loop,videoname,data,img_pub,info_pub)
        while not publishvideo_nomal_thread.done :
            executor.spin_once()
       


if __name__ == '__main__':
    main()