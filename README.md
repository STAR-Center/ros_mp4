# ros_mp4


## Paper
Li, Jian, Bowen Xu, and Sören Schwertfeger. "High-Quality, ROS Compatible Video Encoding and Decoding for High-Definition Datasets." arXiv preprint arXiv:2408.00538 (2024).


	@article{li2024high,
	  title={High-Quality, ROS Compatible Video Encoding and Decoding for High-Definition Datasets},
	  author={Li, Jian and Xu, Bowen and Schwertfeger, S{\"o}ren},
	  journal={arXiv preprint arXiv:2408.00538},
	  year={2024}
	}


## Scripts

Script for compressing a ROS 1 video bagfile into an av1 mp4 plus yaml meta data:

rosbagD2av1.py

Script for playing back mp4 plus yaml meta data on ROS 1:

pub_video.py

ROS 2:

pub_video_ros2.py

Needs ffmpeg with the avi codec installed.
