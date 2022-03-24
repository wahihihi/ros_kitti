#!/usr/bin/env python3

from data_utils import *
from publish_utils import *
import os
from kitti_utils import *

DATA_PATH = '/home/swh/data/kitti/RawData/2011_09_26/2011_09_26_drive_005_sync/'
             
TRACKING_PATH = '/home/swh/data/kitti/tracking/training/label_02/0000.txt'

def compute_3d_box_cam2(h,w,l,x,y,z,yaw):
    """
    Return: 3xn in cam2 coordinate
    """
    R = np.array([[np.cos(yaw),0,np.sin(yaw)],[0,1,0],[-np.sin(yaw),0,np.cos(yaw)]])
    x_corners = [l/2, l/2, -l/2, -l/2,  l/2,  l/2, -l/2, -l/2]
    y_corners = [0,   0,    0,    0,   -h,   -h,   -h,   -h  ]
    z_corners = [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2  ]
    corners_3d_cam2 = np.dot(R,np.vstack([x_corners,y_corners,z_corners]))
    corners_3d_cam2 += np.vstack([x,y,z])
    return corners_3d_cam2
    

if __name__ == '__main__':
	frame = 0
	rospy.init_node('kitti_node',anonymous=True)
	cam_pub = rospy.Publisher('kitti_cam',Image,queue_size=10)
	pcl_pub = rospy.Publisher('kitti_pcl',PointCloud2,queue_size=10)
	ego_pub = rospy.Publisher('kitti_ego_car',MarkerArray,queue_size=10)
	imu_pub = rospy.Publisher('kitti_imu',Imu,queue_size=10)
	gps_pub = rospy.Publisher('kitti_gps', NavSatFix, queue_size=10)
	box3d_pub = rospy.Publisher('kitti_3d',MarkerArray,queue_size=10)
	bridge = CvBridge()

	df_tracking = read_tracking(TRACKING_PATH)
	calib = Calibration('/home/swh/data/kitti/RawData/2011_09_26/',from_video=True)
	
	rate=rospy.Rate(10)
	while not rospy.is_shutdown():
		df_tracking_frame = df_tracking[df_tracking.frame==frame]	
		
		boxes_2d = np.array(df_tracking_frame[df_tracking_frame.frame == frame][['bbox_left', 'bbox_top', 'bbox_right', 'bbox_bottom']])
		types = np.array(df_tracking_frame[df_tracking_frame.frame == frame]['type'])
		boxes_3d = np.array(df_tracking_frame[['height','width','length','pos_x','pos_y','pos_z','rot_y']])
		
		corners_3d_velos = []
		for box_3d in boxes_3d:
			corners_3d_cam2 = compute_3d_box_cam2(*box_3d)
			corners_3d_velo = calib.project_rect_to_velo(corners_3d_cam2.T)
			corners_3d_velos += [corners_3d_velo]
		
		img = read_camera(os.path.join(DATA_PATH,'image_02/data/%010d.png'%frame))
		point_cloud = read_point_cloud(os.path.join(DATA_PATH,'velodyne_points/data/%010d.bin'%frame))
		imu_data = read_imu(os.path.join(DATA_PATH,'oxts/data/%010d.txt'%frame))
		
		
		publish_3dbox(box3d_pub,corners_3d_velos)
		publish_camera(cam_pub,bridge,img,boxes_2d,types)
		publish_point_cloud(pcl_pub,point_cloud)
		publish_ego_car(ego_pub)
		publish_imu(imu_pub,imu_data)
		publish_gps(gps_pub,imu_data)

		rospy.loginfo("camera image and point cloud published")
		rate.sleep()
		frame += 1
		frame %= 154
