import threading
import copy
import numpy as np
from pack_script import *

counter = 0  # Global counter variable
list_to_recons = []
interval_ms = 250


def async_counter(interval_ms=500):
    global counter
    global timerr
    try:
        #global interval_ms
        counter += 1
        slam_DICT_vals, slam_vals = txt.txt_r("MEDIA\\dependencies\\others\\slam_status.txt", 3)
        slam_local_par = slam_DICT_vals["timesampling"]
        interval_ms = slam_local_par[0]
        #print(interval_ms)
        # Schedule the function again
        timerr = threading.Timer(interval_ms / 1000, async_counter, [interval_ms]).start()
    except (KeyError,IndexError):
        pass

async_counter(interval_ms)  # Increment every 500 ms

def slam_reconstruction(CAMERA_LOCAL_vals, pointcloud_frame):
    global counter
    global list_to_recons

    camera_local_par = CAMERA_LOCAL_vals["camera_stream"]

    #save every 500 ms (2 frames per second)
    try:
        if counter>=2:
            #at least 512 points for every pointcloud frame#np.asanyarray(pointcloud_frame.points).shape[0]>512 and
            if  len(list_to_recons)<camera_local_par[12]: 
                list_to_recons.append(copy.deepcopy(pointcloud_frame))
                #print(f"Counter: {counter}, len: {len(list_to_recons)}")
            counter = 0
    except AttributeError:
        print("NONETYPE CORRECTED")

    txt.txt_w(txt_file="MEDIA\\dependencies\\others\\slam_status.txt",selected_line=2,content_to_replace=f"pcd_buffer:{len(list_to_recons)}")

    #clear saved frames
    if camera_local_par[21]:
        list_to_recons = []
        txt.txt_w(txt_file="MEDIA\\dependencies\\others\\slam_status.txt",selected_line=2,content_to_replace=f"pcd_buffer:{0}")



    #if length of list of point cloud frames are major given for client then make SLAM if BUTTON IS ACTIVED    
    if len(list_to_recons)>=camera_local_par[12] and camera_local_par[20]:
        pack_point_cloud_list(point_cloud_list=list_to_recons)

        