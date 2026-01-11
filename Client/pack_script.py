import subprocess
import pickle
import tempfile
import open3d  as o3d
import numpy as np
import txt

RUN_SLAVE_FLAG = True
process = None

def pack_point_cloud_list(point_cloud_list):
    global RUN_SLAVE_FLAG

    slam_DICT_vals, slam_vals = txt.txt_r("MEDIA\\dependencies\\others\\slam_status.txt", 1)
    slam_local_par = slam_DICT_vals["slam_status"]
    global process

    #print(CAMERA_vals[:-3])

    if  slam_local_par[0]:

        pcd_info_list = []

        for i in range(len(point_cloud_list)):
            if point_cloud_list[i] is not None:
                pcd_info = [[np.asanyarray(point_cloud_list[i].points)],[np.asanyarray(point_cloud_list[i].colors)],[np.asanyarray(point_cloud_list[i].normals)]]
                #print(f"packing: {len(pcd_info_list)} {cam_local_par[20]} {RUN_SLAVE_FLAG}")
                pcd_info_list.append(pcd_info)

        # Save object using pickle
        with tempfile.NamedTemporaryFile(mode="wb", delete=False) as temp_file:
            pickle.dump(pcd_info_list, temp_file)
            temp_file_path = temp_file.name
            #print("FILE PATH: ",temp_file_path)

        #subprocess.run(["python", "SLAM_nonblocking_subprocess.py", temp_file_path])
        process = subprocess.Popen(["python", "SLAM_nonblocking_subprocess.py", temp_file_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                                   creationflags=subprocess.CREATE_NEW_CONSOLE)  # Opens a new console)
        txt.txt_w(txt_file="MEDIA\\dependencies\\others\\slam_status.txt",selected_line=1,content_to_replace=f"slam_status:{0},{1}")
        #RUN_SLAVE_FLAG = False
    #else:
        #RUN_SLAVE_FLAG = True

'''
point_cloud_LS = []

#point_cloud = pcd = o3d.io.read_point_cloud("MEDIA\\saved\\reconstructed_pointcloud\\pcdCombinedL333.ply")
#print(np.asanyarray(point_cloud.points))
for i in range(0,10):
    point_cloud = o3d.io.read_point_cloud("MEDIA\\saved\\reconstructed_pointcloud\\pcdCom777.ply")

    #pcd_info = [[np.asanyarray(point_cloud.points)],[np.asanyarray(point_cloud.colors)],[np.asanyarray(point_cloud.normals)]]
    point_cloud_LS.append(point_cloud)

while True:
    pack_point_cloud_list(point_cloud_LS)'
'''