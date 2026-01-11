import open3d as o3d
import numpy as np
import copy
import txt 
import time
import matrix_rotation as rot
import keyboard
def correct_quadrant(points):
    mins = np.min(points, axis=0)  
    calculated_translation = np.abs(mins)      
    points_translated = points + calculated_translation 
    return points_translated
readed_pcd_path_DICT, readed_pcd_path_STR  = txt.txt_r(file_path="MEDIA\\dependencies\\others\\robot_coords.txt",line_especification=1)
print(readed_pcd_path_DICT)
pcd = o3d.io.read_point_cloud(readed_pcd_path_DICT["pcd_path"][0])#o3d.io.read_point_cloud(pcd_data.paths[0])
#pcd = o3d.io.read_point_cloud("MEDIA\\saved\\reconstructed_pointcloud\\pcdCombinedL444.ply")#o3d.io.read_point_cloud(pcd_data.paths[0])
pcd.rotate(rot.rot_x(alpha=np.deg2rad(90)))


original_points = np.asanyarray(pcd.points)
original_points = correct_quadrant(original_points)
# Intercambiar Y y Z
#original_points[:, [1, 2]] = original_points[:, [2, 1]]
#reconfigure points
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(original_points)
point_cloud.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors))
point_cloud.normals = o3d.utility.Vector3dVector(np.asarray(pcd.normals))


point_cloud_max_coord = point_cloud.get_max_bound()
point_cloud_min_coord = point_cloud.get_min_bound()
max_val = np.max(point_cloud_max_coord)
cord_list = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]


def cb(vis):
    #if keyboard.is_pressed("q"):
    #    exit(0)
    idx = vis.get_picked_points()

    try:
        pointsize_confs, __  = txt.txt_r("MEDIA\\dependencies\\others\\pointcloud_confs.txt",1)
        point_size = pointsize_confs["pointsize"][0]
        vis.get_render_option().point_size = point_size
    except (IndexError,KeyError):
        pass
    #print(idx, dir(idx))
    txt.txt_w("MEDIA/dependencies/others/pcd_info.txt",selected_line=1,content_to_replace=f"num_points:{len(idx)}")

    if len(idx)>0:
        
        index_str = np.array2string(np.asanyarray(idx), separator=', ',max_line_width=2048).strip('[]')
        txt.txt_w("MEDIA/dependencies/others/pcd_info.txt",selected_line=3,content_to_replace=f"index:{index_str}")
        #txt2.txt_w("num_coords.txt",f"{len(idx)}")
        #txt2.txt_w("index.txt",f"{index_str}")

vis = o3d.visualization.VisualizerWithEditing()
vis.create_window(window_name="POINT_SELECTOR",width=int(480),height=int(480),left =10, top =30,visible=True)
vis.get_render_option().background_color= np.asarray([255,255,255])   
vis.get_render_option().show_coordinate_frame = True
vis.get_render_option().light_on = True
vis.get_render_option().point_size = 2.5

vis.reset_view_point()
vis.register_animation_callback(cb)

vis.add_geometry(point_cloud)
#pp = vis.get_picked_points()
#print(dir(pp))
#vis.get_view_control().set_lookat([max_val/2,max_val/2,0])
#vis.get_view_control().set_up([max_val/2,0,0])
vis.run()