import open3d as o3d
import numpy as np
import matrix_rotation as rot
import txt
import keyboard 
import time
np.set_printoptions(precision=4, suppress=True)

import subprocess

############################################################################################################################################################
def correct_quadrant(points):
    mins = np.min(points, axis=0)  
    calculated_translation = np.abs(mins)      
    points_translated = points + calculated_translation 
    return points_translated
############################################################################################################################################################
def fill_arrows(max_coords=0):
    global point_coord_arrow_list
    global last_coord
    global coord_frame_list
    point_coord_arrow_list = list()
    last_coord = list()
    coord_frame_list = list()
    for i in range(0,max_coords):
        #arrow frame to indicate orientation of robot
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(0.25)
        arrow_frame = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=0.02,cone_radius=0.075,cone_height=0.05,resolution =20,cylinder_height =0.325)
        arrow_frame.translate([0,0,0])
        coord_frame.translate([0,0,0])
        arrow_frame.rotate(rot.rot_y(np.deg2rad(90)))

        try:
            pcd_coords_configs , __ = txt.txt_r("MEDIA/dependencies/others/robot_coords.txt",3)
            pcd_coords_configs = pcd_coords_configs["pcd_coords_configs"]
            pcd_coords_configs[0]
            #print(pcd_coords_configs)
            if i == 0 or i == max_coords-1:
                if i == 0:
                    arrow_frame.rotate(rot.rot_z(np.deg2rad(-pcd_coords_configs[0])))
                    arrow_frame.paint_uniform_color([0,1,0]) 

                if i == max_coords-1:
                    arrow_frame.paint_uniform_color([1,1,0]) 
                    arrow_frame.rotate(rot.rot_z(np.deg2rad(-pcd_coords_configs[1])))
            else:
                arrow_frame.paint_uniform_color([1, 0, 0]) 
        except:
            print("Error STATIC PCD COORDS")

        arrow_frame.compute_vertex_normals()
        point_coord_arrow_list.append(arrow_frame)
        coord_frame_list.append(coord_frame)
        #reset memory last coords
        last_coord.append([0,0,0])
        #print(dir(arrow_frame))
        vis.add_geometry(point_coord_arrow_list[i])
        vis.add_geometry(coord_frame_list[i])
        vis.update_geometry(point_coord_arrow_list[i])
        vis.update_geometry(coord_frame_list[i])
    try:
        vis.set_view_status(last_view_status)
    except TypeError:
        print("Empty view status")

############################################################################################################################################################
point_coord_arrow_list = list()#[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
last_coord = list()#[[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
coord_frame_list = list()
last_rotation = 0
last_view_status = None
charr = ""
############################################################################################################################################################
def rot_struct(angle = 0):
    global last_rotation
    last_rotation = last_rotation+angle
    point_cloud.rotate(rot.rot_z(beta=np.deg2rad(angle)))
    remove_geometries()

def trans_struct(direction,translation = 0):
    if direction == "up":
        point_cloud.translate([0,translation,0])
    if direction == "down":
        point_cloud.translate([0,-translation,0])
    if direction == "left":
        point_cloud.translate([-translation,0,0])
    if direction == "right":
        point_cloud.translate([translation,0,0])
    remove_geometries()
############################################################################################################################################################
def remove_geometries():
    global coord_frame_list
    global point_coord_arrow_list
    global vis
    for i in range(0,len(point_coord_arrow_list)):
        vis.remove_geometry(point_coord_arrow_list[i])
        vis.remove_geometry(coord_frame_list[i])
############################################################################################################################################################
def cb(vis):

    #if keyboard.is_pressed("q"):
    #    exit(0)
        
    global last_rotation
    global last_view_status
    status = vis.get_view_status()

    if status != last_view_status:
        last_view_status = status

    try:

        num_points , __ = txt.txt_r("MEDIA/dependencies/others/pcd_info.txt",1)
        indexs , ___    = txt.txt_r("MEDIA/dependencies/others/pcd_info.txt",3)
        pointsize_confs, __  = txt.txt_r("MEDIA\\dependencies\\others\\pointcloud_confs.txt",1)

        num_coords = num_points["num_points"][0]
        idxs = indexs["index"]
        point_size = pointsize_confs["pointsize"][0]

        vis.get_render_option().point_size = point_size

        if keyboard.is_pressed('+'):
            rot_struct(angle=-0.5)
            fill_arrows(max_coords=num_coords)
        elif keyboard.is_pressed('-'):
            rot_struct(angle=0.5)
            fill_arrows(max_coords=num_coords)
        elif keyboard.is_pressed('enter'):
            remove_geometries()
            fill_arrows(max_coords=num_coords)
        elif keyboard.is_pressed("ctrl + up"):
            trans_struct(direction="up",translation=0.05)
            fill_arrows(max_coords=num_coords)
        elif keyboard.is_pressed("ctrl + down"):
            trans_struct(direction="down",translation=0.05)
            fill_arrows(max_coords=num_coords)
        elif keyboard.is_pressed("ctrl + left"):
            trans_struct(direction="left",translation=0.05)
            fill_arrows(max_coords=num_coords)
        elif keyboard.is_pressed("ctrl + right"):
            trans_struct(direction="right",translation=0.05)
            fill_arrows(max_coords=num_coords)

        #mcord  = point_cloud.points[idxs]
        #print(last_coord)
        #print("mcoords: ",mcord)
        for i in range(0,num_coords):
            try:

                coord = point_cloud.points[idxs[i]]
                #print(coord)
                if coord[0] != last_coord[i][0] and coord[1] != last_coord[i][1] and coord[2] != last_coord[i][2]:#last_coord[i]!=coord:
                    #arrows coord translate and rotate
                    point_coord_arrow_list[i].translate(coord)
                    #point_coord_arrow_list[i].rotate(rot.rot_z(beta=np.deg2rad(last_rotation)))
                    #origin frame coord translate 
                    coord_frame_list[i].translate(coord)
                    last_coord[i] = np.round(coord, 4)
                    # Convertir a lista de listas y luego a string
                    formatted_str = "[" + ",".join(f"[{','.join(map(str, a))}]" for a in last_coord) + "]"
                    #print(formatted_str)
                    txt.txt_w("MEDIA/dependencies/others/robot_coords.txt",selected_line=2,content_to_replace=f"pcd_coords:{formatted_str}")

     
            except KeyError:
                pass
            finally:
                pass
        


        vis.update_geometry(point_cloud)
        vis.poll_events()
        vis.update_renderer()  
    except Exception as E:
        pass
        #print("Error", E)
##############################################################################
readed_pcd_path_DICT, readed_pcd_path_STR  = txt.txt_r(file_path="MEDIA\\dependencies\\others\\robot_coords.txt",line_especification=1)
print(readed_pcd_path_DICT)
pcd = o3d.io.read_point_cloud(readed_pcd_path_DICT["pcd_path"][0])#o3d.io.read_point_cloud(pcd_data.paths[0])
pcd.rotate(rot.rot_x(alpha=np.deg2rad(90)))

original_points = np.asanyarray(pcd.points)
original_points = correct_quadrant(original_points)

#reconfigure points
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(original_points)
point_cloud.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors))
point_cloud.normals = o3d.utility.Vector3dVector(np.asarray(pcd.normals))

point_cloud_max_coord = point_cloud.get_max_bound()
point_cloud_min_coord = point_cloud.get_min_bound()
max_val = np.max(point_cloud_max_coord)
print(f"max: {point_cloud_max_coord} , min: {point_cloud_min_coord}")
point_cloud_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(2.5)
origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(max_val)
point_cloud_middle = point_cloud.get_center()

#########################################################################
vis = o3d.visualization.Visualizer() #Anyone camera have a visualizar for local stream
vis.create_window(window_name="COORDINATE_CONFIGURATOR",width=int(480),height=int(480),left =500, top =30,visible=True)
vis.get_render_option().background_color= np.asarray([1,1,1])   
vis.get_render_option().show_coordinate_frame = False
vis.get_render_option().light_on = True
#vis.get_render_option().point_size = point_size
vis.add_geometry(origin_frame)
vis.add_geometry(point_cloud)
#vis.get_view_control().set_lookat([max_val/2,max_val/2,0])
#vis.get_view_control().set_up([max_val/2,0,0])
#create coords
fill_arrows()
vis.register_animation_callback(cb)
vis.run()
