import cv2
import numpy as np
from rescale_frame import rescaleFrame
import copy
import open3d as o3d
import txt
import matrix_rotation as rot


camera_open3d_intrinsic_parameters = o3d.camera.PinholeCameraIntrinsic()
RENDER_FLAG = False
vis, frame, point_cloud, camera_gyro_mesh_vis, camera_origin_mesh_vis = None, None, None, None, None
#********************************************************************************************************************************
def build_render_open3d():
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(1.5)
    point_cloud = o3d.geometry.PointCloud()
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=360, height=360)

    vis.get_render_option().background_color= np.asarray([0,0,0])   
    vis.get_render_option().show_coordinate_frame = True
    #vis.get_render_option().line_width = 1
    vis.get_render_option().light_on = True
    vis.reset_view_point(reset_bounding_box=True)

    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(1.25)
    point_cloud = o3d.geometry.PointCloud()
    camera_gyro_mesh_vis = o3d.geometry.TriangleMesh.create_coordinate_frame(1.15)
    camera_origin_mesh_vis = o3d.geometry.TriangleMesh.create_coordinate_frame(1.15)

    vis.add_geometry(frame)
    vis.add_geometry(point_cloud)
    vis.add_geometry(camera_gyro_mesh_vis)
    vis.add_geometry(camera_origin_mesh_vis)

    ctr = vis.get_view_control()
    ctr.set_lookat([-0.5,0.4,1])

    ctr.set_zoom(0.1)
    return vis, frame, point_cloud, camera_gyro_mesh_vis, camera_origin_mesh_vis
#********************************************************************************************************************************
#********************************************************************************************************************************
def render_open3d(camera_poincloud,camera_gyro_mesh,camera_mesh,camera_gyro_mesh_vis,camera_origin_mesh_vis,point_cloud,frame,point_size,vis):
    try:
        ###############################################################################################
        point_cloud.points = o3d.utility.Vector3dVector(np.asarray(camera_poincloud.points))
        point_cloud.colors = o3d.utility.Vector3dVector(np.asarray(camera_poincloud.colors))
        point_cloud.normals = o3d.utility.Vector3dVector(np.asarray(camera_poincloud.normals))
        ###############################################################################################
        camera_gyro_mesh_vis.vertices = o3d.utility.Vector3dVector(np.asarray(camera_gyro_mesh.vertices))
        camera_origin_mesh_vis.vertices = o3d.utility.Vector3dVector(np.asarray(camera_mesh.vertices))
        camera_origin_mesh_vis.paint_uniform_color([1,1,1])

        camera_gyro_mesh_vis.scale(0.15,center=camera_origin_mesh_vis.get_center())
        camera_gyro_mesh_vis.paint_uniform_color([1,0,1])
        
        camera_origin_mesh_vis.scale(0.15,center=camera_origin_mesh_vis.get_center())
        frame.scale(0.15,center=camera_origin_mesh_vis.get_center())
        ###############################################################################################

        point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        vis.update_geometry(point_cloud)
        vis.update_geometry(camera_gyro_mesh_vis)
        vis.update_geometry(camera_origin_mesh_vis)
        vis.update_renderer()
        vis.poll_events()
        vis.get_render_option().point_size = point_size
    except AttributeError:
        print("No Pointcloud")

def build_opencv_frame(IMG_DATA, DICT_vals):

    if IMG_DATA is not None:
        try:

            title= f""

            frame = cv2.imdecode(np.frombuffer(IMG_DATA, dtype=np.uint8), cv2.IMREAD_ANYDEPTH)
            

            frame_w = DICT_vals["camera_stream"][0]
            frame_h = DICT_vals["camera_stream"][1]

            frame_size_2d = DICT_vals["camera_stream"][8]
            stream_2d = DICT_vals["camera_stream"][14]
            run_stream = DICT_vals["camera_stream"][13]
            
            #print(f" frame_shape: {frame.shape} {frame.shape[1]} {frame.shape[0]} / {frame_w} {frame_h}")


            if run_stream :
                if frame is None:
                    return False,False
                else:
                    #confirmar que los tamaños sean los correctos para poder pasar el buffer sin que falte informacion

                    if frame.shape[1] == frame_w and frame.shape[0] == frame_h:

                        depth_vis = rescaleFrame(cv2.applyColorMap(cv2.convertScaleAbs(copy.deepcopy(frame), alpha=0.1), cv2.COLORMAP_JET),frame_size_2d)

                        color_frame = cv2.applyColorMap(cv2.convertScaleAbs(copy.deepcopy(frame), alpha=0.1), cv2.COLORMAP_JET)
                        depth_frame = copy.deepcopy(frame)

                        #print(f"color from depth vis: {color_frame.shape} {color_frame.dtype} ")
                        title=f'(COLOR/DEPTH){frame_w}x{frame_h}'

                        if stream_2d:
                            #show stream
                            cv2.imshow(title, depth_vis)
                            cv2.waitKey(1)
                        else: 
                            cv2.destroyAllWindows()
                        return depth_frame, color_frame
            else: 
                cv2.destroyAllWindows()
        except (ValueError,AttributeError,KeyError,IndexError,cv2.error) as EEE:
            #print("Error al decodificar corregido: ",EEE)
            return False,False
        
        
    

def depth_filter(pointcloud,percetible_depth):
    z_values = np.asarray(pointcloud.points)[:,2] #Select all the Rows last Col(2) Z coord
    zInd = np.where(z_values < percetible_depth/1000)
    filtered_pointcloud = pointcloud.select_by_index(indices=zInd[0])
    return filtered_pointcloud

def build_open3d_pointcloud(CAMERA_LOCAL_vals,depth_frame,color_frame):

    global camera_open3d_intrinsic_parameters
    global vis 
    global frame
    global point_cloud
    global camera_gyro_mesh_vis
    global camera_origin_mesh_vis
    global RENDER_FLAG
    try:
        CAMERA_REMOTE_RECEIVED_DICT_vals, CAMERA_REMOTE_RECEIVED_vals = txt.txt_r("MEDIA\\dependencies\\others\\camera_received.txt", 1)
        camera_local_par = CAMERA_LOCAL_vals["camera_stream"]
        camera_remote_par = CAMERA_REMOTE_RECEIVED_DICT_vals["ORIGINAL_camera_parameters"]

        depth_image = np.asanyarray(depth_frame)
        color_image = np.asanyarray(color_frame)
        bgr2rgbImage = np.asarray(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB),dtype=np.uint8)

        myColorImg = o3d.geometry.Image(bgr2rgbImage)
        myDepthImg = o3d.geometry.Image(depth_image)


        camera_open3d_intrinsic_parameters.set_intrinsics(camera_remote_par[0] , camera_remote_par[1]  , camera_remote_par[2],  camera_remote_par[3] ,  camera_remote_par[4],  camera_remote_par[5])
        
        rgbdImg= o3d.geometry.RGBDImage.create_from_color_and_depth(myColorImg, 
                                                                    myDepthImg,convert_rgb_to_intensity =False) #Activate GrayScale 
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbdImg,camera_open3d_intrinsic_parameters,
                                                            project_valid_depth_only=True) #Better


        #UNIFORM DOWN
        if camera_local_par[11] >=1.0: 
            pcd  = pcd.uniform_down_sample(int(camera_local_par[11]))
        
        #VOXEL SIZE
        #if camera_local_par[9] >=0.0025: 
        pcd  = pcd.voxel_down_sample(camera_local_par[9])
        #FILTER POINTCLOUD WITH PERCEPTIBLE DEPTH (Z COORD)
        pcd = depth_filter(pcd,camera_local_par[6])
        #print(pcd)

        #ACCELERATION FRAME FROM RECEIVED CAMERA, INCLINATION CORRECTION FOR RECONSTRUCTION
        accel_frame_ms2 = np.asanyarray([camera_remote_par[9],camera_remote_par[10],camera_remote_par[11]])
        accel_direction_vector = accel_frame_ms2 / np.linalg.norm(accel_frame_ms2) #Obtain the radians and divide per max in the colum for normalize
        x, y, z = accel_direction_vector[0], accel_direction_vector[2], accel_direction_vector[1]
        pitch = np.arctan2(x, np.sqrt(y**2 + z**2))
        roll = np.arctan2(y, np.sqrt(x**2 + z**2))
        pitch_degrees = np.degrees(pitch)
        roll_degrees = np.degrees(roll)
        rotx = rot.rot_x(alpha=np.deg2rad(roll_degrees))
        rotz = rot.rot_z(beta=np.deg2rad(pitch_degrees))
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        pcd.rotate(rotx)
        pcd.rotate(rotz)

        #CAMERA LENS POSE (POSE)
        mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(1.15)
        CameraMesh = copy.deepcopy(mesh)
        CameraMesh.transform([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        CameraMesh.translate((0, 0, 0))#(cameraResolution[0]/2)/1000
        #CameraMesh.scale(0.5,center=CameraMesh.get_center())

        #CAMERA GIROSCOPE PITCH AND ROLL
        CameraGyroscopeMesh = copy.deepcopy(mesh)
        CameraGyroscopeMesh.transform([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        CameraGyroscopeMesh.translate((0, 0, 0))#(cameraResolution[0]/2)/1000
        CameraGyroscopeMesh.rotate(rotx)
        CameraGyroscopeMesh.rotate(rotz)
        #CameraGyroscopeMesh.scale(0.5,center=CameraMesh.get_center())
        CameraGyroscopeMesh.paint_uniform_color([1,0,1])

        if camera_local_par[16]:
            if not RENDER_FLAG:
                #BUILD WINDOWS 3D TO RENDER
                vis, frame, point_cloud, camera_gyro_mesh_vis, camera_origin_mesh_vis = build_render_open3d()
                RENDER_FLAG = True

            if RENDER_FLAG:
                render_open3d(pcd,CameraGyroscopeMesh,CameraMesh,
                            camera_gyro_mesh_vis,camera_origin_mesh_vis,point_cloud,frame,
                            camera_local_par[10],vis)
        else:
            RENDER_FLAG = False
            vis, frame, point_cloud, camera_gyro_mesh_vis, camera_origin_mesh_vis = None,None,None,None,None
        return pcd
    except:
        pass



