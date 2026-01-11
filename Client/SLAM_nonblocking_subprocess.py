import sys
import pickle
import open3d  as o3d
import numpy as np
import slam as SL
import txt
import copy
import time
from tqdm import tqdm

file_path = sys.argv[1]
frames_to_recons = []

# Read object from file
with open(file_path, "rb") as file:
    object = pickle.load(file)

print(f"Received object {type(object[0])}{len(object)}")

for i in range(len(object)):
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(np.squeeze(np.asanyarray(object[i][0])))
    point_cloud.colors = o3d.utility.Vector3dVector(np.squeeze(np.asanyarray(object[i][1])))
    point_cloud.normals = o3d.utility.Vector3dVector(np.squeeze(np.asanyarray(object[i][2])))  
    print(f"UN-packing: {len(frames_to_recons)}")
    frames_to_recons.append(point_cloud)

CAMERA_DICT_vals, CAMERA_vals = txt.txt_r("MEDIA\\dependencies\\others\\client_data.txt", 1)
camera_local_par = CAMERA_DICT_vals["camera_stream"]

slam = SL.SLAM(camera_local_par[9], color_mode = True)
run_times = []
for i in range(0, len(frames_to_recons)):

    source = copy.deepcopy(frames_to_recons[i])
    source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.25, max_nn=25))
    source  = o3d.geometry.PointCloud.farthest_point_down_sample(source,int(0.3*np.asanyarray(source.points).shape[0]))
    cl, ind = source.remove_statistical_outlier(nb_neighbors=80, std_ratio=0.85)
    source = source.select_by_index(ind)

    start = time.time()
    slam. update(source)
    end = time.time()

    run_times.append(end - start)
    print(f'Step: {i}, FPS: {1 / (end - start):6f}')

slam.save_trajectory()
vis = o3d.visualization.Visualizer()#o3d.visualization.visualizer()
vis.create_window(window_name = "Reconstr 3D",width = 800, height=800,visible= True )
vis.get_render_option().background_color= np.asarray([1,1,1])   
print('Generating scene:')
pcd_combined = o3d.geometry.PointCloud()
vis.add_geometry(pcd_combined)

for i in tqdm(range(len(slam.keyframes))):
    pcd_combined += slam.keyframes[i].cloud.transform(slam.graph.nodes[i].pose)
    vis.update_geometry(pcd_combined)
    try:
        vis.add_geometry(slam.camera_marker[i])
        pass
    except IndexError:
        pass
    vis.get_render_option().show_coordinate_frame = True
    vis.get_render_option().light_on = False
    vis.update_geometry(slam.keyframes[i].cloud.transform(slam.graph.nodes[i].pose))
    vis.reset_view_point(reset_bounding_box=True)
    vis.poll_events()
    vis.update_renderer() 
    time.sleep(0.3)

pcd_combined.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.25, max_nn=25))
pcd_combined  = o3d.geometry.PointCloud.farthest_point_down_sample(pcd_combined,int(0.2*np.asanyarray(pcd_combined.points).shape[0]))#frames_to_recons[point_id].uniform_down_sample(int(3))#
o3d.io.write_point_cloud(f"MEDIA\\saved\\reconstructed_pointcloud\\pcdprobe.ply", pcd_combined,format="ply",write_ascii=False)
vis.run()

