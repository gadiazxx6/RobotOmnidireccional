import txt

def reset_par():

    #client data
    txt.txt_w("MEDIA\\dependencies\\others\\client_data.txt",1,content_to_replace="camera_stream:640,480,90,640,480,60,2500,150,0.3,0.032,2.5,4,10,0,1,0,0,kill,0,0,0,1")#CAMERA
    #txt.txt_w("MEDIA\\dependencies\\others\\client_data.txt",2,content_to_replace="communication:0,0,0,0")#COM
    txt.txt_w("MEDIA\\dependencies\\others\\client_data.txt",3,content_to_replace="manual:1200,0,0,0,0,1")#MANUAL
    txt.txt_w("MEDIA\\dependencies\\others\\client_data.txt",4,content_to_replace="pcd_coords_configs:0,0,0,250,0,0")#PCD COORDS CONFIGS
    txt.txt_w("MEDIA\\dependencies\\others\\client_data.txt",5,content_to_replace="automatic_cmd:0,0")#automatic commands


    txt.txt_w("MEDIA\\dependencies\\others\\pcd_info.txt",1,content_to_replace="num_points:0")
    txt.txt_w("MEDIA\\dependencies\\others\\pcd_info.txt",3,content_to_replace="index:0")  

    txt.txt_w("MEDIA\\dependencies\\others\\robot_coords.txt",2,content_to_replace="pcd_coords:[[0,0,0],[0.2,0.2,0]]")
    txt.txt_w("MEDIA\\dependencies\\others\\robot_coords.txt",3,content_to_replace="pcd_coords_configs:0,0,0,250,0,0")
    txt.txt_w("MEDIA\\dependencies\\others\\robot_coords.txt",4,content_to_replace="automatic_cmd:0,0")
    txt.txt_w("MEDIA\\dependencies\\others\\robot_coords.txt",5,content_to_replace="save_to:MEDIA/saved/reconstructed_pointcloud")
