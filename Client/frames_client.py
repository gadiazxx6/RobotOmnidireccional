import socket
import time
import txt
import numpy as np
from decodificator import *
from make_SLAM import * 
import sys

def config_client_tcp_ip():

    SERVER_DICT_vals, SERVER_vals = txt.txt_r("MEDIA\\dependencies\\others\\client_data.txt", 2)

    SERVER_IP = SERVER_DICT_vals['communication'][0]
    COLOR_PORT = SERVER_DICT_vals['communication'][1]

    buff_size = int(2 * 1024 * 1024) #10MB
    ######################################################################################################
    CAMERA_client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    CAMERA_client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, buff_size)  #  #10MB
    CAMERA_client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, buff_size)  #  #10MB
    ######################################################################################################  
    try:
        CAMERA_client_socket.connect((SERVER_IP, COLOR_PORT))
    except ConnectionRefusedError:
        print("No se ha podido conectar con el servidor, cerrando....")
        time.sleep(1)
        CAMERA_client_socket.close()
        sys.exit("cerrado!")



    return  CAMERA_client_socket


CAMERA_client_socket = config_client_tcp_ip()


def exec_client():
    #try:
        while True:
            #send camera parameters
            CAMERA_DICT_vals, CAMERA_vals = txt.txt_r("MEDIA\\dependencies\\others\\client_data.txt", 1)
            
            try:
                cam_rem_par = CAMERA_DICT_vals["camera_stream"]

                #depth_data = depth_socket.recv(16 * 256 * 256)
                img_data = CAMERA_client_socket.recv(int(2 * 1024 * 1024))
                depth_frame,color_frame = build_opencv_frame(img_data,CAMERA_DICT_vals)
                if depth_frame is not False and color_frame is not False:
                    point_cloud_frame = build_open3d_pointcloud(CAMERA_DICT_vals,depth_frame,color_frame)
                    slam_reconstruction(CAMERA_DICT_vals,point_cloud_frame)
            except (TypeError,KeyError):
                pass
                
    #except Exception as e:
    #    print(f"Error {e}")
    #finally:
    #    CAMERA_client_socket.close()
        #depth_socket.close()
    #    sys.exit("cerrado!")

exec_client()

