import socket
import txt
import time
import cv2
import robot_camera_cv2_rs_v1

#================================================================================================================
def config_server_tcp_ip(CAMERA_PORT= 1235):
    #Create temporal socket to get IP Host 
    temp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    temp_sock.connect(("8.8.8.8", 80))
    HOST = temp_sock.getsockname()[0]
    temp_sock.close()
    buff_size = int(2 * 1024 * 1024) #2MB

    ######################################################################################################
    CAMERA_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    CAMERA_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, buff_size)  #  #10MB
    CAMERA_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, buff_size)  #  #10MB
    ######################################################################################################  

    # BINDING WITH HOST AND PORT and LISTEN INCOMING connection
    CAMERA_socket.bind((HOST,CAMERA_PORT))
    CAMERA_socket.listen(1)
    print(f"[Servidor CAMARA] Esperando conexión... {HOST, CAMERA_PORT}")

    return  CAMERA_socket

    #color and depth buffer to send

CAMERA_socket = config_server_tcp_ip()
#================================================================================================================

intel_d435i = None
#================================================================================================================

while True:
    """ ===================================================\n 
        CAMERA RUN COMMUNICATION \n
        ==================================================="""

    # ACCEPT CLIENT AND GET connection AND ADDRESS
    camera_connection, color_addr = CAMERA_socket.accept()
    #with conn and conn2:
    print(f'Conectado por {color_addr} para Color')
    #=======================================================================================================
    #CAMERA OBJECT
    intel_d435i = robot_camera_cv2_rs_v1.IntelRealsenseD435i(intel_realsense_name="D435i_NO1")
    #=======================================================================================================
    try:
        while True:
            
            #===========================================================================================================
            #RUN STREAM
            intel_d435i.camera_run()
            intel_d435i.write_camera_parameters()
            #===================================================================================================
            #SEND LOCAL SERVER TO REMOTE CLIENT

            try:
                #combined_frames_buffer = intel_d435i.concatenate_original_imgs()
                color_frame_buffer, depth_frame_buffer = intel_d435i.get_original_color_and_depth_buffer()
                camera_connection.send(depth_frame_buffer.tobytes())
            except Exception as e:
                print("Error en buffer: ",e )

            #========================================================================================================
    #================================================================================================================
    except Exception as e:
        print(f"Error en la conexión {e}")
    finally:
        camera_connection.close()
        cv2.destroyAllWindows()
    #================================================================================================================