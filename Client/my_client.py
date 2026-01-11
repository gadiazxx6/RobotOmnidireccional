import socket
import txt
import reset_client_par

#reset_client_par.reset_par()

SERVER_DICT_vals, SERVER_vals = txt.txt_r("MEDIA\\dependencies\\others\\client_data.txt", 2)

SERVER_IP = SERVER_DICT_vals['communication'][0]
PARAM_PORT = SERVER_DICT_vals['communication'][2]

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:

    s.connect((SERVER_IP, PARAM_PORT))
    print("[Cliente] Conectado al servidor.")

    while True:

        try:
            #send camera parameters
            CAMERA_DICT_vals, CAMERA_vals = txt.txt_r("MEDIA\\dependencies\\others\\client_data.txt", 1)
            MANUAL_DICT_vals, MANUAL_vals = txt.txt_r("MEDIA\\dependencies\\others\\client_data.txt", 3)
            pcd_coords_configs_DICT_vals, pcd_coords_configs_vals = txt.txt_r("MEDIA\\dependencies\\others\\client_data.txt", 4)
            automatic_cmd_DICT_vals, automatic_cmd_vals = txt.txt_r("MEDIA\\dependencies\\others\\client_data.txt", 5)
            ROBOT_coords_DICT_vals, ROBOT_coords_vals = txt.txt_r("MEDIA\\dependencies\\others\\robot_coords.txt", 2)

            if CAMERA_vals is not None and MANUAL_vals is not None:

                TO_SEND = CAMERA_vals+'+'+MANUAL_vals+'+'+pcd_coords_configs_vals+'+'+automatic_cmd_vals+'+'+ROBOT_coords_vals
                # Enviar INFO
                s.send(TO_SEND.encode('utf-8'))

        except (IndexError,KeyError):
            pass

        data = s.recv(2048).decode('utf-8')
        #print(f"[Cliente] Respuesta del servidor: {data}")

        if not data:
            print("[Cliente] Servidor desconectado.")
            break
        
            
        txt.txt_w("MEDIA\\dependencies\\others\\camera_received.txt", 1,data)

