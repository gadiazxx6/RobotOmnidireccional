import socket
import txt
import time
import reset_par
import subprocess

reset_par.reset_par()

temp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
temp_sock.connect(("8.8.8.8", 80))
HOST = temp_sock.getsockname()[0]
temp_sock.close()
PORT = 1236        # Puerto no privilegiado
STRING_client = ""
server_flag = False

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:

    s.bind((HOST, PORT))
    s.listen(1)

    while True:
        print(f"[Servidor PARAMETROS] Esperando conexión en {HOST}:{PORT}...")
        conn, addr = s.accept()

        if not server_flag:
            camera_server_process  = None
            camera_server_process = subprocess.Popen(["python", "CAMERA_server.py"])  # Ejecutar el script
            server_flag = True
            

        with conn:

            print(f"[Servidor PARAMETROS] Conectado con {addr}")
            
            try:
                while True:

                    #===================================================================================================        
                    #===================================================================================================
                    #GET INFO FROM CLIENT
                    parameters_data = conn.recv(2048).decode()
                    if not parameters_data:
                        print("[Servidor] Cliente desconectado, NO DATA")
                        break
                    else:

                        parameters_list = parameters_data.split('+')

                        txt.txt_w("MEDIA\\dependencies\\others\\server_data.txt",1,content_to_replace=parameters_list[0])#CAMERA
                        txt.txt_w("MEDIA\\dependencies\\others\\server_data.txt",4,content_to_replace=parameters_list[1])#MANUAL
                        txt.txt_w("MEDIA\\dependencies\\others\\server_data.txt",5,content_to_replace=parameters_list[2])#pcd configs
                        txt.txt_w("MEDIA\\dependencies\\others\\server_data.txt",6,content_to_replace=parameters_list[3])#automatic cmd
                        txt.txt_w("MEDIA\\dependencies\\others\\pcd_coords.txt",1,content_to_replace=parameters_list[4])#pcd coords
                        #===================================================================================================        
                        #===================================================================================================
                        #SEND LOCAL SERVER TO REMOTE CLIENT
                        try:
                            DICT_client, STRING_client = txt.txt_r("MEDIA\\dependencies\\others\\server_data.txt", 2)
                            #print(f"[Servidor] Recibido del cliente: {parameters_list}")
                            conn.send(f"{STRING_client}".encode('utf-8'))
                        except (IndexError,KeyError,TypeError):
                            continue

            except Exception as E:
                print("Error de comunicación: ",E)           
                if camera_server_process !=None:
                    camera_server_process.terminate()   
                    server_flag = False     
                reset_par.reset_par()
                conn.close()

