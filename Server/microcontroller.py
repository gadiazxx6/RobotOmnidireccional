import serial
import time
import txt
import serial.tools.list_ports

def print_com_info(port):
        print("******************************************************")
        print(f"Ports: {port.device}, Descripción: {port.description}")
        print(f"name': {port.name}")
        print(f"Product': {port.product}")
        print(f"Apply usb info': {port.apply_usb_info()}")
        print(f"Serial Number': {port.serial_number}")
        print(f"USB descrption': {port.usb_description()}")
        print(f"USB Info': {port.usb_info()}")
        print(f"PID : {port.pid}")
        print(f"vid : {port.vid}")
        print(f"ID del dispositivo: {port.hwid}")
        print(f"Interface: {port.interface}")
        print(f"Location: {port.location}")
        print(f"Manufacter: {port.manufacturer}")
        print("******************************************************")
    #********************************************************************************************************************************
def get_valid_roboclaw_ports():
    available_ports = serial.tools.list_ports.comports()
    valid_arduino = {"arduino":""}
    for port in available_ports:    
        #VERIFY  A ROBOCLAW AND VERSION
        if str(port.manufacturer) == 'Arduino LLC (www.arduino.cc)':
            #print_com_info(port=port)
            print("ARDUINO FOUND!")
            valid_arduino["arduino"] = str(port.device)
            print("******************************************************")        
            print(valid_arduino)
    return valid_arduino

valid_arduino_COM_port  = get_valid_roboclaw_ports()
ser = serial.Serial(valid_arduino_COM_port["arduino"], 9600, timeout=0.1) 
time.sleep(2)  # Esperar a que la conexión se estabilice


def read_server_info():
    try:
        SERVER_DICT_PAR , SERVER_STR_PAR = txt.txt_r("MEDIA\\dependencies\\others\\server_data.txt", 2)
        MANUAL_DICT_PAR, MANUAL_STR_PAR = txt.txt_r("MEDIA\\dependencies\\others\\server_data.txt", 4)
        AUTOMATIC_DICT_CONF_PAR, AUTOMATIC_STR_CONF_PAR = txt.txt_r("MEDIA\\dependencies\\others\\server_data.txt", 5)
        micro_par_execution  = 0
        time.sleep(0.1)  

        try:
            MICROC_EXEC_DICT_client, MICROC_EXEC_STRING_client = txt.txt_r("MEDIA\\dependencies\\others\\micro_c.txt", 2)
            micro_write_par = MICROC_EXEC_DICT_client["microcontroller_write"]
            micro_par_execution = micro_write_par[0] #read the final execution flag has been completed 
        except (TypeError,KeyError,IndexError) as AAA:
            #pass
            print( "ERROR: ARDUINO",AAA)   
        time.sleep(0.1)  

        serv_par = SERVER_DICT_PAR["ORIGINAL_camera_parameters"]
        man_par = MANUAL_DICT_PAR["manual"]
        aut_par = AUTOMATIC_DICT_CONF_PAR["pcd_coords_configs"]
        #COMMADS : MODE, TOWER, HEAD, TWEEZERS, DISTANCE, ROLL, PITCH, INIT TOWER DIST, FIN TOWER DIST, INIT TWEEZER POS, FIN TWEEZER POS 
        concat_data = f"{man_par[5]},{man_par[2]},{man_par[3]},{man_par[4]},{round(serv_par[12],4)},{round(serv_par[13],3)},{round(serv_par[14],3)},{aut_par[2]},{aut_par[3]},{aut_par[4]},{aut_par[5]},{micro_par_execution}\n"
        print(concat_data)
        return concat_data
    except (TypeError,KeyError,IndexError) as AAA:
        return 0
while True:

    resp = read_server_info()
    if resp != 0:
        ser.write(resp.encode())  # Enviar datos

    time.sleep(0.1)  # Espera antes del siguiente envío (ajustable)

    while ser.in_waiting:  # Si hay datos en el buffer
        response = ser.readline().decode().strip()  # Leer respuesta
        txt.txt_w("MEDIA\\dependencies\\others\\micro_c.txt",1,f"microcontroller_read:{response}")
        print(f"Respuesta de Arduino: {response}")

    #time.sleep(0.1)  # Espera antes del siguiente envío (ajustable)
