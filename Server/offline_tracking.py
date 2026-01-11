# README: Before using this example the motor/controller combination must be
# AUTOTUNED and the settings saved to the Roboclaw using IonMotion SOFTWARE.
# The control required the inverse and direct cinematic matrix to control
# The Control required the trajectory generation based on anyone teory
# CONTROL PROGRAMMED BY: Gabriel Alejandro Diaz Fierro
import time
from roboclaw_3 import Roboclaw
import time
import numpy as np
import txt
import re
import ast

EXEC_THREAD = False 
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#GENERATE SOLUTION FOR 5° (FIFTH )ORDER POLYNOMIAL (GAUSSIAN VELOCITY) WITH SMOOTH ACCELERATION AND DECELERATION
def getCoef(t,i_v,f_v,i_t,f_t):
    Ax = np.array([[i_t**5,i_t**4,i_t**3,i_t**2,i_t,1],
                    [f_t**5,f_t**4,f_t**3,f_t**2,f_t,1],
                    [5*i_t**4,4*i_t**3,3*i_t**2,2*i_t,1,0],
                    [5*f_t**4,4*f_t**3,3*f_t**2,2*f_t,1,0], 
                    [20*i_t**3,12*i_t**2,6*i_t,2,0,0],
                    [20*f_t**3,12*f_t**2,6*f_t,2,0,0]])
    b = np.array([[i_v],[  f_v],[0],[0],[0],[0]]) #s_(0), s(f), s_d(0),s_d(f),s_dd(0),s_dd(f)
    x = None
    try:
        x = np.linalg.solve(Ax , b ) #A,B,C,D,E,F Coeficients
    except np.linalg.LinAlgError:
        #print("Error: Matriz singular, Resolviendo por minimos cuadrados.")
        x, residuals, rank, s = np.linalg.lstsq(Ax, b, rcond=None) #A,B,C,D,E,F Coeficients
    s = np.polyval(x,t) #SIGNAL 
    sd = np.polyval(np.diag(np.diag([5,4,3,2,1])*x[0:5]),t)#SIGNAL DERIVATIVE
    sdd = np.polyval(np.diag(np.diag([20,12,6,2])*x[0:4]),t)#2do SIGNAL DERIVATIVE
    return s[0],sd,sdd
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
def getCoefi(t,i_v,f_v,i_t,f_t):
    a3 = (2*(i_v-f_v))/((f_t-i_t)**3)
    a2 = (-3*(i_v-f_v))/((f_t-i_t)**2)
    a1 = 0
    a0 = i_v
    s = a3* t**3+ a2 * t**2+ a1*t + a0
    ss = 3*a3*t**2+ 2*a2*t+ a1 
    #print(s,ss)
    return s, ss,0
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
def scale( inp,  in_min,  in_max,  out_min,  out_max):
    return int(((inp - in_min) * ((out_max - out_min) / (in_max - in_min))) + out_min)
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
def estimate_time(x_ini, y_ini, x_fin, y_fin, i_angle, f_pos_angle, time_per_meter, time_per_90_degrees):
    x_dist = abs(x_fin - x_ini)
    y_dist = abs(y_fin - y_ini)
    angle_dist = abs(f_pos_angle - i_angle)
    x_time = x_dist * time_per_meter
    y_time = y_dist * time_per_meter
    angular_time = (angle_dist / 90) * time_per_90_degrees
    print("MULTIPLE CALCULATED TIMES: ",x_time, y_time, angular_time)
    return max(x_time, y_time, angular_time)
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
def get_coord_list(raw_string):
    data_str = raw_string
    match = re.search(r"\[\[.*\]\]", data_str)
    obj_list = []
    if match:
        str_list = match.group(0)  # obtain in middle of \[\[.*\]\] 
        obj_list = ast.literal_eval(str_list)  # convert to real list
    else:
        print("No se encontraron datos en el string.")
    return obj_list
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
def get_final_corrected_position(i_pos, f_pos):
    dist = f_pos - i_pos
    #4 mm per 20 cm 
    correction_gain = 0.004 / 0.2  # 0.02 (2%)
    if dist < 0:  # -----
        f_pos += abs(dist) * correction_gain*2.5
    else:  # +++++++++
        f_pos -= abs(dist) * correction_gain*0.4
    return f_pos 
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
def trajectory_tracking(x_i,x_d,y_i,y_d,phi_i,phi_d,rc,rc2,automatic_mode=True):
    global EXEC_THREAD
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% C O N S T A N T S %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    R  = 0.031;# % 3 cm
    L = 0.161;# % 11 cm
    max_motor_vel = 34.5 #34.5 rad/s or 330 RPM ( read datasheet)
    max_PWM = 12000 #rad/s or 330 RPM ( read datasheet)
    time_factor = 5
    time_ang_factor = 3
    address = 0x80
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if automatic_mode:
        #CORRECT DISTANCE CINEMATIC 
        x_dist = abs(x_d-x_i)
        y_dist = abs(y_d-y_i)
        phi_dist = abs(phi_d-phi_i)    
        print("TRACKING BEFORE INFO: ",x_i, x_d, y_i, y_d)
        if x_dist>=0.4:
            x_d = get_final_corrected_position(x_i,x_d)
        if y_dist>=0.4:
            y_d = get_final_corrected_position(y_i,y_d)
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
        calculated_time = estimate_time(x_i,y_i,x_d,y_d,phi_i,phi_d,time_factor,time_ang_factor)
        if calculated_time <=3 or max(x_dist,y_dist)<=0.4 or phi_dist<=90:
            calculated_time = 3
        print("TRACKING AFTER INFO: ",x_i, x_d, y_i, y_d, "TIME: ",calculated_time)
    else:
        calculated_time = 0.75
        set_flag(value=True)
    delta_t = 0 # Timer diff time variable
    curr_time = 0 # integration time variable
    phi_i = np.deg2rad(phi_i)
    phi_d = np.deg2rad(phi_d)
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    while curr_time<=calculated_time and EXEC_THREAD:
        ###############################################################
        #START TIME
        ###############################################################
        start_time = time.time()
        ###############################################################
        #CODE EXECUTION CALCULATION  (DELTA TIME)
        ###############################################################
        curr_time+=delta_t
        #print("current time: ", curr_time)
            
        if curr_time>=calculated_time:
            #time.sleep(0.15)
            return True
        #time.sleep(0.01)

        '''
        INPUT (TRAJECTORY GENERATION) POS, VEL , ACCEL
        '''
        x_req  ,   x_dot_req,  x_dot_dot_req = getCoef(t=curr_time,i_v=x_i,f_v=x_d,i_t=0,f_t=calculated_time)
        y_req  ,   y_dot_req,  y_dot_dot_req = getCoef(t=curr_time,i_v=y_i,f_v=y_d,i_t=0,f_t=calculated_time)
        phi_req,   phi_dot_req,  phi_dot_dot_req  = getCoef(t=curr_time,i_v=phi_i,f_v=phi_d,i_t=0,f_t=calculated_time)
        '''
        REQUIRED GLOBAL VELS 
        '''
        globals_vels_required = np.array([[x_dot_req],[y_dot_req],[phi_dot_req]])
        '''
        DIRECT JACOBIAN MATRIX
        '''
        J = np.array([
                [-np.sin(phi_req), np.cos(phi_req), L],
                [-np.cos((np.pi/6) + phi_req), -np.sin((np.pi/6) + phi_req), L],
                [np.cos(phi_req - (np.pi/6)), np.sin(phi_req - (np.pi/6)), L]
            ])
        '''
        INVERSE CINEMATIC MODEL / CALCULATE REQUIRED WHEELS SPEED
        '''
        MCI = ((1/R)*J)@globals_vels_required
        
        '''
        INVERSE CINCEMATIC MODEL WHEEL REQUIERED ANGULAR VELS
        '''
        q1_req = float(MCI[0][0])
        q2_req = float(MCI[1][0])
        q3_req = float(MCI[2][0])

        '''
        OUT READ
        '''
        try:
            '''
            SCALE CONTROL VARIABLE (CONTROL EFFORT)
            '''
            SC1 = scale(q1_req,0,max_motor_vel,0,max_PWM)
            SC2 = scale(q2_req, 0,max_motor_vel,0,max_PWM)
            SC3 = scale(q3_req, 0,max_motor_vel,0,max_PWM)

            '''
            INJECT CONTROL AUTOMATIC CONTROL, ERROR CALCULATED FOR ROBOCLAWS (INTERNAL PID CONTROL)
            '''
            
            rc.SpeedM2(address,SC1) #q1
            rc2.SpeedM2(address,SC2) #q2
            rc2.SpeedM1(address,SC3) #q3
        except AttributeError:
            #print("no roboclaw!")
            pass

        ##############################################################
        #END TIME CALCULATION
        ###############################################################
        end_time = time.time()
        delta_t = end_time - start_time
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
def automatic_control(rc,rc2):
    try:

        PCD_COORDS_DICT_client, PCD_COORDS_STRING_client = txt.txt_r("MEDIA\\dependencies\\others\\server_data.txt", 7)
        PCD_ORIENTATION_DICT_client, PCD_ORIENTATION_STRING_client = txt.txt_r("MEDIA\\dependencies\\others\\server_data.txt", 5)

        pcd_coords_LIST = get_coord_list(raw_string=PCD_COORDS_STRING_client)
        pcd_orientation_par = PCD_ORIENTATION_DICT_client["pcd_coords_configs"]

        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        INITIAL_ORIENTATION_FLAG = False
        FINAL_ORIENTATION_FLAG = False

        #SUBRUTINE
        for COORD in range(len(pcd_coords_LIST)-1):

            #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            x_i = 0 #%Initial conditions x_0 (Robot initial position)
            y_i = 0 #%Initial conditions y_0 (Robot initial position)
            phi_i = np.deg2rad(0)  #%Initial conditions phi_0 (Robot initial orientation)
            #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            x_d =  0    #%Desired conditions x_d (Robot desired position)
            y_d =  0 #%Desired conditions y_d (Robot desired position)
            phi_d = np.deg2rad(0) #%Desired conditions phi_d (Robot desired orientation)

            #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            current_coord = pcd_coords_LIST[COORD]
            next_coord = pcd_coords_LIST[COORD+1]


            if COORD != 0 and COORD != len(pcd_coords_LIST)-2: #intermedios
                print("inter") 
                x_i = current_coord[0]
                x_d = next_coord[0]
                y_i = current_coord[1]
                y_d = next_coord[1]
                phi_i = 0
                phi_d = 0
                trajectory_tracking(x_i,x_d,y_i,y_d,phi_i,phi_d,rc,rc2)

            #first coord first orientation
            if COORD == 0 and not INITIAL_ORIENTATION_FLAG:
                #NO TRASLADAR, SOLO ROTAR HACIA CERO PARA HACER MOVIMIENTOS SUAVES desde una 
                #orientacion inicial
                print(f"===========================================================================================\n FIRST ---> ORIENTATION: {current_coord} ---> {next_coord}, ORIENTATION: {pcd_orientation_par[0]}")
                x_i = 0
                x_d = 0
                y_i = 0
                y_d = 0
                phi_i = pcd_orientation_par[0]
                phi_d = 0
                INITIAL_ORIENTATION_FLAG =  trajectory_tracking(x_i,x_d,y_i,y_d,phi_i,phi_d,rc,rc2)

                print(f"INIT ORIENTATION FLAG {INITIAL_ORIENTATION_FLAG}")
            #IF THE ROBOT ORIENTATION HAS BEEN COMPLETED
            if COORD == 0 and INITIAL_ORIENTATION_FLAG:
                # TRASLADAR, NO ROTAR YA HA SIDO APLICADO
                print(f"===========================================================================================\n FIRST ---> TRANSLATION: {current_coord} ---> {next_coord}, ORIENTATION: {pcd_orientation_par[0]}")
                x_i = current_coord[0]
                x_d = next_coord[0]
                y_i = current_coord[1]
                y_d = next_coord[1]
                phi_i = 0
                phi_d = 0
                trajectory_tracking(x_i,x_d,y_i,y_d,phi_i,phi_d,rc,rc2)
                print(f"COMPLETED INITIAL - ROTATION + TRASLATION")
                #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            #IF THE ROBOT ORIENTATION HAS NOT BEEN COMPLETED
            if COORD == len(pcd_coords_LIST)-2 and not FINAL_ORIENTATION_FLAG:
                print(f"FINAL -- TRASLATION: {pcd_coords_LIST[COORD]}, ORIENTATION: {pcd_orientation_par[1]}")
                x_i = current_coord[0]
                x_d = next_coord[0]
                y_i = current_coord[1]
                y_d = next_coord[1]
                phi_i = 0
                phi_d = 0
                FINAL_ORIENTATION_FLAG = trajectory_tracking(x_i,x_d,y_i,y_d,phi_i,phi_d,rc,rc2)
            #final coord orientation
            if COORD == len(pcd_coords_LIST)-2 and FINAL_ORIENTATION_FLAG:
                print(f"FINAL --- ROTATION: {pcd_coords_LIST[COORD]}, ORIENTATION: {pcd_orientation_par[1]}")
                x_i = 0
                x_d = 0
                y_i = 0
                y_d = 0
                phi_i = 0
                phi_d = pcd_orientation_par[1]
                trajectory_tracking(x_i,x_d,y_i,y_d,phi_i,phi_d,rc,rc2)
                print("FINAL!!!!")

            print(f"({COORD}) CURRENT COORD : {current_coord} ----> NEXT COORD {next_coord}")
            print(f"xi: {x_i} , yi: {y_i} , phii: {phi_i} , xd: {x_d} , yd: {y_d} , phid: {phi_d}\n===========================================================================================\n")

    except (IndexError, KeyError):
        pass
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
def automatic_coords_follow(rc,rc2):
    automatic_control(rc,rc2)        
def right_follow(rc,rc2,cm_to_right=0.05):
    trajectory_tracking(0,0,0,-cm_to_right,0,0,rc,rc2,False)
def left_follow(rc,rc2,cm_to_left=0.05):
    trajectory_tracking(0,0,0,cm_to_left,0,0,rc,rc2,False)
def set_flag(value):
    global EXEC_THREAD
    EXEC_THREAD = value
