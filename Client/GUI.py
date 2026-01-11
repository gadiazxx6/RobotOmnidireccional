import cv2
import tkinter as tk
from tkinter import PhotoImage
from tkinter import filedialog
from PIL import Image, ImageTk
import cv2
import os
import ctypes as ct
#import urllib.request
from urllib.error import URLError, HTTPError
import subprocess
import numpy as np
#import socket
#import multiprocessing
import os
#SCRIPTS
import txt
import threading
import keyboard
import time
##############################

class GUI(tk.Tk):
    def __init__(self,windowName = "PURPOSITY_ROBOT_CONTROL_GUI"):
        super().__init__()
        self.title(windowName)
        self.screen_width = int(self.winfo_screenwidth()*1)
        self.screen_height = int(self.winfo_screenheight()*1)
        self.faceplates_width_divisor =  self.screen_width * 0.00045
        self.faceplates_height_divisor = self.screen_height * 0.00027
        self.faceplates_x_divisor = 0.035
        self.faceplates_y_divisor = 0.25
        self.FRAME_STATE = 0
        self.txt_file_path = "MEDIA\\dependencies\\others\\client_data.txt"
        self.txt_file_path2 = "MEDIA\\dependencies\\others\\robot_coords.txt"

        self.sep = ','
        self.downtime = 0.25  
        self.timer = None
        self.timer_wait = 100
        self.robot_wheels_command = 0
        self.robot_tower_command = 0
        self.robot_head_command = 0
        self.robot_tweezer_efector_command = 0
        self.first_time = True
        #//////////////////////////////////////////////////////////////////////
        self.create_folders()
        #self.dark_title_bar(self)
        self.configure_window_aesthetics()

        #//////////////////////////////////////////////////////////////////////
        #VARIABLES 
        self.depth_resolution_option = ["256x144", "424x240", "480x270", "640x360", "640x480", "848x100","848x480", "1280x720"]
        self.color_resolution_option = ["320x180", "320x240", "424x240", "640x360", "640x480", "848x480", "960x540","1280x720","1920x1080"]
        self.depth_FPS_options = [6,15,30,60,90,100,300]
        self.color_FPS_options = [6,15,30,60]
        #-----------------------------------------------------------
        #---BOOL-VARIABLES  [0 - True MANUAL,1 - False AUTOMATIC]
        self.bool_variables = [tk.BooleanVar(value=True),tk.BooleanVar(value=False),tk.BooleanVar(value=False),tk.BooleanVar(value=False),tk.BooleanVar(value=False),tk.BooleanVar(value=False)]
        #---DOUBLE-VARIABLES
        self.double_variables = [tk.DoubleVar(),tk.DoubleVar(),tk.DoubleVar(),tk.DoubleVar(),tk.DoubleVar(),tk.DoubleVar(),tk.DoubleVar(),tk.DoubleVar(),tk.DoubleVar(),tk.DoubleVar(),tk.DoubleVar(),tk.DoubleVar()]
        #--STRING-VARIABLES
        self.string_variables = [tk.StringVar(),tk.StringVar(),tk.StringVar(),tk.StringVar()]
        #---INT-VARIABLES  #depth_fps, color_fps
        self.int_variables = [tk.IntVar(),tk.IntVar(),tk.IntVar()]
        #-----------------------------------------------------------
        #---------STATUS FRAME
        self.status_frame = tk.Frame(self, bg=self.hex_colors["light_gray"], bd=3, relief="solid")
        self.status_frame_width =  None  # Ancho del Frame
        self.status_frame_height = None  # Altura del Frame
        #status frame labels config
        self.status_labels, NAN2 , NAN3, NAN4 = self.create_frame_items(num_labels=6,selected_frame="status_frame") #NEED 6 ITEMS TYPE LABELS 
        self.manual_status = self.status_labels[0]
        self.automatic_status = self.status_labels[1]
        self.traking_status = self.status_labels[2]
        self.COMUNICATION_status = self.status_labels[3]
        self.CAMERA_status = self.status_labels[4]
        self.STATUS_TITLE = self.status_labels[5]
       
        #-----------------------------------------------------------
        #---------COMMUNICATION FRAME
        self.COM_frame = tk.Frame(self, bg=self.hex_colors["light_gray"], bd=3, relief="solid")
        self.COM_frame_width =  None  # Ancho del Frame
        self.COM_frame_height = None  # Altura del Frame

        self.com_labels, self.com_buttons , NAN_SLIDER , self.com_entries = self.create_frame_items(num_labels=4,num_buttons=2,selected_frame="com_frame",num_entry=[3,15]) #NEED 1 ITEMS TYPE LABELS, 4 BUTTONS, 0 SLIDERS 
       
        self.com_TITLE = self.com_labels[0]
        self.com_ip_label = self.com_labels[1]
        self.com_camera_stream_port_label = self.com_labels[2]
        self.com_parameters_port_label = self.com_labels[3]
        
        #comunications SAVE button config
        self.comunication_save_button = self.com_buttons[0]
        self.button_initial_config(button=self.comunication_save_button,initial_text="Save Conf.", initial_color="gray", init_command=self.toggle_save_ip_ports_button)

        #comunications CONNECT button config
        self.comunication_connect_button = self.com_buttons[1]
        self.button_initial_config(button=self.comunication_connect_button,initial_text="Connect TO", initial_color="gray", init_command=self.toggle_connect_ip_ports_button)
        self.connect_FLAG = False
        #IP comunication entry config
        self.ip_entry = self.com_entries[0]
        self.camera_port_entry = self.com_entries[1]
        self.parameters_port_entry = self.com_entries[2]

        #-----------------------------------------------------------
        #---------CAMERA FRAME
        self.CAMERA_frame = tk.Frame(self, bg=self.hex_colors["light_gray"], bd=3, relief="solid")
        self.CAMERA_frame_width =  self.CAMERA_frame.winfo_width()  # Ancho del Frame
        self.CAMERA_frame_height = self.CAMERA_frame.winfo_height()  # Altura del Frame

        self.camera_labels, self.camera_buttons , self.camera_sliders , self.camera_entries = self.create_frame_items(num_labels=7,num_buttons=8,
            num_sliders=8 ,selected_frame="camera_frame",num_entry=[1,19])
        
        #CAMERA labels
        self.camera_stream_label = self.camera_labels[0]
        self.saved_pointcloud_label = self.camera_labels[6]

        self.fps_label = self.camera_labels[1]
        self.resolution_label = self.camera_labels[2]
        self.depth_label = self.camera_labels[3]
        self.color_label = self.camera_labels[4]
        self.gui_3D_interactive_label = self.camera_labels[5]

        #CAMERA ENTRY
        self.gui_3D_interactive_entry = self.camera_entries[0]
        self.gui_3D_interactive_entry.bind("<KeyPress>", self.on_key_gui_3d)
        self.gui_key = 's'
        #CAMERA buttons configs
        self.MAKE_RECONSTRUCTION_button = self.camera_buttons[7]
        self.MAKE_RECONSTRUCTION_button_FLAG = False
        self.button_initial_config(button=self.MAKE_RECONSTRUCTION_button,initial_text="Make SLAM", initial_color="gray", init_command=self.toggle_MAKE_RECONSTRUCTION_button)
    
        self.RESET_SAVED_FRAMES_button = self.camera_buttons[6]
        self.RESET_SAVED_FRAMES_FLAG = True
        self.button_initial_config(button=self.RESET_SAVED_FRAMES_button,initial_text="Clear FRAMES", initial_color="white", init_command=self.toggle_clear_frames_button)
    

        self.camera_pause_button = self.camera_buttons[5]
        self.camera_pause_FLAG = False
        self.button_initial_config(button=self.camera_pause_button,initial_text="RUN", initial_color="white", init_command=self.toggle_camera_pause_button)
    
        self.decimate_val_button = self.camera_buttons[4]
        self.button_initial_config(button=self.decimate_val_button,initial_text="Dec: ", initial_color="white", init_command=self.toggle_decimation_button)
        self.decimate_value = 0


        self.RUN_camera_button = self.camera_buttons[0]
        self.button_initial_config(button=self.RUN_camera_button,initial_text="  RUN CAMERA  ", initial_color="gray", init_command=self.toggle_run_camera_button)
        self.run_camera_FLAG = False

        self.visualizer_2D_button = self.camera_buttons[1]
        self.button_initial_config(button=self.visualizer_2D_button,initial_text="2D Rem. Win", initial_color="white", init_command=self.toggle_2D_visualizer_button)
        self.stream_2d_FLAG = True

        self.visualizer_3D_button = self.camera_buttons[2]
        self.button_initial_config(button=self.visualizer_3D_button,initial_text="3D Rem. Win", initial_color="gray", init_command=self.toggle_3D_visualizer_button)
        self.stream_3d_interactive = False

        self.visualizer_3D_local_button = self.camera_buttons[3]
        self.button_initial_config(button=self.visualizer_3D_local_button,initial_text="3D Loc. Win ", initial_color="gray", init_command=self.toggle_3D_visualizer_SLAM_button)
        self.stream_3d_static = False

        #CAMERA sliders
        self.camera_perceptible_depth_slider = self.camera_sliders[0]
        self.slider_initial_config(slider=self.camera_perceptible_depth_slider,variable_init=self.double_variables[0],
            from_init=0.0,to_init=3000.0,initial_val=2500.0,resolution_init=25.0,label_init="Depth(mm)",
            length_init=80,foreground_init=self.hex_colors["black"],bg_init=self.hex_colors["gray"],width_init=8,font_init=self.fonts[8],
            function_bind=self.get_sliders_values)

        self.camera_laser_power_slider = self.camera_sliders[1]
        self.slider_initial_config(slider=self.camera_laser_power_slider,variable_init=self.double_variables[1],
            from_init=0.0,to_init=250.0,initial_val=150.0,resolution_init=1.0,label_init=" Laser PWR",
            length_init=80,foreground_init=self.hex_colors["black"],bg_init=self.hex_colors["gray"],width_init=8,font_init=self.fonts[8],
            function_bind=self.get_sliders_values)

        self.visualizer_2D_size_slider = self.camera_sliders[2]
        self.slider_initial_config(slider=self.visualizer_2D_size_slider,variable_init=self.double_variables[2],
            from_init=0.2,to_init=1.0,initial_val=0.3,resolution_init=0.05,label_init=" Win. Size",
            length_init=80,foreground_init=self.hex_colors["black"],bg_init=self.hex_colors["gray"],width_init=8,font_init=self.fonts[8],
            function_bind=self.get_sliders_values)

        self.SLAM_3D_voxel_size_slider = self.camera_sliders[3]
        self.slider_initial_config(slider=self.SLAM_3D_voxel_size_slider,variable_init=self.double_variables[3],
            from_init=0.01,to_init=0.05,initial_val=0.032,resolution_init=0.001,label_init="Voxel Size",
            length_init=80,foreground_init=self.hex_colors["black"],bg_init=self.hex_colors["gray"],width_init=8,font_init=self.fonts[8],
            function_bind=self.get_sliders_values)
        
        self.SLAM_3D_point_size_slider = self.camera_sliders[4]
        self.slider_initial_config(slider=self.SLAM_3D_point_size_slider,variable_init=self.double_variables[4],
            from_init=0.25,to_init=10,initial_val=2.5,resolution_init=0.1,label_init="Point Size",
            length_init=80,foreground_init=self.hex_colors["black"],bg_init=self.hex_colors["gray"],width_init=8,font_init=self.fonts[8],
            function_bind=self.get_sliders_values)
        
        self.SLAM_3D_uniform_down_slider = self.camera_sliders[5]
        self.slider_initial_config(slider=self.SLAM_3D_uniform_down_slider,variable_init=self.double_variables[5],
            from_init=0.0,to_init=100.0,initial_val=4,resolution_init=1.0,label_init="Unif. Down",
            length_init=80,foreground_init=self.hex_colors["black"],bg_init=self.hex_colors["gray"],width_init=8,font_init=self.fonts[8],
            function_bind=self.get_sliders_values)
        
        self.SLAM_3D_num_pcd = self.camera_sliders[6]
        self.slider_initial_config(slider=self.SLAM_3D_num_pcd,variable_init=self.double_variables[6],
            from_init=0.0,to_init=250.0,initial_val=10,resolution_init=1.0,label_init="N - FRAMES",
            length_init=80,foreground_init=self.hex_colors["black"],bg_init=self.hex_colors["gray"],width_init=8,font_init=self.fonts[8],
            function_bind=self.get_sliders_values)


        self.SLAM_time_sampling = self.camera_sliders[7]
        self.slider_initial_config(slider=self.SLAM_time_sampling,variable_init=self.double_variables[10],
            from_init=0.0,to_init=5000.0,initial_val=500,resolution_init=1.0,label_init="Sampling (ms)",
            length_init=95,foreground_init=self.hex_colors["black"],bg_init=self.hex_colors["gray"],width_init=8,font_init=self.fonts[8],
            function_bind=self.get_sliders_values)
        
        #CAMERA selectors

        #depth config -------------------------
        self.depth_width = 640
        self.depth_height = 480
        self.string_variables[0].set(self.depth_resolution_option[4])
        self.depth_selector = tk.OptionMenu(self.CAMERA_frame,self.string_variables[0],*self.depth_resolution_option,command=self.selected_depth_options)
        self.depth_selector.config(width=6,bg = self.hex_colors["gray"],foreground=self.hex_colors["black"])
        
        self.selected_depth_FPS_FLAG = False
        self.depth_FPS = 60
        self.depth_FPS_selector = None 
        #color config -------------------------
        self.color_width = 640
        self.color_height = 480
        self.string_variables[1].set(self.color_resolution_option[4])
        self.color_selector = tk.OptionMenu(self.CAMERA_frame,self.string_variables[1],*self.color_resolution_option,command=self.selected_color_options)
        self.color_selector.config(width=6,bg = self.hex_colors["gray"],foreground=self.hex_colors["black"])
        
        self.selected_color_FPS_FLAG = False
        self.color_FPS = 60
        self.color_FPS_selector = None 
        
        #self.create_depth_fps_selector()

        #-----------------------------------------------------------
        #---------MANUAL MODE FRAME (THE LAST TO UPDATE )
        self.MANUAL_frame = tk.Frame(self, bg=self.hex_colors["light_gray"], bd=3, relief="solid")

        self.manual_frame_width =  self.MANUAL_frame.winfo_width()  # Ancho del Frame
        self.manual_frame_height = self.MANUAL_frame.winfo_height()  # Altura del Frame

        self.manual_labels, NAN2 , self.manual_sliders , NAN4 = self.create_frame_items(num_labels=18,
            selected_frame="manual_frame",num_sliders=1)
        
        #MANUAL MODE ENTRIES
        self.manual_robot_vel_slider = self.manual_sliders[0]
        self.slider_initial_config(slider=self.manual_robot_vel_slider,variable_init=self.double_variables[7],
            from_init=0.0,to_init=12000.0,initial_val=1200.0,resolution_init=10.0,label_init=" Wheels Velocity ",
            length_init=180,foreground_init=self.hex_colors["black"],bg_init=self.hex_colors["gray"],width_init=13,font_init=self.fonts[12],
            function_bind=self.get_sliders_values) 
        #MANUAL MODE LABELS
        #---robot wheels labels
        self.manual_mode_label = self.manual_labels[0]
        #up arrow
        self.up_label = self.manual_labels[1]
        self.img_up_0 = Image.open("MEDIA\\dependencies\\arrows_images\\up_0.png")
        self.img_up_1 = Image.open("MEDIA\\dependencies\\arrows_images\\up_1.png")
        #down arrow
        self.down_label = self.manual_labels[2]
        self.img_down_0 = Image.open("MEDIA\\dependencies\\arrows_images\\down_0.png")
        self.img_down_1 = Image.open("MEDIA\\dependencies\\arrows_images\\down_1.png")
        #right arrow
        self.right_label = self.manual_labels[3]
        self.img_right_0 = Image.open("MEDIA\\dependencies\\arrows_images\\right_0.png")
        self.img_right_1 = Image.open("MEDIA\\dependencies\\arrows_images\\right_1.png")
        #left arrow
        self.left_label = self.manual_labels[4]
        self.img_left_0 = Image.open("MEDIA\\dependencies\\arrows_images\\left_0.png")
        self.img_left_1 = Image.open("MEDIA\\dependencies\\arrows_images\\left_1.png")
        #diagonal up right 
        self.diagonal_up_right_label = self.manual_labels[5]
        self.img_u_right_0 = Image.open("MEDIA\\dependencies\\arrows_images\\up_right_0.png")
        self.img_u_right_1 = Image.open("MEDIA\\dependencies\\arrows_images\\up_right_1.png")
        #diagonal up left right 
        self.diagonal_up_left_label = self.manual_labels[6]
        self.img_u_left_0 = Image.open("MEDIA\\dependencies\\arrows_images\\up_left_0.png")
        self.img_u_left_1 = Image.open("MEDIA\\dependencies\\arrows_images\\up_left_1.png")
        #diagonal down right 
        self.diagonal_down_right_label = self.manual_labels[7]
        self.img_d_right_0 = Image.open("MEDIA\\dependencies\\arrows_images\\down_right_0.png")
        self.img_d_right_1 = Image.open("MEDIA\\dependencies\\arrows_images\\down_right_1.png")
        #diagonal down left 
        self.diagonal_down_left_label = self.manual_labels[8]
        self.img_d_left_0 = Image.open("MEDIA\\dependencies\\arrows_images\\down_left_0.png")
        self.img_d_left_1 = Image.open("MEDIA\\dependencies\\arrows_images\\down_left_1.png")
        #center
        self.center_label = self.manual_labels[9]
        self.img_center_0 = Image.open("MEDIA\\dependencies\\arrows_images\\center_0.png")
        self.img_center_1 = Image.open("MEDIA\\dependencies\\arrows_images\\center_1.png") 

        #---robot TWIST labels
        self.twist_right_label = self.manual_labels[16]
        self.img_twist_right_0 = Image.open("MEDIA\\dependencies\\arrows_images\\h_right_0.png")
        self.img_twist_right_1 = Image.open("MEDIA\\dependencies\\arrows_images\\h_right_1.png") 

        self.twist_left_label = self.manual_labels[17]
        self.img_twist_left_0 = Image.open("MEDIA\\dependencies\\arrows_images\\h_left_0.png")
        self.img_twist_left_1 = Image.open("MEDIA\\dependencies\\arrows_images\\h_left_1.png")        
        #---robot tower labels
        self.tower_up_label = self.manual_labels[10]
        self.img_tower_up_0 = Image.open("MEDIA\\dependencies\\arrows_images\\tower_up_0.png")
        self.img_tower_up_1 = Image.open("MEDIA\\dependencies\\arrows_images\\tower_up_1.png") 

        self.tower_down_label = self.manual_labels[11]
        self.img_tower_down_0 = Image.open("MEDIA\\dependencies\\arrows_images\\tower_down_0.png")
        self.img_tower_down_1 = Image.open("MEDIA\\dependencies\\arrows_images\\tower_down_1.png") 
        #---robot head labels
        self.head_right_label = self.manual_labels[12]
        self.img_head_right_0 = Image.open("MEDIA\\dependencies\\arrows_images\\h_right_0.png")
        self.img_head_right_1 = Image.open("MEDIA\\dependencies\\arrows_images\\h_right_1.png") 

        self.head_left_label = self.manual_labels[13]
        self.img_head_left_0 = Image.open("MEDIA\\dependencies\\arrows_images\\h_left_0.png")
        self.img_head_left_1 = Image.open("MEDIA\\dependencies\\arrows_images\\h_left_1.png") 
        #---robot tweezers labels
        self.tweezer_label = self.manual_labels[14]
        self.img_tweezer_0 = Image.open("MEDIA\\dependencies\\arrows_images\\tweezer_0.png")
        self.img_tweezer_1 = Image.open("MEDIA\\dependencies\\arrows_images\\tweezer_1.png")
        self.img_tweezer_2 = Image.open("MEDIA\\dependencies\\arrows_images\\tweezer_2.png")

        #---robot status manual command 
        self.manual_CMD_label = self.manual_labels[15]
        #-----------------------------------------------------------
        #---------MANUAL MODE FRAME (THE LAST TO UPDATE )
        self.AUTOMATIC_frame = tk.Frame(self, bg=self.hex_colors["light_gray"], bd=3, relief="solid")

        self.AUTOMATIC_frame_width =  self.AUTOMATIC_frame.winfo_width()  # Ancho del Frame
        self.AUTOMATIC_frame_height = self.AUTOMATIC_frame.winfo_height()  # Altura del Frame

        self.AUTOMATIC_labels, self.AUTOMATIC_buttons , NAN3 , self.AUTOMATIC_entries = self.create_frame_items(num_labels=12,
            selected_frame="automatic_frame",num_entry=[5,19],num_buttons=7)
        
        self.default_reconstruction_path = "MEDIA/saved/reconstructed_pointcloud" 
        self.pointcloud_path = "NONE [EMPTY]"
        #AUTOMATIC labels
        self.AUTOMATIC_TITLE  = self.AUTOMATIC_labels[0]
        self.save_to_path_label = self.AUTOMATIC_labels[1]
        self.open_from_path_label = self.AUTOMATIC_labels[2]

        self.init_orientation_label = self.AUTOMATIC_labels[3]
        self.init_tower_elevation = self.AUTOMATIC_labels[4]
        self.init_efector_label = self.AUTOMATIC_labels[5]
        self.INIT_COORDINATE_label = self.AUTOMATIC_labels[6]

        self.final_orientation_label = self.AUTOMATIC_labels[7]
        self.final_tower_elevation = self.AUTOMATIC_labels[8]
        self.final_efector_label = self.AUTOMATIC_labels[9]
        self.FINAL_COORDINATE_label = self.AUTOMATIC_labels[10]
        #save button config
        self.save_to_button = self.AUTOMATIC_buttons[0]
        self.button_initial_config(button=self.save_to_button,initial_text="Save Reconst. to  ", initial_color="white", init_command=self.toggle_save_to_button)
        #open from  button config
        self.open_from__button = self.AUTOMATIC_buttons[1]
        self.button_initial_config(button=self.open_from__button,initial_text="Open Reconst. from", initial_color="white", init_command=self.toggle_open_from_button)
        # init_efector_conf_button  button config
        self.init_efector_conf_button = self.AUTOMATIC_buttons[2]
        self.init_efector_FLAG = False
        self.button_initial_config(button=self.init_efector_conf_button,initial_text="CLOSED", initial_color="gray", init_command=self.toggle_efector_init_button)
        # final_efector_conf_button  button config
        self.final_efector_conf_button = self.AUTOMATIC_buttons[3]
        self.final_efector_FLAG = False
        self.button_initial_config(button=self.final_efector_conf_button,initial_text="CLOSED", initial_color="gray", init_command=self.toggle_efector_final_button)
        # interactive_coordinate_selector_button  button config
        self.interactive_coordinate_selector_button = self.AUTOMATIC_buttons[4]
        self.interactive_coordinate_GUI_FLAG = False
        self.button_initial_config(button=self.interactive_coordinate_selector_button,initial_text="OPEN INTERACTIVE COORDINATE 3D GUI", initial_color="gray", init_command=self.toggle_selector_coordinate_GUI)
        # return_robot_button  button config
        self.return_robot_button = self.AUTOMATIC_buttons[5]
        self.return_ROBOT_FLAG = False
        self.button_initial_config(button=self.return_robot_button,initial_text="RETURN ROBOT TO ORIGIN", initial_color="gray", init_command=self.toggle_return_robot_button)
        # return_robot_button  button config
        self.start_robot_tracking_button = self.AUTOMATIC_buttons[6]
        self.start_ROBOT_FLAG = False
        self.button_initial_config(button=self.start_robot_tracking_button,initial_text="START ROBOT TRACKING", initial_color="gray", init_command=self.toggle_START_robot_TRACKING_button)

        #CAMERA ENTRY
        self.init_orientation_entry = self.AUTOMATIC_entries[0]
        self.final_orientation_entry = self.AUTOMATIC_entries[1]
        self.init_tower_entry = self.AUTOMATIC_entries[2]
        self.final_tower_entry = self.AUTOMATIC_entries[3]

        self.init_orientation_entry.insert(0, 0)  
        self.final_orientation_entry.insert(0, 0)  
        self.init_tower_entry.insert(0, 0)  
        self.final_tower_entry.insert(0, 250) 
 
        self.init_orientation_entry.bind("<KeyPress>", self.on_key_gui_3d)
        self.final_orientation_entry.bind("<KeyPress>", self.on_key_gui_3d)
        self.init_tower_entry.bind("<KeyPress>", self.on_key_gui_3d)
        self.final_tower_entry.bind("<KeyPress>", self.on_key_gui_3d)

        self.init_orientation_entry.bind("<KeyRelease>", self.on_key_gui_3d)
        self.final_orientation_entry.bind("<KeyRelease>", self.on_key_gui_3d)
        self.init_tower_entry.bind("<KeyRelease>", self.on_key_gui_3d)
        self.final_tower_entry.bind("<KeyRelease>", self.on_key_gui_3d)

        #-----------------------------------------------------------
        #---------MENU FRAME (THE LAST TO UPDATE )
        self.MENU_frame = tk.Frame(self, bg=self.hex_colors["light_gray"], bd=3, relief="solid")
        self.menu_frame_width =  self.MENU_frame.winfo_width()  # Ancho del Frame
        self.menu_frame_height = self.MENU_frame.winfo_height()  # Altura del Frame

        self.menu_labels, self.menu_buttons , NAN3, NAN4 = self.create_frame_items(num_labels=1,num_buttons=4,selected_frame="menu_frame") #NEED 1 ITEMS TYPE LABELS, 4 BUTTONS, 0 SLIDERS 
        self.menu_TITLE  = self.menu_labels[0]
        #comunication button config
        self.comunication_control_button = self.menu_buttons[0]
        self.button_initial_config(button=self.comunication_control_button,initial_text="COM Crtl", initial_color="gray", init_command=self.toggle_comunication_button)

        #camera button config
        self.camera_stream_control_button = self.menu_buttons[1]
        self.button_initial_config(button=self.camera_stream_control_button,initial_text="Camera & Stream Crtl", initial_color="gray", init_command=self.toggle_camera_button)
        #mode manual button config
        self.mode_manual_button = self.menu_buttons[2]
        self.button_initial_config(button=self.mode_manual_button,initial_text="Manual", initial_color="gray", init_command=self.toggle_mode_button)
        self.ROBOT_MODE_FLAG = True
        #mode Automatic button config
        self.mode_automatic_button = self.menu_buttons[3]
        self.button_initial_config(button=self.mode_automatic_button,initial_text="Automatic", initial_color="gray", init_command=self.toggle_mode_automatic_button)
        #-----------------------------------------------------------
        #-----------------------------------------------------------        
        #BUTTON (PRINCIPAL BUTTON)
        self.main_menu_button = tk.Button(self, text="B\nA\nC\nK", command=self.toggle_main_menu_button,bg=self.hex_colors["white"],font=self.fonts[11],bd=1, relief="solid")

        #//////////////////////////////////////////////////////////////////////
        #self.bind("<Configure>", self.resize_bg)
        self.resizable(False, False)
        #/////////////////////////////////////////////////////////////////////////////////////////////////////////////
        # Escuchar cualquier tecla y reiniciar el temporizador
        keyboard.on_press(self.reload_temp)
        

        
        keyboard.add_hotkey("up", lambda: self.detect_keyboard(key_comb="up"))
        keyboard.add_hotkey("up + right", lambda: self.detect_keyboard(key_comb="up + right"))
        keyboard.add_hotkey("up + left", lambda: self.detect_keyboard(key_comb="up + left"))
        keyboard.add_hotkey("down", lambda: self.detect_keyboard(key_comb="down"))
        keyboard.add_hotkey("down + right", lambda: self.detect_keyboard(key_comb="down + right"))
        keyboard.add_hotkey("down + left", lambda: self.detect_keyboard(key_comb="down + left"))
        keyboard.add_hotkey("right", lambda: self.detect_keyboard(key_comb="right"))
        keyboard.add_hotkey("left", lambda: self.detect_keyboard(key_comb="left"))
        keyboard.add_hotkey("ctrl + right", lambda: self.detect_keyboard(key_comb="ctrl + right"))
        keyboard.add_hotkey("ctrl + left", lambda: self.detect_keyboard(key_comb="ctrl + left"))    

        #/////////////////////////////////////////////////////////////////////////////////////////////////////////////
        keyboard.add_hotkey("shift + up", lambda: self.detect_keyboard(key_comb="shift + up"))
        keyboard.add_hotkey("shift + down", lambda: self.detect_keyboard(key_comb="shift + down"))
        keyboard.add_hotkey("shift + right", lambda: self.detect_keyboard(key_comb="shift + right"))
        keyboard.add_hotkey("shift + left", lambda: self.detect_keyboard(key_comb="shift + left"))
        keyboard.add_hotkey("shift + ctrl + left", lambda: self.detect_keyboard(key_comb="shift + ctrl + left"))
        keyboard.add_hotkey("shift + ctrl + right", lambda: self.detect_keyboard(key_comb="shift + ctrl + right"))
        


        # Iniciar el primer temporizador
        self.timer_function()
        self.reload_temp()
        self.verify_buttons_status()
        txt.txt_w(txt_file="MEDIA\\dependencies\\others\\slam_status.txt",selected_line=1,content_to_replace=f"slam_status:{0},{1},{0}")
        txt.txt_w(txt_file="MEDIA\\dependencies\\others\\slam_status.txt",selected_line=2,content_to_replace=f"pcd_buffer:0")

        self.selected_depth_options(selected_depth_option="640x480")
        depth_fps_option_temp = self.depth_FPS_options[-1]
        self.selected_depth_FPS_options(selected_depth_FPS_option = depth_fps_option_temp)

        self.selected_color_options(selected_color_option="640x480")
        color_fps_option_temp = self.depth_FPS_options[-2]
        self.selected_color_FPS_options(selected_color_FPS_option = color_fps_option_temp)

        self.toggle_run_camera_button()

        self.write_data()
        self.write_robot_cmd()

    def toggle_save_to_button(self):
        self.seleccionar_carpeta()
        self.write_robot_cmd()
        self.write_data()

    def toggle_open_from_button(self):
        self.seleccionar_archivo()
        self.write_robot_cmd()
        self.write_data()

    def toggle_selector_coordinate_GUI(self):
        self.interactive_coordinate_GUI_FLAG = not self.interactive_coordinate_GUI_FLAG
        if self.interactive_coordinate_GUI_FLAG:
            self.interactive_coordinate_selector_button.config(bg=self.hex_colors["white"],bd=2, relief="solid", height=1,font=self.fonts[11])
            self.write_robot_cmd()
            self.write_data()
            self.coord_GUI_3D_EDITING = subprocess.Popen(["python", "editing_pcd_vis.py"])
            self.coord_GUI_3D_STATIC = subprocess.Popen(["python", "static_pcd_vis.py"])
        else:
            self.write_robot_cmd()
            self.write_data()
            self.interactive_coordinate_selector_button.config(bg=self.hex_colors["gray"],bd=2, relief="solid", height=1,font=self.fonts[11])
            self.coord_GUI_3D_EDITING.terminate() 
            self.coord_GUI_3D_EDITING.kill()
            self.coord_GUI_3D_STATIC.terminate() 
            self.coord_GUI_3D_STATIC.kill()

    def toggle_START_robot_TRACKING_button(self):
        self.start_ROBOT_FLAG = not self.start_ROBOT_FLAG
        if self.start_ROBOT_FLAG:
            self.start_robot_tracking_button.config(bg=self.hex_colors["white"],bd=2, relief="solid", height=1,font=self.fonts[11])
        else:
            self.start_robot_tracking_button.config(bg=self.hex_colors["gray"],bd=2, relief="solid", height=1,font=self.fonts[11])
        self.write_robot_cmd()
        self.write_data()

    def toggle_efector_init_button(self):
        self.init_efector_FLAG = not self.init_efector_FLAG
        if self.init_efector_FLAG:
            self.init_efector_conf_button.config(text="OPEN",bg=self.hex_colors["white"],bd=2, relief="solid", height=1,font=self.fonts[9])
        else:
            self.init_efector_conf_button.config(text="CLOSED",bg=self.hex_colors["gray"],bd=2, relief="solid", height=1,font=self.fonts[9])
        self.write_robot_cmd()
        self.write_data()

    def toggle_efector_final_button(self):
        self.final_efector_FLAG = not self.final_efector_FLAG
        if self.final_efector_FLAG:
            self.final_efector_conf_button.config(text="OPEN",bg=self.hex_colors["white"],bd=2, relief="solid", height=1,font=self.fonts[9])
        else:
            self.final_efector_conf_button.config(text="CLOSED",bg=self.hex_colors["gray"],bd=2, relief="solid", height=1,font=self.fonts[9])
        self.write_robot_cmd()
        self.write_data()
    
    def toggle_return_robot_button(self):
        self.return_ROBOT_FLAG = not self.return_ROBOT_FLAG
        if self.return_ROBOT_FLAG:
            self.return_robot_button.config(bg=self.hex_colors["white"],bd=2, relief="solid", height=1,font=self.fonts[11])
        else:
            self.return_robot_button.config(bg=self.hex_colors["gray"],bd=2, relief="solid", height=1,font=self.fonts[11])
        self.write_robot_cmd()
        self.write_data()



    def seleccionar_carpeta(self):
        self.default_reconstruction_path = filedialog.askdirectory(initialdir=self.default_reconstruction_path)
        self.save_to_path_label.config(text=self.default_reconstruction_path)
        self.write_robot_cmd()
        self.write_data()

    def seleccionar_archivo(self):
        self.pointcloud_path = filedialog.askopenfilename(initialdir=self.pointcloud_path)
        self.open_from_path_label.config(text=self.pointcloud_path)
        self.write_robot_cmd()
        self.write_data()
                    
    def reload_temp(self,evento=None):
        if self.timer:
            self.timer.cancel()  # Cancela el temporizador anterior
        self.timer = threading.Timer(self.downtime, self.NO_PRESS)  # Reinicia el temporizador
        self.timer.start()


    def on_key_gui_3d(self,event):

        #erase after
        #print("Tecla presionada:", event.keysym)  
        if event.keysym == 's' or event.keysym == 'S' or event.keysym == 'p' or event.keysym == 'P' or event.keysym == 'R' or event.keysym == 'r' or event.keysym == 'D' or event.keysym == 'd' or event.keysym == 'z' or event.keysym == 'Z' or event.keysym == 'C' or event.keysym == 'c' or event.keysym == 'E' or event.keysym == 'e':
            self.gui_key = event.keysym
            self.gui_3D_interactive_entry.delete(0, tk.END)  
            self.gui_3D_interactive_entry.insert(0, self.gui_key) 
        else: 
            self.gui_key = 'kill'
            self.gui_3D_interactive_entry.delete(0, tk.END)
            self.gui_3D_interactive_entry.insert(0, "INVALID CMD!")  

        #write first
        self.write_data()
        self.write_robot_cmd()

    def NO_PRESS(self):
        #print("No se ha presionado ninguna tecla en los últimos", self.downtime, "segundos.")
        self.robot_wheels_command = 0
        self.robot_head_command = 0
        self.robot_tower_command = 0
        self.robot_tweezer_efector_command = 0

        self.manual_CMD_label.config(text="       CMD[ REPOSE ]:[   NONE   ]        ",width=len("       CMD[ REPOSE ]:[   NONE   ]        "),bg=self.hex_colors["gray"],bd=0, relief="solid", height=1,font=self.fonts[11])
        self.show_label_img(label_img=self.tweezer_label,img=self.img_tweezer_0)                        
        self.show_label_img(label_img=self.center_label,img=self.img_center_1)   
        self.show_label_img(label_img=self.up_label,img=self.img_up_0)
        self.show_label_img(label_img=self.down_label,img=self.img_down_0)
        self.show_label_img(label_img=self.right_label,img=self.img_right_0)            
        self.show_label_img(label_img=self.left_label,img=self.img_left_0)            
        self.show_label_img(label_img=self.diagonal_up_right_label,img=self.img_u_right_0)            
        self.show_label_img(label_img=self.diagonal_up_left_label,img=self.img_u_left_0)            
        self.show_label_img(label_img=self.diagonal_down_right_label,img=self.img_d_right_0)                        
        self.show_label_img(label_img=self.diagonal_down_left_label,img=self.img_d_left_0)    

        self.show_label_img(label_img=self.tower_up_label,img=self.img_tower_up_0)                        
        self.show_label_img(label_img=self.tower_down_label,img=self.img_tower_down_0)   

        self.show_label_img(label_img=self.head_left_label,img=self.img_head_left_0)  
        self.show_label_img(label_img=self.head_right_label,img=self.img_head_right_0)                        
                      
        self.show_label_img(label_img=self.twist_left_label,img=self.img_twist_left_0)  
        self.show_label_img(label_img=self.twist_right_label,img=self.img_twist_right_0)  

        self.write_data()

    def detect_keyboard(self, key_comb):
        if  self.bool_variables[0].get():
            #///////////////////////////////////////////////
            if key_comb == "shift + ctrl + left":
                self.robot_tweezer_efector_command = 1
                self.manual_CMD_label.config(text="CMD[ TWEEZER ]:[ OPEN ] ",width=len("CMD[           ]:[           ]"),bg=self.hex_colors["gray"],bd=0, relief="solid", height=1,font=self.fonts[11])
                self.tweezer_label.place(x=self.manual_frame_width*0.817, y=self.manual_frame_height*0.125,
                                    width=100, height=133) 
                self.show_label_img(label_img=self.tweezer_label,img=self.img_tweezer_2)                        
 
            if key_comb == "shift + ctrl + right":
                self.robot_tweezer_efector_command = 2 
                self.manual_CMD_label.config(text="CMD[ TWEEZER ]:[ CLOSE ] ",width=len("CMD[           ]:[           ] "),bg=self.hex_colors["gray"],bd=0, relief="solid", height=1,font=self.fonts[11])
                self.tweezer_label.place(x=self.manual_frame_width*0.817, y=self.manual_frame_height*0.125,
                                    width=100, height=133)  
                self.show_label_img(label_img=self.tweezer_label,img=self.img_tweezer_1)  

            if key_comb == "up":
                self.robot_wheels_command = 1
                self.manual_CMD_label.config(text="CMD[ WHEELS-(UP) ]:[ UP ] ",width=len("CMD[           ]:[           ]"),bg=self.hex_colors["gray"],bd=0, relief="solid", height=1,font=self.fonts[11])
                self.show_label_img(label_img=self.up_label,img=self.img_up_1)
                self.show_label_img(label_img=self.center_label,img=self.img_center_0)                        
            elif key_comb == "down":
                self.robot_wheels_command = 2
                self.manual_CMD_label.config(text="CMD[ WHEELS-(DOWN) ]:[ DOWN ] ",width=len("CMD[           ]:[           ]"),bg=self.hex_colors["gray"],bd=0, relief="solid", height=1,font=self.fonts[11])
                self.show_label_img(label_img=self.down_label,img=self.img_down_1)
                self.show_label_img(label_img=self.center_label,img=self.img_center_0)                        
            elif key_comb == "right":
                self.robot_wheels_command = 3
                self.manual_CMD_label.config(text="CMD[ WHEELS-(RIGHT) ]:[ RIGHT ] ",width=len("CMD[           ]:[           ]"),bg=self.hex_colors["gray"],bd=0, relief="solid", height=1,font=self.fonts[11])
                self.show_label_img(label_img=self.right_label,img=self.img_right_1)            
                self.show_label_img(label_img=self.center_label,img=self.img_center_0)                        
            elif key_comb == "left":
                self.robot_wheels_command = 4
                self.manual_CMD_label.config(text="CMD[ WHEELS-(LEFT) ]:[ LEFT ] ",width=len("CMD[           ]:[           ]"),bg=self.hex_colors["gray"],bd=0, relief="solid", height=1,font=self.fonts[11])
                self.show_label_img(label_img=self.left_label,img=self.img_left_1)            
                self.show_label_img(label_img=self.center_label,img=self.img_center_0)                                    
            elif key_comb == "up + right":
                self.robot_wheels_command = 5
                self.manual_CMD_label.config(text="CMD[ WHEELS-(UP+RIGHT) ]:[ UP+RIGHT ]",width=len("CMD[               ]:[               ]"),bg=self.hex_colors["gray"],bd=0, relief="solid", height=1,font=self.fonts[11])
                self.show_label_img(label_img=self.diagonal_up_right_label,img=self.img_u_right_1)            
                self.show_label_img(label_img=self.center_label,img=self.img_center_0)                        
            elif key_comb == "up + left":
                self.robot_wheels_command = 6
                self.manual_CMD_label.config(text="CMD[ WHEELS-(UP+LEFT) ]:[ UP+LEFT ] ",width=len("CMD[               ]:[               ] "),bg=self.hex_colors["gray"],bd=0, relief="solid", height=1,font=self.fonts[11])
                self.show_label_img(label_img=self.diagonal_up_left_label,img=self.img_u_left_1)            
                self.show_label_img(label_img=self.center_label,img=self.img_center_0)                        
            elif key_comb == "down + right":
                self.robot_wheels_command = 7
                self.manual_CMD_label.config(text="CMD[ WHEELS-(DOWN+RIGHT) ]:[ DOWN+RIGHT ]",width=len("CMD[                 ]:[                 ] "),bg=self.hex_colors["gray"],bd=0, relief="solid", height=1,font=self.fonts[11])
                self.show_label_img(label_img=self.diagonal_down_right_label,img=self.img_d_right_1) 
                self.show_label_img(label_img=self.center_label,img=self.img_center_0)                        
                       
            elif key_comb == "down + left":
                self.robot_wheels_command = 8
                self.manual_CMD_label.config(text="CMD[ WHEELS-(DOWN+LEFT) ]:[ DOWN+LEFT ]",width=len("CMD[                ]:[                ] "),bg=self.hex_colors["gray"],bd=0, relief="solid", height=1,font=self.fonts[11])
                self.show_label_img(label_img=self.diagonal_down_left_label,img=self.img_d_left_1)
                self.show_label_img(label_img=self.center_label,img=self.img_center_0)       
            
            elif key_comb == "ctrl + right":
                self.robot_wheels_command = 9
                self.manual_CMD_label.config(text="CMD[ WHEELS-(DIAG-RIGHT) ]:[ CTRL+RIGHT ]",width=len("CMD[                ]:[                ] "),bg=self.hex_colors["gray"],bd=0, relief="solid", height=1,font=self.fonts[11])
                self.show_label_img(label_img=self.twist_right_label,img=self.img_twist_right_1)
                self.show_label_img(label_img=self.center_label,img=self.img_center_0)   

            elif key_comb == "ctrl + left":
                self.robot_wheels_command = 10
                self.manual_CMD_label.config(text="CMD[ WHEELS-(DIAG-LEFT) ]:[ CTRL+LEFT ]",width=len("CMD[                ]:[                ] "),bg=self.hex_colors["gray"],bd=0, relief="solid", height=1,font=self.fonts[11])
                self.show_label_img(label_img=self.twist_left_label,img=self.img_twist_left_1)
                self.show_label_img(label_img=self.center_label,img=self.img_center_0)    
            #///////////////////////////////////////////////
            if key_comb == "shift + up":
                self.robot_tower_command = 1
                self.manual_CMD_label.config(text="CMD[ TOWER ]:[ UP ] ",width=len("CMD[           ]:[           ]"),bg=self.hex_colors["gray"],bd=0, relief="solid", height=1,font=self.fonts[11])
                self.show_label_img(label_img=self.tower_up_label,img=self.img_tower_up_1)                        
            if key_comb == "shift + down":
                self.robot_tower_command = 2
                self.manual_CMD_label.config(text="CMD[ TOWER ]:[ DOWN ] ",width=len("CMD[           ]:[           ]"),bg=self.hex_colors["gray"],bd=0, relief="solid", height=1,font=self.fonts[11])
                self.show_label_img(label_img=self.tower_down_label,img=self.img_tower_down_1)                        
            #///////////////////////////////////////////////
            if key_comb == "shift + left":
                self.robot_head_command = 1
                self.manual_CMD_label.config(text="CMD[ HEAD ]:[ LEFT ] ",width=len(" CMD[           ]:[           ]"),bg=self.hex_colors["gray"],bd=0, relief="solid", height=1,font=self.fonts[11])
                self.show_label_img(label_img=self.head_left_label,img=self.img_head_left_1)                        
  
            if key_comb == "shift + right":
                self.robot_head_command = 2  
                self.manual_CMD_label.config(text="CMD[ HEAD ]:[ RIGHT ] ",width=len("CMD[           ]:[           ]"),bg=self.hex_colors["gray"],bd=0, relief="solid", height=1,font=self.fonts[11])
                self.show_label_img(label_img=self.head_right_label,img=self.img_head_right_1)                        
                      
        self.write_data()

    def timer_function(self):
        #read every self.timer_wait ms

        self.after(self.timer_wait, self.timer_function)
        #print(f"wheels: {self.robot_wheels_command}, tower: {self.robot_tower_command}, head:{self.robot_head_command}, tweezer: {self.robot_tweezer_efector_command} ")

        #BUTTONS
        self.main_menu_button.place(x=self.screen_width*0.005 , y=self.screen_height*0.005,width=self.screen_width*0.025, height=self.screen_height*0.35)
        #FRAMES 
        self.update_status_frame_items() #ALWAYS REFRESHED
        if self.first_time:
            self.update_MENU_frame_items()
            self.update_COM_frame_items()
            self.update_CAMERA_frame_items()
            self.update_MANUAL_frame_items()    
            self.update_AUTOMATIC_frame_items() 
            self.first_time = False

        if self.FRAME_STATE == 0: self.update_MENU_frame_items()
        if self.FRAME_STATE == 1: self.update_COM_frame_items()
        if self.FRAME_STATE == 2: self.update_CAMERA_frame_items()
        if self.FRAME_STATE == 3: self.update_MANUAL_frame_items()
        if self.FRAME_STATE == 4: self.update_AUTOMATIC_frame_items() 

        self.gui_key = 'kill'
        #self.write_data()

    def show_label_img(self,label_img, img):

        img_tk = ImageTk.PhotoImage(img.resize((label_img.winfo_width(), label_img.winfo_height())))
        label_img.config(image = img_tk)
        label_img.image = img_tk 

    def button_initial_config(self,button, initial_text="       ", initial_color="gray",init_command = None):
        button.config(text=initial_text,width=len(initial_text),bg=self.hex_colors[initial_color],
                                bd=2, relief="solid", height=1,font=self.fonts[11],command=init_command)
    
    def slider_initial_config(self, slider ,variable_init,from_init = 0, to_init = 0, initial_val = 0, resolution_init = 0,label_init="     ",length_init = 20,foreground_init='white',bg_init="black",width_init=4, font_init=("Arial", 7,"bold"), function_bind=None):
        slider.config(from_=from_init, to=to_init, resolution=resolution_init, variable=variable_init, orient=tk.HORIZONTAL, label=label_init,length=length_init, foreground=foreground_init,bg=bg_init,width=width_init, font=font_init)
        slider.set(initial_val)   
        slider.bind("<ButtonRelease-1>", function_bind)
     
    def selected_color_options(self,selected_color_option):
        width, height = selected_color_option.split('x')
        self.color_width,self.color_height = int(width),int(height)
        print("color wxh: ",width,height)
        self.color_selector.config(width=6,bg = self.hex_colors["white"],foreground=self.hex_colors["black"])
        self.create_color_fps_selector()
        #OFF RUN STREAM
        self.run_camera_FLAG = 0#not self.run_camera_FLAG 
        self.RUN_camera_button.config(bg = self.hex_colors["gray"],foreground=self.hex_colors["black"])
        #write
        self.write_data()
        
    def create_color_fps_selector(self):
        if self.color_width>=320 and self.color_width<=960 and self.color_width>=180 and self.color_height<=540:
               self.color_FPS_options = [6,15,30,60]
        else:
               self.color_FPS_options = [6,15,30]
        #SHOW FPS SELECTOR
        self.int_variables[1].set(self.color_FPS_options[-1])
        self.color_FPS_selector = tk.OptionMenu(self.CAMERA_frame,self.int_variables[1],*self.color_FPS_options,command=self.selected_color_FPS_options)
        self.color_FPS_selector.config(width=6,bg = self.hex_colors["gray"],foreground=self.hex_colors["black"])
        self.color_FPS_selector.place_forget()
        self.color_FPS_selector.place(x=self.CAMERA_frame_width*0.13, y=self.CAMERA_frame_height*0.65)
        #write color
        self.write_data()

    def selected_color_FPS_options(self,selected_color_FPS_option):
        self.selected_color_FPS_FLAG = True
        self.color_FPS = selected_color_FPS_option
        #print(self.color_FPS)
        self.color_FPS_selector.config(width=6,bg = self.hex_colors["white"],foreground=self.hex_colors["black"])
        print("color FPS: ",self.color_FPS)
        #OFF RUN STREAM
        self.run_camera_FLAG = 0#not self.run_camera_FLAG 
        self.RUN_camera_button.config(bg = self.hex_colors["gray"],foreground=self.hex_colors["black"])
        #write
        self.write_data()



    def write_data(self,write_comm = False):
        s = ','


        txt.txt_w(txt_file=self.txt_file_path,selected_line=1,
                    content_to_replace=f"camera_stream:{self.depth_width}{s}{self.depth_height}{s}{self.depth_FPS}{s}{self.color_width}{s}{self.color_height}{s}{self.color_FPS}"+
                    f"{s}{int(self.camera_perceptible_depth_slider.get())}{s}{int(self.camera_laser_power_slider.get())}{s}{self.double_variables[2].get()}{s}{self.double_variables[3].get()}{s}{self.double_variables[4].get()}{s}{int(self.double_variables[5].get())}{s}{int(self.double_variables[6].get())}"+
                    f"{s}{int(self.run_camera_FLAG)}{s}{int(self.stream_2d_FLAG)}{s}{int(self.stream_3d_interactive)}{s}{int(self.stream_3d_static)}{s}{self.gui_key}{s}{int(self.decimate_value)}{s}{int(self.camera_pause_FLAG)}{s}{int(self.MAKE_RECONSTRUCTION_button_FLAG)}{s}{int(self.RESET_SAVED_FRAMES_FLAG)}")

        #if write_comm:
        txt.txt_w(txt_file=self.txt_file_path,selected_line=2,
                content_to_replace=f"communication:{self.ip_entry.get()}{s}{self.camera_port_entry.get()}{s}{self.parameters_port_entry.get()}")

        txt.txt_w(txt_file=self.txt_file_path,selected_line=3,
                  content_to_replace=f"manual:{int(self.manual_robot_vel_slider.get())},{self.robot_wheels_command},{self.robot_tower_command},{self.robot_head_command},{self.robot_tweezer_efector_command},{int(self.ROBOT_MODE_FLAG)}")

        
        txt.txt_w(txt_file=self.txt_file_path,selected_line=4,
                content_to_replace=f"pcd_coords_configs:{self.init_orientation_entry.get()},{self.final_orientation_entry.get()},{self.init_tower_entry.get()},{self.final_tower_entry.get()},{int(self.init_efector_FLAG)},{int(self.final_efector_FLAG)}")

        txt.txt_w(txt_file=self.txt_file_path,selected_line=5,
                content_to_replace=f"automatic_cmd:{int(self.start_ROBOT_FLAG)},{int(self.return_ROBOT_FLAG)}")
        
        txt.txt_w(txt_file="MEDIA\\dependencies\\others\\slam_status.txt",selected_line=3,content_to_replace=f"timesampling:{int(self.SLAM_time_sampling.get())}")

        txt.txt_w(txt_file="MEDIA\\dependencies\\others\\pointcloud_confs.txt",selected_line=1,content_to_replace=f"pointsize:{self.SLAM_3D_point_size_slider.get()}")

        
    def write_robot_cmd(self):
        txt.txt_w(txt_file=self.txt_file_path2,selected_line=1,
                content_to_replace=f"pcd_path:{self.pointcloud_path}")
        
        txt.txt_w(txt_file=self.txt_file_path2,selected_line=3,
                content_to_replace=f"pcd_coords_configs:{self.init_orientation_entry.get()},{self.final_orientation_entry.get()},{self.init_tower_entry.get()},{self.final_tower_entry.get()},{int(self.init_efector_FLAG)},{int(self.final_efector_FLAG)}")

        txt.txt_w(txt_file=self.txt_file_path2,selected_line=4,
                content_to_replace=f"automatic_cmd:{int(self.start_ROBOT_FLAG)},{int(self.return_ROBOT_FLAG)}")
        
        txt.txt_w(txt_file=self.txt_file_path2,selected_line=5,
                content_to_replace=f"save_to:{self.default_reconstruction_path}")
        

    def get_sliders_values(self,event):
        sl1=self.camera_perceptible_depth_slider.get()
        sl2=self.camera_laser_power_slider.get()
        sl3=self.visualizer_2D_size_slider.get()
        sl4=self.SLAM_3D_voxel_size_slider.get()
        sl5=self.SLAM_3D_point_size_slider.get()
        sl6=self.SLAM_3D_uniform_down_slider.get()
        sl7=self.SLAM_3D_num_pcd.get()
        sl8=self.manual_robot_vel_slider.get()
        #write camera
        self.write_data()


    def selected_depth_options(self,selected_depth_option):
        width, height = selected_depth_option.split('x')
        self.depth_width,self.depth_height = int(width),int(height)
        print("depth wxh: ",width,height)
        self.depth_selector.config(width=6,bg = self.hex_colors["white"],foreground=self.hex_colors["black"])
        self.create_depth_fps_selector()
        #OFF RUN STREAM
        self.run_camera_FLAG = 0#not self.run_camera_FLAG 
        self.RUN_camera_button.config(bg = self.hex_colors["gray"],foreground=self.hex_colors["black"])
        #write info
        self.write_data()


    def create_depth_fps_selector(self):
        if self.depth_width == 256 and self.depth_height == 144:
                                    self.depth_FPS_options= [90,300]
        if self.depth_width >= 424 and self.depth_width <= 848 and self.depth_height >= 240 and self.depth_height <= 480:
                                    self.depth_FPS_options = [6,15,30,60,90]       
        if self.depth_width == 848 and self.depth_height == 100:
                                    self.depth_FPS_options= [100,300]
        if self.depth_width == 1280 and self.depth_height == 720:
                                    self.depth_FPS_options = [6,15,30]
        #SHOW FPS SELECTOR
        self.int_variables[0].set(self.depth_FPS_options[-1])
        self.depth_FPS_selector = tk.OptionMenu(self.CAMERA_frame,self.int_variables[0],*self.depth_FPS_options,command=self.selected_depth_FPS_options)
        self.depth_FPS_selector.config(width=6,bg = self.hex_colors["gray"],foreground=self.hex_colors["black"])
        self.depth_FPS_selector.place_forget()
        self.depth_FPS_selector.place(x=self.menu_frame_width*0.035, y=self.menu_frame_height*0.65)
        #write
        self.write_data()

    def selected_depth_FPS_options(self,selected_depth_FPS_option):
        self.selected_depth_FPS_FLAG = True
        self.depth_FPS = selected_depth_FPS_option
        self.depth_FPS_selector.config(width=6,bg = self.hex_colors["white"],foreground=self.hex_colors["black"])
        print("depth FPS: ",self.depth_FPS)
        #OFF RUN STREAM
        self.run_camera_FLAG = 0#not self.run_camera_FLAG 
        self.RUN_camera_button.config(bg = self.hex_colors["gray"],foreground=self.hex_colors["black"])
        #write
        self.write_data()





 
    def toggle_run_camera_button(self):
        self.run_camera_FLAG = not self.run_camera_FLAG 
        if self.run_camera_FLAG:
            self.RUN_camera_button.config(bg = self.hex_colors["white"],foreground=self.hex_colors["black"])
        else:
            self.RUN_camera_button.config(bg = self.hex_colors["gray"],foreground=self.hex_colors["black"])
        self.write_data()

    def toggle_decimation_button(self):
        self.decimate_value = self.decimate_value+1
        if self.decimate_value>3:
            self.decimate_value = 0
        self.decimate_val_button.config(text=f" Dec: {int(self.decimate_value)}",bg = self.hex_colors["white"],foreground=self.hex_colors["black"])
        self.write_data()

    def write_SLAM_info(self):
        txt.txt_w(txt_file="MEDIA\\dependencies\\others\\slam_status.txt",selected_line=1,
                content_to_replace=f"slam_status:{int(self.MAKE_RECONSTRUCTION_button_FLAG)},{int(self.RESET_SAVED_FRAMES_FLAG)}")

    def toggle_clear_frames_button(self):
        self.RESET_SAVED_FRAMES_FLAG = not self.RESET_SAVED_FRAMES_FLAG
        if self.RESET_SAVED_FRAMES_FLAG:
            self.MAKE_RECONSTRUCTION_button_FLAG = False
            self.RESET_SAVED_FRAMES_button.config(bg = self.hex_colors["white"],foreground=self.hex_colors["black"],font=self.fonts[8])
        else:
            self.RESET_SAVED_FRAMES_button.config(bg = self.hex_colors["gray"],foreground=self.hex_colors["black"],font=self.fonts[8])
        self.write_SLAM_info()
        self.write_data()

    def toggle_MAKE_RECONSTRUCTION_button(self):
        if not self.RESET_SAVED_FRAMES_FLAG:
            self.MAKE_RECONSTRUCTION_button_FLAG = not self.MAKE_RECONSTRUCTION_button_FLAG
            if self.MAKE_RECONSTRUCTION_button_FLAG:
                self.MAKE_RECONSTRUCTION_button.config(bg = self.hex_colors["white"],foreground=self.hex_colors["black"],font=self.fonts[8])
            else:
                self.MAKE_RECONSTRUCTION_button.config(bg = self.hex_colors["gray"],foreground=self.hex_colors["black"],font=self.fonts[8])
            self.write_data()
            self.write_SLAM_info()

    def toggle_camera_pause_button(self):
        self.camera_pause_FLAG = not self.camera_pause_FLAG
        if not self.camera_pause_FLAG:
            self.camera_pause_button.config(text=f"RUN",bg = self.hex_colors["white"],foreground=self.hex_colors["black"],font=self.fonts[6])
        else:
            self.camera_pause_button.config(text=f"paused",bg = self.hex_colors["gray"],foreground=self.hex_colors["black"],font=self.fonts[6])
        self.write_data()

    def toggle_2D_visualizer_button(self):
        self.stream_2d_FLAG = not self.stream_2d_FLAG 
        if self.stream_2d_FLAG:
            self.visualizer_2D_button.config(bg = self.hex_colors["white"],foreground=self.hex_colors["black"])
        else:
            self.visualizer_2D_button.config(bg = self.hex_colors["gray"],foreground=self.hex_colors["black"])
        self.write_data()

    def toggle_3D_visualizer_button(self):
        self.stream_3d_interactive = not self.stream_3d_interactive
        if self.stream_3d_interactive:
            self.visualizer_3D_button.config(bg = self.hex_colors["white"],foreground=self.hex_colors["black"])
        else:
            self.visualizer_3D_button.config(bg = self.hex_colors["gray"],foreground=self.hex_colors["black"])
        self.write_data()

    def toggle_3D_visualizer_SLAM_button(self):
        self.stream_3d_static = not self.stream_3d_static
        if self.stream_3d_static:
            self.visualizer_3D_local_button.config(bg = self.hex_colors["white"],foreground=self.hex_colors["black"])
        else:
            self.visualizer_3D_local_button.config(bg = self.hex_colors["gray"],foreground=self.hex_colors["black"])
        self.write_data()


    def toggle_main_menu_button(self):
        #SHOW the MENU Frame and items 
        self.update_MENU_frame_items()     
        #Oculto cualquiera de los otros frames COM, CAMERA, MANUAL, AUTOMATIC
        self.hide_frame("com")
        self.hide_frame("camera")
        self.hide_frame(selected_frame="manual")
        self.hide_frame(selected_frame="automatic")
        self.FRAME_STATE = 0
        self.write_data()

    def toggle_comunication_button(self):
        #SHOW the COM Frame and items 
        self.update_COM_frame_items()
        #go to frame comunication control and hide anyone
        self.hide_frame(selected_frame="menu")
        self.hide_frame(selected_frame="camera")
        self.hide_frame(selected_frame="manual")
        self.hide_frame(selected_frame="automatic")

        self.FRAME_STATE = 1
        self.write_data()

    def toggle_camera_button(self):
        #SHOW the CAMERA Frame and items 
        self.update_CAMERA_frame_items()
        #go to frame comunication control and hide anyone MENU, COM, CAMERA, MANUAL, AUTOMATIC
        self.hide_frame(selected_frame="menu")
        self.hide_frame(selected_frame="com")
        self.hide_frame(selected_frame="manual")
        self.hide_frame(selected_frame="automatic")

        self.FRAME_STATE = 2
        self.write_data()

    def toggle_connect_ip_ports_button(self):
        self.connect_FLAG = not self.connect_FLAG 

        self.toggle_save_ip_ports_button()
        if self.connect_FLAG:
            self.comunication_connect_button.config(bg=self.hex_colors["white"])
            self.client_process = subprocess.Popen(["python", "my_client.py"])

            self.client_camera_process = subprocess.Popen(["python", "frames_client.py"])  # Ejecutar el script
        else:
            self.comunication_connect_button.config(bg=self.hex_colors["gray"])

            self.client_process.terminate() 
            self.client_process.wait()

            self.client_camera_process.terminate()
            self.client_camera_process.wait()
        self.write_data()


    def toggle_save_ip_ports_button(self):
        self.write_data()

    def toggle_mode_automatic_button(self):
        ##############################################################
        #                AUTOMATIC MODE
        ##############################################################
        self.bool_variables[1].set(True)#not self.bool_variables[1].get()) #MODO AUTOMATICO (true)
        self.bool_variables[0].set(False) #MODO MANUAL (FALSE)
        self.ROBOT_MODE_FLAG = False
        if self.bool_variables[1].get():  
            self.mode_automatic_button.config(bg=self.hex_colors["white"])
            self.mode_manual_button.config(bg=self.hex_colors["gray"])
            self.automatic_status.config(bg=self.hex_colors["green_medium_light"])
            self.manual_status.config(bg=self.hex_colors["red_medium_light"])
        else:                                       
            self.mode_automatic_button.config(bg=self.hex_colors["gray"])
            self.mode_manual_button.config(bg=self.hex_colors["white"])
            self.automatic_status.config(bg=self.hex_colors["red_medium_light"])
            self.manual_status.config(bg=self.hex_colors["green_medium_light"])
        self.hide_frame("menu")
        self.hide_frame("com")
        self.hide_frame("camera")
        self.hide_frame("manual")
        self.FRAME_STATE = 4
        self.write_data()

    def toggle_mode_button(self):
        ##############################################################
        #                MANUAL MODE
        ##############################################################
        #self.bool_variables[0].set(not self.bool_variables[0].get())
        self.bool_variables[0].set(True)#not self.bool_variables[1].get()) #MODO AUTOMATICO (false)
        self.bool_variables[1].set(False) #MODO MANUAL (true)
        self.ROBOT_MODE_FLAG = True
        if self.bool_variables[0].get(): 
            self.mode_manual_button.config(bg=self.hex_colors["white"])
            self.mode_automatic_button.config(bg=self.hex_colors["gray"])
            self.manual_status.config(bg=self.hex_colors["green_medium_light"])
            self.automatic_status.config(bg=self.hex_colors["red_medium_light"])
            #self.toggle_size(w=0.75,h=0.45)
        else:                                      
            self.mode_manual_button.config(bg=self.hex_colors["gray"])
            self.mode_automatic_button.config(bg=self.hex_colors["white"])
            self.manual_status.config(bg=self.hex_colors["red_medium_light"])
            self.automatic_status.config(bg=self.hex_colors["green_medium_light"])
        #SHOW the CAMERA Frame and items 
        #self.update_MANUAL_frame_items()
        self.update_MANUAL_frame_items()
        self.hide_frame("menu")
        self.hide_frame("com")
        self.hide_frame("camera")
        self.hide_frame("automatic")
        self.FRAME_STATE = 3
        self.write_data()

    def hide_frame(self,selected_frame):
        frame_to_hide = None

        if selected_frame == "menu":
            frame_to_hide = self.MENU_frame
        if selected_frame == "com":
            frame_to_hide = self.COM_frame
        if selected_frame == "camera":
            frame_to_hide = self.CAMERA_frame
        if selected_frame == "manual":
            frame_to_hide = self.MANUAL_frame
        if selected_frame == "automatic":
            frame_to_hide = self.AUTOMATIC_frame
            
        if frame_to_hide.winfo_ismapped():  # Si fram MENU está visible
            frame_to_hide.place_forget()     # Oculta del menu para dirigirse al frame de Comunicacion
        self.write_data()

    def create_frame_items(self,selected_frame="self",num_labels=0,num_buttons=0,num_sliders=0,num_entry=[0,10]):
        label_items = []
        button_items = []
        slider_items = []
        entry_items = []
        my_frame = None
        if selected_frame == "self":
            my_frame = self #ROOT SUPER CLASS WINDOW
        if selected_frame == "status_frame":
            my_frame = self.status_frame #STATUS FRAME SUBSECTION
        if selected_frame == "menu_frame":
            my_frame = self.MENU_frame #MENU FRAME SUBSECTION
        if selected_frame == "com_frame":
            my_frame = self.COM_frame #COM FRAME SUBSECTION
        if selected_frame == "camera_frame":
            my_frame = self.CAMERA_frame #CAMERA FRAME SUBSECTION
        if selected_frame == "manual_frame":
            my_frame = self.MANUAL_frame #MANUAL FRAME SUBSECTION
        if selected_frame == "automatic_frame":
            my_frame = self.AUTOMATIC_frame #AUTOMATIC FRAME SUBSECTION

        #CREATE LABELS (INDICATORS)
        if num_labels>=1: 
            
            for i in range(0, num_labels):  
                label = tk.Label(my_frame, text="          ",bg=self.hex_colors["gray"], font=self.fonts[11], width=9, height=2,bd=0, relief="solid")
                label_items.append(label)

        if num_buttons>=1:
            for i2 in range(0, num_buttons):  
                button = tk.Button(my_frame, text="          ", command=None,bg=self.hex_colors["gray"],font=self.fonts[10],bd=3, relief="solid")
                button_items.append(button)

        if num_sliders>=1:
            for i3 in range(0,num_sliders):
                
                slider = tk.Scale(my_frame, from_=0.0, to=0.1, resolution=0.001, variable=None, orient=tk.HORIZONTAL, label="     ",length=80) 
                slider_items.append(slider)
        
        if num_entry[0]>=1:
            for i4 in range(0,num_entry[0]):
                entry = tk.Entry(my_frame, width=num_entry[1])
                entry_items.append(entry)
    
        
        print(selected_frame," items [labels,buttons,sliders,entries]: ", len(label_items),len(button_items),len(slider_items),len(entry_items))

        return label_items, button_items, slider_items,entry_items

    def update_COM_frame_items(self):
        #REFRESH COM FRAME
        self.COM_frame.place(x=self.screen_width*self.faceplates_x_divisor, y=self.screen_height*self.faceplates_y_divisor*0.6, width=self.screen_width*self.faceplates_width_divisor, height=self.screen_height*self.faceplates_height_divisor)  # Posición en la mitad derecha
        self.COM_frame_width =  self.COM_frame.winfo_width()  # Ancho del Frame
        self.COM_frame_height = self.COM_frame.winfo_height()  # Altura del Frame

        #REFRESH COM ITEMS
        #entries
        self.ip_entry.place(x=self.COM_frame_width*0.045, y=self.COM_frame_height*0.5)
        self.camera_port_entry.place(x=self.COM_frame_width*0.22, y=self.COM_frame_height*0.5)
        self.parameters_port_entry.place(x=self.COM_frame_width*0.41, y=self.COM_frame_height*0.5)
        
        #labels
        self.com_ip_label.place(x=self.COM_frame_width*0.045, y=self.COM_frame_height*0.28)
        self.com_ip_label.config(text="IP Config",width=len("IP Config")+1,bg=self.hex_colors["light_gray"],bd=0, relief="solid", height=1,font=self.fonts[10])
        
        self.com_camera_stream_port_label.place(x=self.COM_frame_width*0.2, y=self.COM_frame_height*0.28)
        self.com_camera_stream_port_label.config(text="Camera PORT",width=len("Camera Stream PORT")+1,bg=self.hex_colors["light_gray"],bd=0, relief="solid", height=1,font=self.fonts[10])

        self.com_parameters_port_label.place(x=self.COM_frame_width*0.4, y=self.COM_frame_height*0.28)
        self.com_parameters_port_label.config(text="Param. PORT",width=len("Parameters PORT"),bg=self.hex_colors["light_gray"],bd=0, relief="solid", height=1,font=self.fonts[10])

        self.com_TITLE.place(x=self.COM_frame_width*0.25, y=self.COM_frame_height*0.035)
        self.com_TITLE.config(text="COMMUNICATION CONTROL - [OPTIONS]",width=len("COMMUNICATION CONTROL - [OPTIONS]")+2,bg=self.hex_colors["light_gray"],bd=0, relief="solid", height=1,font=self.fonts[13])
        
        #buttons 
        self.comunication_save_button.place(x=self.COM_frame_width*0.59, y=self.COM_frame_height*0.5)
        self.comunication_connect_button.place(x=self.COM_frame_width*0.81, y=self.COM_frame_height*0.5)

    def update_CAMERA_frame_items(self):
        #REFRESH MENU FRAME
        self.CAMERA_frame.place(x=self.screen_width*self.faceplates_x_divisor, y=self.screen_height*self.faceplates_y_divisor*0.6, width=self.screen_width*self.faceplates_width_divisor, height=self.screen_height*self.faceplates_height_divisor)  # Posición en la mitad derecha
        self.CAMERA_frame_width =  self.CAMERA_frame.winfo_width()  # Ancho del Frame
        self.CAMERA_frame_height = self.CAMERA_frame.winfo_height()  # Altura del Frame
        #REFRESH CAMERA LABELS
        self.camera_stream_label.place(x=self.CAMERA_frame_width*0.28, y=self.CAMERA_frame_height*0.015)
        self.camera_stream_label.config(text="CAMERA & STREAM CONTROL - [OPTIONS]",width=len("CAMERA & STREAM CONTROL - [OPTIONS]")+1,bg=self.hex_colors["light_gray"],bd=0, relief="solid", height=1,font=self.fonts[13])
        

        
        self.fps_label.place(x=self.CAMERA_frame_width*0.005, y=self.CAMERA_frame_height*0.71)
        self.fps_label.config(text="FPS",width=len("FPS")+1,bg=self.hex_colors["light_gray"],bd=0, relief="solid", height=1,font=self.fonts[7])
        
        self.resolution_label.place(x=self.CAMERA_frame_width*0.005, y=self.CAMERA_frame_height*0.47)
        self.resolution_label.config(text="RES",width=len("RES")+1,bg=self.hex_colors["light_gray"],bd=0, relief="solid", height=1,font=self.fonts[7])
        
        self.depth_label.place(x=self.CAMERA_frame_width*0.06, y=self.CAMERA_frame_height*0.31)
        self.depth_label.config(text="DEPTH",width=len("DEPTH")+1,bg=self.hex_colors["light_gray"],bd=0, relief="solid", height=1,font=self.fonts[7])
        
        self.color_label.place(x=self.CAMERA_frame_width*0.15, y=self.CAMERA_frame_height*0.31)
        self.color_label.config(text="COLOR",width=len("COLOR")+1,bg=self.hex_colors["light_gray"],bd=0, relief="solid", height=1,font=self.fonts[7])
        
        self.gui_3D_interactive_label.place(x=self.CAMERA_frame_width*0.545, y=self.CAMERA_frame_height*0.74)
        self.gui_3D_interactive_label.config(text="[R]-RESET,[Z]-P_SZ,[C]-COL,[S]-S_PNG,[E]-S_PLY",width=len("[R]-RESET,[Z]-P_SZ,[C]-COL,[S]-S_PNG,[E]-S_PLY")+3,bg=self.hex_colors["light_gray"],bd=0, relief="solid", height=3,font=self.fonts[5])
        
        self.gui_3D_interactive_entry.place(x=self.CAMERA_frame_width*0.65, y=self.CAMERA_frame_height*0.635)
        self.gui_3D_interactive_entry.config(width=7)

        #REFRESH CAMERA FRAME ITEMS
        #buttons  
        #self.decimate_val_button.place(x=self.CAMERA_frame_width*0.65, y=self.CAMERA_frame_height*0.635)
        #self.decimate_val_button.config(text=f"Dec:{self.decimate_value}",width=6,bg=self.hex_colors["white"],bd=2, relief="solid",font=self.fonts[6])

        #self.camera_pause_button.place(x=self.CAMERA_frame_width*0.60, y=self.CAMERA_frame_height*0.635)
        #self.camera_pause_button.config(bg = self.hex_colors["white"],foreground=self.hex_colors["black"],font=self.fonts[6])

        self.RUN_camera_button.place(x=self.CAMERA_frame_width*0.03, y=self.CAMERA_frame_height*0.09)
        self.visualizer_2D_button.place(x=self.CAMERA_frame_width*0.45, y=self.CAMERA_frame_height*0.40)
        self.visualizer_3D_button.place(x=self.CAMERA_frame_width*0.60, y=self.CAMERA_frame_height*0.40)
        self.visualizer_3D_local_button.place(x=self.CAMERA_frame_width*0.81, y=self.CAMERA_frame_height*0.085)
        #**********************************SLIDERS*************************************************
        self.camera_perceptible_depth_slider.place(x=self.CAMERA_frame_width*0.23, y=self.CAMERA_frame_height*0.46)
        self.MAKE_RECONSTRUCTION_button.place(x=self.CAMERA_frame_width*0.23, y=self.CAMERA_frame_height*0.79)
        self.MAKE_RECONSTRUCTION_button.config(foreground=self.hex_colors["black"],font=self.fonts[8])
        #time.sleep(0.01)
        #if self.RESET_SAVED_FRAMES_FLAG:
        #    self.MAKE_RECONSTRUCTION_button_FLAG = False
        #    self.MAKE_RECONSTRUCTION_button.config(bg = self.hex_colors["gray"],foreground=self.hex_colors["black"],font=self.fonts[8])
        try:
            slam_DICT_vals, slam_vals = txt.txt_r("MEDIA\\dependencies\\others\\slam_status.txt", 1)
            pcd_num_DICT_vals, pcd_num_vals = txt.txt_r("MEDIA\\dependencies\\others\\slam_status.txt", 2)


            slam_local_par = slam_DICT_vals["slam_status"]
            pcd_local_par = pcd_num_DICT_vals["pcd_buffer"]

            self.saved_pointcloud_label.place(x=self.CAMERA_frame_width*0.38, y=self.CAMERA_frame_height*0.2)
            self.saved_pointcloud_label.config(text=f"SAVED POINTCLOUDS: {str(pcd_local_par[0])} ",width=len("SAVED POINTCLOUDS:    "),bg=self.hex_colors["gray"],bd=0, relief="solid", height=1,font=self.fonts[11])

            #print(cam_local_par)
            if slam_local_par[1]:
                self.RESET_SAVED_FRAMES_button.config(bg = self.hex_colors["white"],foreground=self.hex_colors["black"],font=self.fonts[8])
            else:
                self.RESET_SAVED_FRAMES_button.config(bg = self.hex_colors["gray"],foreground=self.hex_colors["black"],font=self.fonts[8])

            if slam_local_par[0]:
                self.MAKE_RECONSTRUCTION_button.config(bg = self.hex_colors["white"],foreground=self.hex_colors["black"],font=self.fonts[8])
            else:
                self.MAKE_RECONSTRUCTION_button.config(bg = self.hex_colors["gray"],foreground=self.hex_colors["black"],font=self.fonts[8])
        except (TypeError,KeyError,IndexError) as IE:
            print(IE)

        self.camera_laser_power_slider.place(x=self.CAMERA_frame_width*0.33, y=self.CAMERA_frame_height*0.47)
        self.RESET_SAVED_FRAMES_button.place(x=self.CAMERA_frame_width*0.33, y=self.CAMERA_frame_height*0.79)
        self.RESET_SAVED_FRAMES_button.config(foreground=self.hex_colors["black"],font=self.fonts[8])

        self.visualizer_2D_size_slider.place(x=self.CAMERA_frame_width*0.45, y=self.CAMERA_frame_height*0.64)
        #    3D SLAM SLIDERS
        self.SLAM_time_sampling.place(x=self.CAMERA_frame_width*0.23, y=self.CAMERA_frame_height*0.14)

        self.SLAM_3D_voxel_size_slider.place(x=self.CAMERA_frame_width*0.77, y=self.CAMERA_frame_height*0.31)
        self.SLAM_3D_point_size_slider.place(x=self.CAMERA_frame_width*0.87, y=self.CAMERA_frame_height*0.31)
        self.SLAM_3D_uniform_down_slider.place(x=self.CAMERA_frame_width*0.77, y=self.CAMERA_frame_height*0.64)
        self.SLAM_3D_num_pcd.place(x=self.CAMERA_frame_width*0.87, y=self.CAMERA_frame_height*0.64)
        #**********************************SELECTOS*************************************************

        #  CAMERA CONFIS 
        self.depth_selector.place(x=self.CAMERA_frame_width*0.035, y=self.CAMERA_frame_height*0.41)
        if self.selected_depth_FPS_FLAG:
            self.depth_FPS_selector.place(x=self.CAMERA_frame_width*0.035, y=self.CAMERA_frame_height*0.65)

        self.color_selector.place(x=self.CAMERA_frame_width*0.13, y=self.CAMERA_frame_height*0.41)
        if self.selected_color_FPS_FLAG:
            self.color_FPS_selector.place(x=self.CAMERA_frame_width*0.13, y=self.CAMERA_frame_height*0.65)


    def update_MANUAL_frame_items(self):
        self.MANUAL_frame.place(x=self.screen_width*self.faceplates_x_divisor, y=self.screen_height*self.faceplates_y_divisor*0.6, width=self.screen_width*self.faceplates_width_divisor, height=self.screen_height*self.faceplates_height_divisor)  # Posición en la mitad derecha
        self.manual_frame_width =  self.MANUAL_frame.winfo_width()  # Ancho del Frame
        self.manual_frame_height = self.MANUAL_frame.winfo_height()  # Altura del Frame

        self.manual_mode_label.place(x=self.manual_frame_width*0.31, y=self.manual_frame_height*0.005)
        self.manual_mode_label.config(text="ROBOT MANUAL CONTROL - [OPTIONS]",width=len("ROBOT MANUAL CONTROL - [OPTIONS]")+1,bg=self.hex_colors["light_gray"],bd=0, relief="solid", height=1,font=self.fonts[13])
        #sliders
        self.manual_robot_vel_slider.place(x=self.manual_frame_width*0.4, y=self.manual_frame_height*0.25)
        #labels
        #status label
        self.manual_CMD_label.place(x=self.manual_frame_width*0.29, y=self.manual_frame_height*0.72)
        #self.manual_CMD_label.config(text=" CMD[ REPOSE ]:[   NONE   ] ",width=len(" CMD[              ]:[              ] ")+1,bg=self.hex_colors["gray"],bd=0, relief="solid", height=1,font=self.fonts[13])
        self.up_label.place(x=self.manual_frame_width*0.15, y=self.manual_frame_height*0.14,
                            width=60, height=45)

        self.down_label.place(x=self.manual_frame_width*0.15, y=self.manual_frame_height*0.68,
                            width=60, height=45)

        self.left_label.place(x=self.manual_frame_width*0.085, y=self.manual_frame_height*0.41,
                            width=60, height=45)

        self.right_label.place(x=self.manual_frame_width*0.21, y=self.manual_frame_height*0.41,
                        width=60, height=45)
        #diagonal up,down,left,right
        self.diagonal_up_right_label.place(x=self.manual_frame_width*0.21, y=self.manual_frame_height*0.14,
                        width=60, height=45)
        
        self.diagonal_up_left_label.place(x=self.manual_frame_width*0.085, y=self.manual_frame_height*0.14,
                        width=60, height=45)
        
        self.diagonal_down_left_label.place(x=self.manual_frame_width*0.085, y=self.manual_frame_height*0.68,
                            width=60, height=45)
        
        self.diagonal_down_right_label.place(x=self.manual_frame_width*0.21, y=self.manual_frame_height*0.68,
                            width=60, height=45)
        
        self.twist_right_label.place(x=self.manual_frame_width*0.28, y=self.manual_frame_height*0.14,
                            width=60, height=68) 
        
        self.twist_left_label.place(x=self.manual_frame_width*0.017, y=self.manual_frame_height*0.14,
                            width=60, height=68) 


        #center
    
        self.center_label.place(x=self.manual_frame_width*0.15, y=self.manual_frame_height*0.41,
                            width=61, height=45)
        
        #tower
        self.tower_up_label.place(x=self.manual_frame_width*0.75, y=self.manual_frame_height*0.14,
                            width=60, height=68)  
        
        self.tower_down_label.place(x=self.manual_frame_width*0.75, y=self.manual_frame_height*0.56,
                    width=60, height=68)  
        #head
        self.head_left_label.place(x=self.manual_frame_width*0.687, y=self.manual_frame_height*0.125,
                            width=60, height=45)  
        
        self.head_right_label.place(x=self.manual_frame_width*0.687, y=self.manual_frame_height*0.386,
                            width=60, height=45) 

        self.tweezer_label.place(x=self.manual_frame_width*0.817, y=self.manual_frame_height*0.125,
                            width=100, height=133)  
       
        
        #self.RESET_MANUAL()

    def update_AUTOMATIC_frame_items(self):
        #REFRESH MENU FRAME
        self.AUTOMATIC_frame.place(x=self.screen_width*self.faceplates_x_divisor, y=self.screen_height*self.faceplates_y_divisor*0.6, width=self.screen_width*self.faceplates_width_divisor, height=self.screen_height*self.faceplates_height_divisor)  # Posición en la mitad derecha
        self.AUTOMATIC_frame_width =  self.AUTOMATIC_frame.winfo_width()  # Ancho del Frame
        self.AUTOMATIC_frame_height = self.AUTOMATIC_frame.winfo_height()  # Altura del Frame        

        #labels
        self.AUTOMATIC_TITLE.place(x=self.menu_frame_width*0.32, y=self.menu_frame_height*0.045)
        self.AUTOMATIC_TITLE.config(text="AUTOMATIC MENU - [OPTIONS]",width=len("AUTOMATIC MENU - [OPTIONS]")+2,bg=self.hex_colors["light_gray"],bd=0, relief="solid", height=1,font=self.fonts[13])

        self.save_to_button.place(x=self.menu_frame_width*0.025, y=self.menu_frame_height*0.07)
        self.save_to_path_label.config(text="..."+self.default_reconstruction_path[-30:],width=len("..."+self.default_reconstruction_path[-30:])+2,bg=self.hex_colors["light_gray"],bd=0, relief="solid", height=1,font=self.fonts[9])
        self.save_to_path_label.place(x=self.menu_frame_width*0.025, y=self.menu_frame_height*0.27)

        self.open_from__button.place(x=self.menu_frame_width*0.025, y=self.menu_frame_height*0.41)
        self.open_from_path_label.config(text="..."+self.pointcloud_path[-30:],width=len("..."+self.pointcloud_path[-30:])+2,bg=self.hex_colors["light_gray"],bd=0, relief="solid", height=1,font=self.fonts[9])
        self.open_from_path_label.place(x=self.menu_frame_width*0.025, y=self.menu_frame_height*0.6)

        self.interactive_coordinate_selector_button.place(x=self.menu_frame_width*0.025, y=self.menu_frame_height*0.75)
        self.return_robot_button.place(x=self.menu_frame_width*0.47, y=self.menu_frame_height*0.75)
        self.start_robot_tracking_button.place(x=self.menu_frame_width*0.75, y=self.menu_frame_height*0.75)


        self.INIT_COORDINATE_label.place(x=self.menu_frame_width*0.48, y=self.menu_frame_height*0.21)
        self.INIT_COORDINATE_label.config(text="INITIAL COORDINATE",width=len("INITIAL COORDINATE")+2,bg=self.hex_colors["light_gray"],bd=0, relief="solid", height=1,font=self.fonts[11])

        self.FINAL_COORDINATE_label.place(x=self.menu_frame_width*0.72, y=self.menu_frame_height*0.21)
        self.FINAL_COORDINATE_label.config(text="FINAL COORDINATE",width=len("FINAL COORDINATE")+2,bg=self.hex_colors["light_gray"],bd=0, relief="solid", height=1,font=self.fonts[11])

        self.init_orientation_label.place(x=self.menu_frame_width*0.39, y=self.menu_frame_height*0.35)
        self.init_orientation_label.config(text="Orientation         ",width=len("Orientation         ")+2,bg=self.hex_colors["light_gray"],bd=0, relief="solid", height=1,font=self.fonts[9])
        self.init_orientation_entry.place(x=self.menu_frame_width*0.52, y=self.menu_frame_height*0.35)
        self.final_orientation_entry.place(x=self.menu_frame_width*0.75, y=self.menu_frame_height*0.35)


        self.init_tower_elevation.place(x=self.menu_frame_width*0.39, y=self.menu_frame_height*0.47)
        self.init_tower_elevation.config(text="Tower El.(mm)",width=len("Tower El.(mm)")+2,bg=self.hex_colors["light_gray"],bd=0, relief="solid", height=1,font=self.fonts[9])
        self.init_tower_entry.place(x=self.menu_frame_width*0.52, y=self.menu_frame_height*0.45)
        self.final_tower_entry.place(x=self.menu_frame_width*0.75, y=self.menu_frame_height*0.45)

        self.init_efector_label.place(x=self.menu_frame_width*0.39, y=self.menu_frame_height*0.59)
        self.init_efector_label.config(text="Efector Posit.",width=len("Efector Posit.")+2,bg=self.hex_colors["light_gray"],bd=0, relief="solid", height=1,font=self.fonts[9])
        self.init_efector_conf_button.place(x=self.menu_frame_width*0.52, y=self.menu_frame_height*0.58)
        self.init_efector_conf_button.config(width=9, relief="solid", height=1,font=self.fonts[9])
        self.final_efector_conf_button.place(x=self.menu_frame_width*0.75, y=self.menu_frame_height*0.58)
        self.final_efector_conf_button.config(width=9, relief="solid", height=1,font=self.fonts[9])

    def update_MENU_frame_items(self):
        #REFRESH MENU FRAME
        self.MENU_frame.place(x=self.screen_width*self.faceplates_x_divisor, y=self.screen_height*self.faceplates_y_divisor*0.6, width=self.screen_width*self.faceplates_width_divisor, height=self.screen_height*self.faceplates_height_divisor)  # Posición en la mitad derecha
        self.menu_frame_width =  self.MENU_frame.winfo_width()  # Ancho del Frame
        self.menu_frame_height = self.MENU_frame.winfo_height()  # Altura del Frame

        #REFRESH ITEMS ON MENU FRAME
        #buttons
        self.comunication_control_button.place(x=self.menu_frame_width*0.05, y=self.menu_frame_height*0.5)
        self.camera_stream_control_button.place(x=self.menu_frame_width*0.25, y=self.menu_frame_height*0.5)
        self.mode_manual_button.place(x=self.menu_frame_width*0.60, y=self.menu_frame_height*0.5)
        self.mode_automatic_button.place(x=self.menu_frame_width*0.78, y=self.menu_frame_height*0.5)
        #labels
        self.menu_TITLE.place(x=self.menu_frame_width*0.35, y=self.menu_frame_height*0.045)
        self.menu_TITLE.config(text="MAIN MENU - [OPTIONS]",width=len("MAIN MENU - [OPTIONS]")+2,bg=self.hex_colors["light_gray"],bd=0, relief="solid", height=1,font=self.fonts[13])


    def update_status_frame_items(self):

        #REFRESH STATUS FRAME
        self.status_frame.place(x=self.screen_width*self.faceplates_x_divisor, y=self.screen_height*0.01, width=self.screen_width*self.faceplates_width_divisor, height=self.screen_height*self.faceplates_height_divisor*0.6)  # Posición en la mitad derecha
        self.status_frame_width =  self.status_frame.winfo_width()  # Ancho del Frame
        self.status_frame_height = self.status_frame.winfo_height()  # Altura del Frame
        #REFRESH STATUS ON MENU FRAME
        STATUS_text = "ROBOT STATUS - [INDICATORS]"
        self.STATUS_TITLE.config(text=STATUS_text,width=len(STATUS_text),bg=self.hex_colors["light_gray"],bd=0, relief="solid", height=1,font=self.fonts[13])
        self.STATUS_TITLE.place(x=self.status_frame_width*0.31, y=self.status_frame_height*0.015)

        com_text = "COM-IP/port"
        self.COMUNICATION_status.config(text=com_text,width=len(com_text))
        self.COMUNICATION_status.place(x=self.status_frame_width*0.05, y=self.status_frame_height*0.45)

        aut_text = "Automatic"
        self.automatic_status.config(text=aut_text,width=len(aut_text)+3)
        self.automatic_status.place(x=self.status_frame_width*0.25, y=self.status_frame_height*0.45)

        manual_text = "Manual"
        self.manual_status.config(text=manual_text,width=len(manual_text)+3)
        self.manual_status.place(x=self.status_frame_width*0.450, y=self.status_frame_height*0.45)

        camera_text = "Camera Signal"
        self.CAMERA_status.config(text=camera_text,width=len(camera_text)+1)
        self.CAMERA_status.place(x=self.status_frame_width*0.62, y=self.status_frame_height*0.45)

        traking_text = "Tracking"
        self.traking_status.config(text=traking_text,width=len(traking_text)+1)
        self.traking_status.place(x=self.status_frame_width*0.84, y=self.status_frame_height*0.45)

    def verify_buttons_status(self):
        #manual button
        if self.bool_variables[0].get():  
            self.mode_manual_button.config(bg=self.hex_colors["white"])
            self.manual_status.config(bg=self.hex_colors["green_medium_light"])
        else:                                         
            self.mode_manual_button.config(bg=self.hex_colors["gray"])
            self.manual_status.config(bg=self.hex_colors["red_medium_light"])

        #automatic button
        if self.bool_variables[1].get():  
            self.mode_automatic_button.config(bg=self.hex_colors["white"])
            self.automatic_status.config(bg=self.hex_colors["green_medium_light"])
        else:                                                   
            self.mode_automatic_button.config(bg=self.hex_colors["gray"])
            self.automatic_status.config(bg=self.hex_colors["red_medium_light"])

    def toggle_size(self, w , h ):
        self.width_divisor = w
        self.height_divisor = h
        #self.geometry(f"{int(self.screen_width*self.width_divisor)}x{int(self.screen_height*self.height_divisor)}")

    def create_folders(self):
        media_path = 'MEDIA'
        saved_path = os.path.join(media_path, 'saved')
        dependencies_path = os.path.join(media_path, 'dependencies')
        color_path = os.path.join(saved_path, 'color_layer')
        depth_path = os.path.join(saved_path, 'depth_layer')
        images_path = os.path.join(saved_path, 'images')
        pointcloud_path = os.path.join(saved_path, 'reconstructed_pointcloud')

        for folder in [media_path, saved_path, color_path, depth_path, images_path, pointcloud_path,dependencies_path]:
            if not os.path.exists(folder):
                os.makedirs(folder)
                print(f"CORRECTED: Folder created: {folder}")
            else:
                print(f"OK: Folder already exists: {folder}")

    def dark_title_bar(self,window):
        """
        MORE INFO:
        https://learn.microsoft.com/en-us/windows/win32/api/dwmapi/ne-dwmapi-dwmwindowattribute
        """
        window.update()
        DWMWA_USE_IMMERSIVE_DARK_MODE = 19
        set_window_attribute = ct.windll.dwmapi.DwmSetWindowAttribute
        get_parent = ct.windll.user32.GetParent
        hwnd = get_parent(window.winfo_id())
        rendering_policy = DWMWA_USE_IMMERSIVE_DARK_MODE
        value = 2
        value = ct.c_int(value)
        set_window_attribute(hwnd, rendering_policy, ct.byref(value),
                            ct.sizeof(value))
        window.update()

    def configure_window_aesthetics(self):
        self.hex_colors = {
            "red_light": "#FFC1C1",
            "red_medium_light": "#FF6B6B",
            "red_medium_dark": "#D32F2F",
            "red_dark": "#9A0007",
            "red": "#FF0000",
            "orange_light": "#FFD8B1",
            "orange_medium_light": "#FFA726",
            "orange_medium_dark": "#FB8C00",
            "orange_dark": "#E65100",
            "orange": "#FFA500",
            "yellow_light": "#FFF9C4",
            "yellow_medium_light": "#FFF176",
            "yellow_medium_dark": "#FBC02D",
            "yellow_dark": "#F57F17",
            "yellow": "#FFFF00",
            "green_light": "#C8E6C9",
            "green_medium_light": "#81C784",
            "green_medium_dark": "#388E3C",
            "green_dark": "#1B5E20",
            "green": "#008000",
            "blue_light": "#BBDEFB",
            "blue_medium_light": "#64B5F6",
            "blue_medium_dark": "#1976D2",
            "blue_dark": "#0D47A1",
            "blue": "#0000FF",
            "indigo_light": "#C5CAE9",
            "indigo_medium_light": "#7986CB",
            "indigo_medium_dark": "#303F9F",
            "indigo_dark": "#1A237E",
            "indigo": "#4B0082",
            "violet_light": "#E1BEE7",
            "violet_medium_light": "#BA68C8",
            "violet_medium_dark": "#8E24AA",
            "violet_dark": "#4A148C",
            "violet": "#EE82EE",
            'black': '#000000',
            'white': '#FFFFFF',
            'gray': '#808080',
            'light_gray': '#D3D3D3',
            'dark_gray': '#A9A9A9',
            'dim_gray': '#696969',
            'slate_gray': '#708090',
            'silver': '#C0C0C0',
        }
        self.fonts = {
            0:("Courier New", 10, "normal"), 
            4:("Courier New", 6, "bold"),
            5:("Courier New", 6, "bold"),
            6:("Courier New", 6, "bold"),
            7:("Courier New", 7, "bold"),       
            8:("Courier New", 8, "bold"), 
            9:("Courier New", 8, "bold"),       
            10:("Courier New", 10, "bold"),       
            11:("Courier New", 11, "bold"),       
            12:("Courier New", 12, "bold"),       
            13:("Courier New", 13, "bold"),            
        }

        #image_url = "https://t4.ftcdn.net/jpg/05/29/91/21/240_F_529912107_MXmORAfxfR7rYDJLHEvRXlwn8gi2vaPm.jpg"  
        image_path = os.path.dirname(os.path.join(os.getcwd(), __file__))+"\\MEDIA\\dependencies\\others"+"\\PURPOSITY_background2.png"#actual_path+"\\bg.jpg"
        '''try:
            resp = urllib.request.urlopen(image_url)
            imagen_bytes = np.asarray(bytearray(resp.read()), dtype="uint8")
            self.img_url = cv2.imdecode(imagen_bytes, cv2.IMREAD_COLOR)
            self.opencv_image = cv2.cvtColor(self.img_url, cv2.COLOR_BGR2RGBA)
        except:'''
        try:
            self.opencv_image = cv2.cvtColor(cv2.imread(image_path), cv2.COLOR_BGR2RGBA)
        except cv2.error:
            #pass
            self.opencv_image = cv2.cvtColor(np.tile(np.array([43, 43, 45], dtype=np.uint8), self.screen_width * self.screen_height).reshape(self.screen_width, self.screen_height, 3), cv2.COLOR_BGR2RGBA)
        #finally:
            #    self.opencv_image = cv2.cvtColor(np.tile(np.array([43, 43, 45], dtype=np.uint8), self.screen_width * self.screen_height).reshape(self.screen_width, self.screen_height, 3), cv2.COLOR_BGR2RGBA)
            #pass
        
        self.geometry(f"{int((self.screen_width*self.faceplates_width_divisor)+(self.screen_width*0.04))}x{int(self.screen_height*self.faceplates_height_divisor+(self.screen_height*0.16))}")
        transparency = 0.65 
        self.opencv_image[:, :, 3] = transparency * self.opencv_image[:, :, 3]
        self.image = Image.fromarray(self.opencv_image)
        self.bg_image = ImageTk.PhotoImage(self.image)
        # Crear una etiqueta para la imagen de fondo
        self.bg_label = tk.Label(self, image=self.bg_image)
        self.bg_label.place(relwidth=1, relheight=1)
        try:
            self.iconbitmap("MEDIA\\dependencies\\others\\PURPOSITY.ico")
        except:
            pass
        finally:
            self.icon_image = cv2.cvtColor(np.tile(np.array([0, 0, 255], dtype=np.uint8), self.screen_width * self.screen_height).reshape(self.screen_width, self.screen_height, 3), cv2.COLOR_BGR2RGBA)
            self.icimage = Image.fromarray(self.icon_image)
            self.bg_icimage = ImageTk.PhotoImage(self.icimage)
            self.iconphoto(True,self.bg_icimage)   
    

    '''
    def resize_bg(self, event):
        #print("EVENTO: ",event)
        #principal window events
        self.screen_width = event.width
        self.screen_height = event.height
        #print(self.screen_width, self.screen_height)
        #//BACKGROUND CONFIS
        resized_image = self.image.resize((self.screen_width, self.screen_height))
        self.bg_image = ImageTk.PhotoImage(resized_image)
        self.bg_label.config(image=self.bg_image)
        #//DIVISION LINES CONFIGS 
        self.update_main_menu_lines()

        #BUTTONS
        self.main_menu_button.place(x=self.screen_width*0.02 , y=self.screen_height*0.095,width=self.screen_width*0.085, height=self.screen_height*0.85)
    
        #FRAMES 
        self.update_status_frame_items() #ALWAYS REFRESHED
        if self.FRAME_STATE == 0: self.update_MENU_frame_items()
        if self.FRAME_STATE == 1: self.update_COM_frame_items()
        if self.FRAME_STATE == 2: self.update_CAMERA_frame_items()
        if self.FRAME_STATE == 3: self.update_MANUAL_frame_items()
    '''
app = GUI(windowName="CLIENT_GUI_PURSPOSITY_ROBOT")
#subprocess.Popen(["python", "slam/depth_camera_slam.py"])
#while True:
#    multiprocessing.Process(target=app.mainloop())
app.mainloop()