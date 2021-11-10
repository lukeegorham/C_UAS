# Imports to get the correct libraries and help the code run
import tkinter as tk
from tkinter import *
from PIL import ImageTk, Image
# import threading
# import socket
# import sys
# from time import sleep, time
# import struct
# import math
# import cv2
# import pickle
# import numpy as np
import struct
# import zlib
# import time
import datetime
import pytz

# from C2_updatedmessageformats import returnToLaunch
# Global variable that indicates whether we have a camera feed for the GUI
# It must be a global because the receive_imagery function is outside the C2GUI class
camera = 1
lat_val = 0
long_val = 0
alt_val = 0
status_t = 0
status_a = 0
rtb_flag = 0
follow_me_flag = 0
cam_flag = 0
cam_count = 0
cam_cont = "auto"
disc_lat = 11
disc_long = 22
disc_alt = 33
#gnd_direction = 0
gnd_direction = ["North", "East", "South", "West"]
map_options = ["Stillman", "Athletic Fields", "Krusty Krab", "Close"]
current_map = "Stillman"
yaw = 0
gnd_view = 0
yaw_view = 0
follow_count = 0
follow_cont = "disabled"
rtb_count = 0
rtb_cont = "disabled"
speed_val = 0
speed_changed = "Not Updated"
speed_count = 0
status_radar = 0
status_acoustic = 0
status_true = 0
toggle_acoustic = 0
acoustic_stat = "Off"
drone_img_processing = 1
ground_img_processing = 1
followme_select = 3
fm_view = "Sensor Fusion"
ground_pic_cont = 0
drone_pic_cont = 0

# This method runs all necessary functions to set up the GUI.
# The rad_array variable is an array of Radar objects (which store data received from radar)
# The old variable stores the index of the oldest entry of radar_data, as the array is circular
# The new variable stores the index of the newest entry of radar_data
def run_main(rad_array, old, new):
    # Array containing camera, radar, discovery drone, and mitigate drone statuses, in that order
    statuses = [1, 1, 1, 1, 1]
    # Array containing if a drone is detected, its speed, and its time to the FOB, in that order
    # When updated, positions two and three (indexes 1 and 2) should contain the actual number for
    # nefarious drone speed and time to FOB. In the case of multiple drones, use the highest speed
    # and lowest time to give the user the worst case scenario.
    info = [-1, -1, -1]
    # Creating a GUI class object
    program = C2GUI(statuses, info, rad_array, old, new)
    # Saving our background image so the program will actually display it
    image = ImageTk.PhotoImage(Image.open("Wideview Background shortened no labels.PNG")) # This line doesn't actually load the picture. Check line 198 or search "photo" - Max
    #image = ImageTk.PhotoImage(Image.open("krusty krab.PNG")) # This line doesn't actually load the picture. Check line 198 or search "photo" - Max
    # Updating what the GUI should look like
    program.runGUI()
    # Displaying an image from the discovery drone
    # receive_imagery()
    # Refreshing the screen to display the updates given by runGUI
    program.window.update()
    # Returning the C2GUI object to the C2 code so it works with the same GUI object as it runs
    return program


# This class stores a GUI object, as well as all the data necessary to make the GUI display properly.
# The class also updates the information as needed based on inputs from C2.
class C2GUI:
    # This method creates a new GUI object and initializes the variables that are needed to run it
    # (Most variables are set to -1, 0, or 1 by default based on the context of the variable)
    def __init__(self, stats, inf, rad_array, old, new):
        # Creating a Tkinter (GUI) oject
        self.window = tk.Tk()
        # Titling the GUI
        self.window.title("C2 User Interface")
        # Initializing class arrays with info from main
        self.status = stats
        self.info = inf
        # # Setting the discovery and mitigation as initially inactive (0)
        # self.disc_active = 0
        # self.mit_active = 0
        # # Determining the initial text for the buttons
        # self.disc_text = "Launch discovery"
        # self.mit_text = "Launch mitigation"
        # # Storing variables in a new class for time calculations
        self.log = rad_array
        # Implementing a circular array to store the data
        # Size (depth) of the circular array)
        self.logDepth = 20
        # Index of the most recent entry
        self.new = new
        # Index of the oldest entry
        self.old = old
        # Index of the most recent drone data pointer
        self.drone_new = 0
        # Boolean that tells if the array has been filled all the way or not
        self.looped = 0
        # Array that holds acoustic data
        self.pods = []
        # Creating widgets to go on the GUI

        # All of the Dictionaries being passed from C2
        self.acoustic_targets = {}  # array that stores infromation from the

        self.true_UAV_mess = {}  # array that stores true_UAV position information

        self.radar_asterix_48 = {}  # a dictionary containing infromation from the Type 48 Asterix Messages

        self.radar_asterix_34 = {}  # a dictionary containing the type 34 Asterix messages

        self.DiscoveryDrone = {}  # a dictionationary containing the information from the discovery drone

        self.t_breadcrumb = {}  # a Dictionary containing breadcrumb information

        self.a_breadcrumb = {}

        self.radarEstimate = {}  # Dictionary containing sensor fusion values from C2

        self.DiscoveryDroneoffset = {}

        self.create_widgets()



    # This method creates the widgets (display boxes, buttons, etc.) that are displayed on the GUI
    # This only needs to run once on setup for the GUI

    def print_hello(self, g):
        x = 2 + g
        print(x)

    def create_widgets(self):
        global current_map
        # # Creating the frame for asset statuses
        # self.top_frame = tk.LabelFrame(self.window, text="Asset Statuses")
        # self.window.rowconfigure(1, weight=1)
        #
        # # Creating the label that displays the status of the camera
        # self.cam = tk.Label(self.top_frame, text="Camera", bg=self.determine_color(0))
        # # Creating the label that displays the status of the radar
        # self.rad = tk.Label(self.top_frame, text="Radar", bg=self.determine_color(1))
        # # Creating the label that displays the status of the discovery drone
        # self.disc = tk.Label(self.top_frame, text="Discovery Drone", bg=self.determine_color(2))
        # # Creating the label that displays the status of the mitigate drone
        # self.mit = tk.Label(self.top_frame, text="Mitigate Drone", bg=self.determine_color(3))
        # # Creating the label that displays the status of the acoustics
        # self.acoustics = tk.Label(self.top_frame, text="Acoustics", bg=self.determine_color(4))
        #
        # # Creating the frame for event statuses (drone detection)
        # self.left_frame = tk.LabelFrame(self.window, text="Event Statuses")
        # self.window.rowconfigure(2, weight=1)
        #
        # # Creating the label that displays whether a drone has been detected
        # self.detect = tk.Label(self.left_frame, text=self.determine_text(0)[0], bg=self.determine_text(0)[1])
        # # Creating the label that displays the speed of the incoming drone
        # self.speed = tk.Label(self.left_frame, text="Incoming drone speed: "+self.determine_text(1)[0]+" m/s", bg=self.determine_text(1)[1])
        # # Creating the label that displays the time it will take for the drone to get to the FOB
        # self.time = tk.Label(self.left_frame, text="Incoming drone time to FOB: "+self.determine_text(2)[0]+" sec", bg=self.determine_text(2)[1])
        self.bottom_frame = tk.Frame(self.window)
        # self.enter_waypoint = tk.Button(self.bottom_frame, text="Enter Waypoint", command=self.waypoint)
        #
        # # Creating the frame for the control buttons

        map_select = tk.StringVar(self.window)
        map_select.set(map_options[0])

        option_var = tk.StringVar(self.window)
        option_var.set(gnd_direction[0])


        # Creating the button that launches and pauses/unpauses the discovery drone
        # self.disc_button = tk.Button(self.bottom_frame, text=self.disc_text, command=self.pause_disc)
        # Creating the button that launches and pauses/unpauses the mitigate drone
        # self.mit_button = tk.Button(self.bottom_frame, text=self.mit_text, command=self.pause_mit)
        # Creating the button that returns the drones to base
        # self.rtb_button = tk.Button(self.bottom_frame, text="Return to Base", command=self.return_base)
        # Creating the button that launches both drones
        # self.launch_button = tk.Button(self.bottom_frame, text="Select Waypoint", command=self.returnToLaunch)
        # Creating the button that allows the mitigate drone to mitigate the threat
        # self.mitigate_button = tk.Button(self.bottom_frame, text="Mitigate Threat", command=self.mitigate)

        # Creating the frame for the map
        self.mapFrame = tk.Frame(self.window)
        # Opening the background image
        photo = Image.open("Wideview Background shortened no labels.PNG")
        #photo = Image.open("krusty krab.png") # test for changing imaged - finally works - Max

        # Determining the size of the photo for future reference
        self.img_width, self.img_height = photo.size
        # Creating the canvas to display the map and locations of the drones, radar, and acoustic pods
        self.plot = tk.Canvas(self.mapFrame, height=self.img_height + 1, width=self.img_width + 1)
        # Packing the plot (allows us to display it)
        self.plot.pack()
        # Saving the image as several different tkinter objects so garbage collection doesn't delete the photo and it actually displays
        self.img = ImageTk.PhotoImage(photo)
        image_label = tk.Label(image=self.img)
        image_label.image = self.img
        # Displaying the background image
        #pic = self.plot.create_image(0, 0, anchor=NW, image=self.img)

        self.enter_waypoint = tk.Button(text="Enter Waypoint", command=self.waypoint)
        self.enter_gnd_loc = tk.Button(text="Ground Station Location", command=self.gnd_loc)
        self.health_status = tk.Button(text="Subsystem Health Status", command=1)
        self.rtb = tk.Button(text="Return to Base", command=self.rtb_control)
        self.follow_me = tk.Button(text="Follow-Me", command=self.follow_me_control)
        self.gnd_cam_control = tk.Button(text="Ground Camera", command=self.cam_control)
        self.speed = Entry()
        self.speed_title = Label(text="Discovery Drone Speed Input")
        self.speed_button = tk.Button(text="Update Speed", command=self.save_speed)
        # self.cur_pos = Label(
        # text=f"Current Latitude: {lat_val}     Current Longitude: {long_val}       Current Altitude: {alt_val}")
        self.true_breadcrumb = tk.Checkbutton(text="True UAV Breadcrumb", command=self.toggle_true_breadcrumb)
        self.pod_breadcrumb = tk.Checkbutton(text="Acoustic Breadcrumb", command=self.toggle_acoustic_breadcrumb)
        self.acoustic_dot = tk.Checkbutton(text="Acoustic Position", command=self.toggle_acoustic_dot)
        self.radar_dot = tk.Checkbutton(text="Radar Position", command=self.toggle_radar_dot)
        self.true_dot = tk.Checkbutton(text="True Position", command=self.toggle_true_dot)
        self.zoom = tk.Scale(label="Zoom", orient=HORIZONTAL, length=200)

        #self.R1 = Radiobutton(text="North", variable=gnd_direction, value=1)
        #self.R2 = Radiobutton(text="East", variable=gnd_direction, value=2)
        #self.R3 = Radiobutton(text="South", variable=gnd_direction, value=3)
        #self.R4 = Radiobutton(text="West", variable=gnd_direction, value=4)

        self.Y1 = Radiobutton(text="0 deg", variable=yaw, value=5, command=self.yaw1)
        self.Y2 = Radiobutton(text="90 deg", variable=yaw, value=6, command=self.yaw2)
        self.Y3 = Radiobutton(text="180 deg", variable=yaw, value=7, command=self.yaw3)
        self.Y4 = Radiobutton(text="270 deg", variable=yaw, value=8, command=self.yaw4)

        self.fusion_acoustic = tk.Button(text="Acoustic Sensor Fusion", command=self.acoustic_msg)
        self.ground_processing = tk.Checkbutton(text="Ground Camera Image Processing", command=self.image_ground)
        self.ground_processing.select()
        self.drone_processing = tk.Checkbutton(text="Drone Camera Image Processing", command=self.image_drone)
        self.drone_processing.select()
        self.image_processing = Label(text="Image Processing")
        self.fm1 = Radiobutton(text="True UAV", variable=followme_select, value=9, command=self.fm_sel1)
        self.fm2 = Radiobutton(text="Radar", variable=followme_select, value=10, command=self.fm_sel2)
        self.fm3 = Radiobutton(text="Sensor Fusion", variable=followme_select, value=11, command=self.fm_sel3)
        self.pic_drone = tk.Checkbutton(text="Discovery Drone", command=self.drone_pic)
        self.pic_ground = tk.Checkbutton(text="Ground Camera", command=self.ground_pic)
        self.take_pic_cont = Label(text="Save Images")

        #self.Stillman_map = Radiobutton(text="Stillman Field", variable=map_select, value=12, command=self.Stillman_or_Athletics)
        #self.Athletic_fields_map = Radiobutton(text="Athletic Fields", variable=map_select, value=13, command=self.Stillman_or_Athletics)
        #self.Krusty_Krab_map = Radiobutton(text="Krusty Krab", variable=map_select, value=14, command=self.krusty_Krab)
        #for maps, map_val in map_options:
            #self.map_buttons = tk.Radiobutton(self.window, text=maps, variable=map_select, value=map_val, command=self.Stillman_or_Athletics(map_select))

        self.map_buttons = tk.OptionMenu(self.window, map_select, *map_options, command=self.Stillman_or_Athletics)
        #current_map = map_select.get()
        self.camera_direction = tk.OptionMenu(self.window, option_var, *gnd_direction,)


    # This method updates the actual display elements of the GUI based on inputs from the C2 code
    # The method does not actually receive the updates from C2, but rather simply looks at the object
    # memory locations (self.info, self.pods, etc.) to update the GUI accordingly
    def runGUI(self):
        global current_map
        # Updating status array from a global variable so we know if the camera is sending or not
        # This ensures the asset status sidebar is accurate
        self.status[0] = camera

        # Drawing the top frame (asset statuses)
        # self.top_frame.grid(row=1, column=1, pady=10, padx=10, sticky=tk.W)
        # # Drawing the camera status box and ensuring it is the right color
        # self.cam.grid(row=2, column=1, pady=10, padx=10, columnspan=2, sticky=tk.W + tk.E)
        # self.cam.configure(bg=self.determine_color(0))
        # # Drawing the radar status box and ensuring it is the right color
        # self.rad.grid(row=3, column=1, pady=10, padx=10, columnspan=2, sticky=tk.W + tk.E)
        # self.rad.configure(bg=self.determine_color(1))
        # # Drawing the discovery drone status box and ensuring it is the right color
        # self.disc.grid(row=4, column=1, pady=10, padx=10, columnspan=2, sticky=tk.W + tk.E)
        # self.disc.configure(bg=self.determine_color(2))
        # # Drawing the mitigate drone status box and ensuring it is the right color
        # self.mit.grid(row=5, column=1, pady=10, padx=10, columnspan=2, sticky=tk.W + tk.E)
        # self.mit.configure(bg=self.determine_color(3))
        # # Drawing the acoustic status box and ensuring it is the right color
        # self.acoustics.grid(row=6, column=1, pady=10, padx=10, columnspan=2, sticky=tk.W + tk.E)
        # self.acoustics.configure(bg=self.determine_color(4))
        #
        # # Drawing the left frame (event statuses)
        # self.left_frame.grid(row=2, column=1, pady=10, padx=10, sticky=tk.W)
        # # Drawing the drone detected box and ensuring it is the right color and displays the right text
        # self.detect.grid(row=2, column=1, pady=10, padx=10, sticky=tk.W)
        # self.detect.configure(text=self.determine_text(0)[0], bg=self.determine_text(0)[1])
        # # Drawing the drone speed box and ensuring it is the right color and displays the right text
        # self.speed.grid(row=3, column=1, pady=10, padx=10, sticky=tk.W)
        # self.speed.configure(text="Incoming drone speed: "+self.determine_text(1)[0]+" m/s", bg=self.determine_text(1)[1])
        # # Drawing the drone time to FOB box and ensuring it is the right color and displays the right text
        # self.time.grid(row=4, column=1, pady=10, padx=10, sticky=tk.W)
        # self.time.configure(text="Incoming drone time to FOB: "+self.determine_text(2)[0]+" sec", bg=self.determine_text(2)[1])
        #
        # # Drawing the bottom frame (control buttons)
        # self.bottom_frame.grid(row=3, column=1, pady=10, padx=10, sticky=tk.W)
        # Drawing the launch discovery button and ensuring the displayed text is correct
        # self.disc_button.grid(row=1, column=1, padx=5, pady=5, sticky=tk.W)
        # self.disc_button.configure(text=self.disc_text)
        # Drawing the launch mitigation button and ensuring the displayed text is correct
        # self.mit_button.grid(row=1, column=2, padx=5, pady=5, sticky=tk.E)
        # self.mit_button.configure(text=self.mit_text)
        # Drawing the return to base button
        # self.rtb_button.grid(row=2, column=1, columnspan=2, padx=5, pady=5, sticky=tk.E+tk.W)
        # Drawing the launch (both) drones button
        # self.launch_button.grid(row=3, column=1, columnspan=2, padx=5, pady=5, sticky=tk.E + tk.W)
        # Drawing the mitigate threat button
        # self.mitigate_button.grid(row=4, column=1, columnspan=2, padx=5, pady=5, sticky=tk.E + tk.W)
        # self.enter_waypoint.grid(row=5, column=1, columnspan=2, padx=5, pady=5, sticky=tk.E + tk.W)
        self.enter_waypoint.place(x=1365, y=46)
        # self.health_status.grid(row=2, column=4, columnspan=1, padx=5, pady=5)
        self.health_status.place(x=1365, y=6)
        self.rtb.place(x=1365, y=86)
        self.follow_me.place(x=1365, y=126)
        # self.cur_pos.place(x=300, y=670)
        self.true_breadcrumb.place(x=910, y=670)
        self.pod_breadcrumb.place(x=1060, y=670)
        self.zoom.place(x=170, y=660)
        self.gnd_cam_control.place(x=1365, y=166)
        self.enter_gnd_loc.place(x=1365, y=206)
        #self.R1.place(x=1365, y=406)
        #self.R2.place(x=1365, y=446)
        #self.R3.place(x=1365, y=486)
        #self.R4.place(x=1365, y=526)
        self.Y1.place(x=1365, y=606)
        self.Y2.place(x=1365, y=646)
        self.Y3.place(x=1365, y=686)
        self.Y4.place(x=1365, y=726)
        self.speed.place(x=170, y=760)
        self.speed_title.place(x=170, y=730)
        self.speed_button.place(x=300, y=755)
        self.acoustic_dot.place(x=1060, y=710)
        self.radar_dot.place(x=1210, y=710)
        self.true_dot.place(x=910, y=710)
        self.fusion_acoustic.place(x=1365, y=246)
        self.ground_processing.place(x=900, y=765)
        self.drone_processing.place(x=1150, y=765)
        self.image_processing.place(x=1070, y=745)
        self.fm1.place(x=1365, y=276)
        self.fm2.place(x=1365, y=316)
        self.fm3.place(x=1365, y=356)
        self.pic_drone.place(x=520, y=760)
        self.pic_ground.place(x=650, y=760)
        self.take_pic_cont.place(x=605, y=740)

        self.map_buttons.place(x=20, y=680)
        #self.Athletic_fields_map.place(x=20, y=720)
        #self.Krusty_Krab_map.place(x=20, y=760)
        self.camera_direction.place(x=1365, y=406)

        # Getting the width and height of the background (parade field) image
        width = self.img_width
        height = self.img_height
        # Drawing the background picture
        pic = self.plot.create_image(0, 0, anchor=NW, image=self.img)
        # Ensuring there is at least one data entry in the radar log variable
        # before attempting to read the data and display it
        if len(self.log) >= 1:
            # Calculating radar x and y map coordinates
            radar_x, radar_y = normalize_locs(self.log[self.new].radarLong_deg, self.log[self.new].radarLat_deg, width,
                                              height, current_map)
            # Drawing the radar on the map
            rad = self.plot.create_rectangle(radar_x - 10, radar_y - 10, radar_x + 10, radar_y + 10, fill="#fcf51e")
            # Updating the variable that controls the asset status sidebar so we know the radar is sending data
            self.status[1] = 1

        # Positioning the map on the overall GUI
        self.mapFrame.grid(row=1, column=3, rowspan=3, pady=5, padx=5, sticky=tk.W)
        # Drawing the acoustic pod on the GUI map
        self.draw_acoustics()
        # self.draw_True_UAV_Pos()
        # self.draw_acoustic_targets()
        # self.draw_Radar_Position()
        self.draw_radar_34()
        self.draw_DiscoveryDrone_Pos()
        self.status_view()
        self.cur_pos_lat = Label(text=f"Current Latitude: {round(disc_lat, 6)}")
        self.cur_pos_long = Label(text=f"Current Longitude: {round(disc_long, 6)}")
        self.cur_pos_alt = Label(text=f"Current Altitude: {disc_alt}")
        self.cam_stat = Label(text=f"{cam_cont}")
        self.follow_stat = Label(text=f"{follow_cont}")
        self.rtb_stat = Label(text=f"{rtb_cont}")
        # print(f"status_t = {status_t}")
        # print(f"rtb_flag = {rtb_flag}")
        self.gnd_stat = Label(text=f"Camera Direction: {gnd_view}")
        self.yaw_stat = Label(text=f"yaw selelcted: {yaw_view}")
        self.speed_up_stat = Label(text=f"{speed_changed}  {speed_count}")
        self.acoustic_view = Label(text=f"{acoustic_stat}")
        self.fm_stat = Label(text=f"Follow-me Mode: {fm_view}")
        self.cur_pos_lat.place(x=400, y=670)
        self.cur_pos_long.place(x=570, y=670)
        self.cur_pos_alt.place(x=760, y=670)
        self.cam_stat.place(x=1475, y=168)
        self.follow_stat.place(x=1475, y=128)
        self.rtb_stat.place(x=1475, y=88)
        self.gnd_stat.place(x=1365, y=566)
        self.yaw_stat.place(x=1365, y=766)
        self.speed_up_stat.place(x=395, y=760)
        self.acoustic_view.place(x=1500, y=248)
        self.fm_stat.place(x=1362, y=381)
        if status_t == 1:
            self.draw_t_breadcrumb()
        # print(f"status_a = {status_a}")
        if status_a == 1:
            self.draw_a_breadcrumb()
        if status_true == 1:
            self.draw_True_UAV_Pos()
        if status_acoustic == 1:
            self.draw_acoustic_targets()
        if status_radar == 1:
            self.draw_Radar_Position()
        self.draw_radarEstimate()
        self.draw_DiscoveryDroneoffset()

        x_test, y_test = normalize_locs(-104.887928, 39.015772, self.img_width,
                                              self.img_height, current_map)
        podtest2 = self.plot.create_polygon([x_test+5, y_test+5, x_test-5, y_test+5, x_test-5, y_test-5, x_test+5, y_test-5], fill="red")
        self.plot.tag_raise(podtest2)

        # self.draw_breadcrumb()
        # Receiving and displaying images received from the discovery drone
        # (This might need to eventually be put into the C2 code instead)
        # receive_imagery()

    # Function that determines the color of the asset status boxes given the state of the status array
    # Box is the index of the appropriate item (0 for camera, 1 for radar, etc.)
    def determine_color(self, box):
        # If the asset is good (communicating with the GUI/C2)
        if self.status[box] == 1:
            # Returning the color cyan (light blue)
            return "#00ffff"
        # If the asset is not good (not communicating with the GUI/C2)
        else:
            # Returning a light red color
            return "#ff5e5e"

    # Method that controls the color and text of the event status boxes
    # Box is the index of each event status (o for drone detected, 1 for drone speed, and 2 for time to fob)
    # Returns and array where index 0 contains an appropriate string to display/incorporate into the text already
    #   being displayed and index 1 is a string containing the color of the box
    def determine_text(self, box):
        # Displaying unique text for box 1 in either case
        if box == 0:
            # If no drone is detected
            if self.info[box] < 0:
                return ["No drone detected", "light green"]
            # If a drone is detected (The hexadecimal indicates a light red color)
            else:
                return ["Drone detected", "#ff5e5e"]
        # Displaying the drone speed/time to fob of the other two boxes
        else:
            # If no drone is detected
            if self.info[box] < 0:
                return ["N/A", "light green"]
            # If a drone is detected (The hexadecimal indicates a light red color))
            else:
                return [str(self.info[box]), "#ff5e5e"]

    # This method draws the acoustic pods at the appropriate location on the GUI map
    # (You might want to field test this though, I'm not sure it 100% works in the field with real acoustic data)
    def draw_acoustics(self):
        # Ensuring there is actual acoustic data to display to prevent an error
        if len(self.pods) > 0:
            # Indicating acoustics are good/sending data (1) in the asset status panel
            self.status[4] = 1
            # Looping through the acoustic pod entries
            current_time = datetime.datetime.now(tz=pytz.utc).timestamp()
            try:
                for jj in self.pods:
                    # print(jj)
                    # Getting the recently calculated and stored x and y coordinates of the acoustic pod
                    j = self.pods[jj]
                    acoustic_x = j.grid_x
                    acoustic_y = j.grid_y
                    # If the pod hears a drone (AKA, this is a simulated Acoustic Target Message)
                    if (j.tgt_active == True):  # If a UAV has been detected
                        # Turn the pod blue on the display
                        color = "blue"

                    # If the pod hears anything else (not a drone)
                    # computer_time_target = time.clock() #gets the time when the target was detect
                    # computer_time_target = j.time_stamp
                    # int_computer_time_target = int(computer_time_target)
                    # print(int_computer_time_target)
                    else:
                        # Turn the pod light green
                        color = "red"
                    # computer_time_method = time.clock()
                    # normal_time = int(computer_time_method)
                    # print(normal_time)
                    # if (normal_time - int_computer_time_target) > 2:
                    #     color = 'red'
                    # Calculate the points necessary to create a triangle
                    time_elapsed = current_time - j.time_last_target_message
                    # print(j.time_last_target_message)
                    # print(time_elapsed)
                    if time_elapsed > 2:
                        j.tgt_active = False  # indicates that the UAV has left the range of the Acoustic Pod
                    points = [acoustic_x, acoustic_y - 8, acoustic_x - 12, acoustic_y + 12, acoustic_x + 12,
                              acoustic_y + 12]
                    # Actually draw the triangle on the GUI map
                    pod = self.plot.create_polygon(points, fill=color)
                    # Bring the acoustic pod triangle in front of the GUI map image
                    self.plot.tag_raise(pod)
                    # Finding the points to draw a circle around the pod and indicate its range
                    # (You should probably change this to be more accurate or possibly even dynamically allocated)
                    ac_rad = self.plot.create_oval(acoustic_x - 70, acoustic_y - 70, acoustic_x + 70, acoustic_y + 70,
                                                   outline=color)
                    # Bring the acoustic pod circle in front of the GUI map image
                    self.plot.tag_raise(ac_rad)
            except:
                print("Acoustic Pod Problem")
        # If there are no entries in the acoustic pod array from C2
        else:
            # Indicating the acoustics are bad/not sending data (-1) in the asset status panel
            self.status[4] = -1

    # This method updates the objects in the acoustic array with the x and y locations of each pod on the GUI map
    def update_acoustics(self):
        # Number of acoustic pods
        num_pods = len(self.pods)
        # Getting locations for acoustic pods
        for m in self.pods:
            try:
                # Calculating the normalized acoustic locations in x and y coordinates on the GUI map
                #temp_ac_x, temp_ac_y = normalize_locs(self.pods[m].pod_long, self.pods[m].pod_lat, self.img_width,
                                                      #self.img_height, current_map)
                temp_ac_x, temp_ac_y = normalize_locs(self.pods[m].pod_long, self.pods[m].pod_lat, self.img_width,
                                                      self.img_height, current_map)
                # Storing the x and y locations in the actual acoustic objects
                self.pods[m].update_grid(temp_ac_x, temp_ac_y)
            except:
                print("Error in Acoustic Pod Dictionary")

    # Display Target Pod Estimator Information on GUI
    def draw_acoustic_targets(self):
        current_estimator_time = datetime.datetime.now(tz=pytz.utc).timestamp()
        if len(self.acoustic_targets) > 0:
            for jj in self.acoustic_targets:
                try:
                    j = self.acoustic_targets[jj]
                    acousticx = j.grid_x_tgt
                    acousticy = j.grid_y_tgt
                    color = 'orange'
                    estimator_timestamp = current_estimator_time - j.time_last_message
                except:
                    print("No Pod Target Information Present")
            if estimator_timestamp < 1:
                target_shape = self.plot.create_oval(acousticx - 10, acousticy - 10, acousticx + 10, acousticy + 10,
                                                     outline=color, fill=color)
                self.plot.tag_raise(target_shape)

    def update_acoustic_targets(self):
        for jj in self.acoustic_targets:
            try:
                j = self.acoustic_targets[jj]
                temp_target_x, temp_target_y = normalize_locs(self.acoustic_targets[jj].est_tgt_long,
                                                              self.acoustic_targets[jj].est_tgt_lat, self.img_width,
                                                              self.img_height, current_map)  # temporary x and y location of
                # target from a singular pod message
                self.acoustic_targets[jj].update_grid(temp_target_x, temp_target_y)
            except:
                print("Error in the Acoustic Targets Dictionary")

    # Display True UAV information on GUI
    def draw_True_UAV_Pos(self):
        if len(self.true_UAV_mess) > 0:
            try:
                for ii in self.true_UAV_mess:
                    i = self.true_UAV_mess[ii]
                    truex = i.grid_x
                    truey = i.grid_y
                    color = 'purple'
                    UAV_shape = self.plot.create_oval(truex - 10, truey - 10, truex + 10, truey + 10, outline=color,
                                                      fill=color)
                    self.plot.tag_raise(UAV_shape)  # pot the True UAV position
            except:
                print("Error in True UAV Dictionary")

    def update_True_UAV(self):
        for ii in self.true_UAV_mess:
            global disc_lat
            disc_lat = self.true_UAV_mess[ii].UAV_lat
            global disc_long
            disc_long = self.true_UAV_mess[ii].UAV_long
            global disc_alt
            disc_alt = self.true_UAV_mess[ii].UAV_alt
            try:
                temp_UAV_x, temp_UAV_y = normalize_locs(self.true_UAV_mess[ii].UAV_long, self.true_UAV_mess[ii].UAV_lat,
                                                        self.img_width, self.img_height, current_map)  # temporary x ad y location of
                self.true_UAV_mess[ii].update_grid(temp_UAV_x, temp_UAV_y)
            except:
                print('Error in the True UAV Dictionary')

    # Display of Type 48 information on GUI
    def draw_Radar_Position(self):
        currentRadartime = datetime.datetime.now(tz=pytz.utc).timestamp()
        try:
            for tt in self.radar_asterix_48:
                t = self.radar_asterix_48[tt]
                # print(int(tt.radar_lat))
                radar_timestamp = currentRadartime - t.last_radar_message
                # print(long_correct)
                radarx = t.grid_x_48
                radary = t.grid_y_48
                color = 'blue'
                if radar_timestamp < 0.5:
                    UAV_shape = self.plot.create_oval(radarx - 10, radary - 10, radarx + 10, radary + 10, outline=color,
                                                      fill=color)
                    self.plot.tag_raise(UAV_shape)
        except:
            print("Problem with the Asterix 48 Dictionary")

    def update_radar_48(self):
        form = '>i'  # signed 32 byte integer
        for tt in self.radar_asterix_48:
            try:
                long = struct.unpack(form,
                                     self.radar_asterix_48[tt].radar_long)  # unpacks the bytes into an integer value
                long_correct = long[0] / (1e5)
                # print(long_correct)
                lat = struct.unpack(form,
                                    self.radar_asterix_48[tt].radar_lat)  # unpacks the bytes into an integer value
                lat_correct = lat[0] / (1e5)
                # print(lat_correct)
                temp_radar_x, temp_radar_y = normalize_locs(long_correct, lat_correct,
                                                            self.img_width,
                                                            self.img_height, current_map)  # temporary x ad y location of
                self.radar_asterix_48[tt].update_grid(temp_radar_x, temp_radar_y)
            except:
                print("Error in RADAR 48 Dictionary")
                # write the error and the timestamp/information to

    # Display of Type 34 Information on GUI
    def draw_radar_34(self):
        for rr in self.radar_asterix_34:
            try:
                r = self.radar_asterix_34[rr]
                # print(int(tt.radar_lat))

                # print(long_correct)
                radarx = r.grid_x_34
                radary = r.grid_y_34
                color = 'red'
                Radar_Station_Shape = self.plot.create_rectangle(radarx - 10, radary - 10, radarx + 10, radary + 10,
                                                                 outline=color, fill=color)
                self.plot.tag_raise(Radar_Station_Shape)
            except:
                print("Error in Asterix 48 Dictionary")

    def update_radar_34(self):
        form = '>BBB'
        for rr in self.radar_asterix_34:
            try:
                r = self.radar_asterix_34[rr]
                long_34 = int.from_bytes(self.radar_asterix_34[rr].source_long, "big", signed=False)
                correct_long_34 = (long_34 * (180 / (2 ** 23))) - 360
                # long_34 = struct.unpack(form, self.radar_asterix_34[rr].source_long)
                # print(correct_long_34)
                lat_34 = int.from_bytes(self.radar_asterix_34[rr].source_lat, "big", signed=False)
                correct_lat_34 = lat_34 * (180 / (2 ** 23))
                # print(correct_lat_34)
                temp_radar_x, temp_radar_y = normalize_locs(correct_long_34, correct_lat_34, self.img_width,
                                                            self.img_height, current_map)  # temporary x and y location of
                # positioning of the radar station from a type 34 message
                self.radar_asterix_34[rr].update_grid(temp_radar_x, temp_radar_y)
            except:
                print('Error in the RADAR 34 Dictionary')

    # Display True UAV information on GUI
    def draw_radarEstimate(self):
        current_estimateTime = datetime.datetime.now(tz=pytz.utc).timestamp()
        if len(self.radarEstimate) > 0:
            for ii in self.radarEstimate:
                i = self.radarEstimate[ii]
                truex = i.grid_x_sensor
                truey = i.grid_y_sensor
            color = 'pink'
            estimate_timestamp = current_estimateTime - i.sensorTime
            if estimate_timestamp < 0.5:
                radarEst_shape = self.plot.create_oval(truex - 10, truey - 10, truex + 10, truey + 10,
                                                       outline=color,
                                                       fill=color)
                self.plot.tag_raise(radarEst_shape)  # plot the Estimated UAV position

    def update_radarEstimate(self):
        for ii in self.radarEstimate:
            # if self.radarEstimate[ii].filterExpiration == False:
            try:
                temp_UAV_x, temp_UAV_y = normalize_locs(self.radarEstimate[ii].long_est,
                                                        self.radarEstimate[ii].lat_est, self.img_width,
                                                        self.img_height, current_map)  # temporary x ad y location of
                self.radarEstimate[ii].update_grid(temp_UAV_x, temp_UAV_y)
            except:
                print('Error in the Radar Estimate Dictionary')

    # Section for plotting the Discovery Drone
    def draw_DiscoveryDrone_Pos(self):
        if len(self.DiscoveryDrone) > 0:
            for ii in self.DiscoveryDrone:
                i = self.DiscoveryDrone[ii]
                truex = i.grid_x
                truey = i.grid_y
            color = 'yellow'
            Disc_shape = self.plot.create_oval(truex - 10, truey - 10, truex + 10, truey + 10, outline=color,
                                               fill=color)
            self.plot.tag_raise(Disc_shape)  # pot the True UAV position

    # def draw_breadcrumb(self):
    #     if(len(self.breadcrumb) > 0):
    #         for zz in self.breadcrumb:
    #             z = self.breadcrumb[zz]
    #             n = len(z.posBuffer)
    #             x_hist = []
    #             y_hist = []
    #             x, y = normalize_locs(z.posBuffer[n].long, z.posBuffer[n].lat, self.img_width, self.img_height)
    #             x_hist.append(x)
    #             y_hist.append(y)
    #
    #             color = 'black'
    #             if n > 5:
    #                 breadcrumb = self.plot.create_line(x_hist[n-1], y_hist[n-1], x_hist[n-2], y_hist[n-2], x_hist[n-3], y_hist[n-3], x_hist[n-4], y_hist[n-4], x_hist[n-5], y_hist[n-5], outline=color, dash=(5, 2), width=5)
    #                 self.plot.tag_raise(breadcrumb)

    # def draw_breadcrumb(self):
    #     if (len(self.breadcrumb) > 0):
    #         for zz in self.breadcrumb:
    #             z = self.breadcrumb[zz]
    #             n = len(z.posBuffer)
    #             x_hist = []
    #             y_hist = []
    #             for k in range(n):
    #                 x, y = normalize_locs(z.posBuffer[k].long, z.posBuffer[k].lat, self.img_width, self.img_height)
    #                 x_hist.append(x)
    #                 y_hist.append(y)
    #
    #             color = 'purple'
    #             n = len(z.posBuffer)
    #             if n > 5:
    #                 breadcrumb = self.plot.create_line(x_hist[n - 1], y_hist[n - 1], x_hist[n - 2], y_hist[n - 2],
    #                                                    x_hist[n - 3], y_hist[n - 3], x_hist[n - 4], y_hist[n - 4],
    #                                                    x_hist[n - 5], y_hist[n - 5], x_hist[n - 6], y_hist[n - 6],
    #                                                    x_hist[n - 7], y_hist[n - 7], x_hist[n - 8], y_hist[n - 8],
    #                                                    x_hist[n - 9], y_hist[n - 9], x_hist[n - 10], y_hist[n - 10],
    #                                                    fill=color, dash=(5, 2),
    #                                                    width=5)
    #                 self.plot.tag_raise(breadcrumb)
    # def update_breadcrumb(self):
    #     for zz in self.breadcrumb:
    #         temp_UAV_x, temp_UAV_y = normalize_locs(self.breadcrumb[zz].UAV_long, self.breadcrumb[zz].UAV_lat,
    #                                                 self.img_width, self.img_height)  # temporary x ad y location of
    #         self.breadcrumb[zz].update_grid(temp_UAV_x, temp_UAV_y)

    def draw_t_breadcrumb(self):
        if len(self.t_breadcrumb) > 0:
            for zz in self.t_breadcrumb:
                z = self.t_breadcrumb[zz]
                n = len(z.posBuffer)
                x_hist = []
                y_hist = []
                for k in range(n):
                    x, y = normalize_locs(z.posBuffer[k].long, z.posBuffer[k].lat, self.img_width, self.img_height, current_map)
                    x_hist.append(x)
                    y_hist.append(y)

                color = 'purple'
                n = len(z.posBuffer)
                if n > 5:
                    t_breadcrumb = self.plot.create_line(x_hist[n - 1], y_hist[n - 1], x_hist[n - 2], y_hist[n - 2],
                                                         x_hist[n - 3], y_hist[n - 3], x_hist[n - 4], y_hist[n - 4],
                                                         x_hist[n - 5], y_hist[n - 5], x_hist[n - 6], y_hist[n - 6],
                                                         x_hist[n - 7], y_hist[n - 7], x_hist[n - 8], y_hist[n - 8],
                                                         x_hist[n - 9], y_hist[n - 9], x_hist[n - 10], y_hist[n - 10],
                                                         fill=color, dash=(5, 2),
                                                         width=5)
                    self.plot.tag_raise(t_breadcrumb)

    def update_t_breadcrumb(self):
        for zz in self.t_breadcrumb:
            temp_UAV_x, temp_UAV_y = normalize_locs(self.t_breadcrumb[zz].UAV_long, self.t_breadcrumb[zz].UAV_lat,
                                                    self.img_width, self.img_height, current_map)  # temporary x ad y location of
            self.t_breadcrumb[zz].update_grid(temp_UAV_x, temp_UAV_y)

    def draw_a_breadcrumb(self):
        if len(self.a_breadcrumb) > 0:
            current_estimator_time = datetime.datetime.now(tz=pytz.utc).timestamp()
            for aa in self.a_breadcrumb:
                a = self.a_breadcrumb[aa]
                n = len(a.tgt_posBuffer)
                x_hist = []
                y_hist = []
                for k in range(n):
                    x, y = normalize_locs(a.tgt_posBuffer[k].long, a.tgt_posBuffer[k].lat, self.img_width,
                                          self.img_height, current_map)
                    x_hist.append(x)
                    y_hist.append(y)
                estimator_timestamp = current_estimator_time - a.time_last_message
                color = 'orange'
                n = len(a.tgt_posBuffer)
                if estimator_timestamp < 1:
                    if n > 5:
                        a_breadcrumb = self.plot.create_line(x_hist[n - 1], y_hist[n - 1], x_hist[n - 2], y_hist[n - 2],
                                                             x_hist[n - 3], y_hist[n - 3], x_hist[n - 4], y_hist[n - 4],
                                                             x_hist[n - 5], y_hist[n - 5], x_hist[n - 6], y_hist[n - 6],
                                                             x_hist[n - 7], y_hist[n - 7], x_hist[n - 8], y_hist[n - 8],
                                                             x_hist[n - 9], y_hist[n - 9], x_hist[n - 10],
                                                             y_hist[n - 10],
                                                             fill=color, dash=(5, 2),
                                                             width=5)
                        self.plot.tag_raise(a_breadcrumb)

    def update_a_breadcrumb(self):
        for aa in self.a_breadcrumb:
            temp_target_x, temp_target_y = normalize_locs(self.a_breadcrumb[aa].est_tgt_long,
                                                          self.a_breadcrumb[aa].est_tgt_lat,
                                                          self.img_width,
                                                          self.img_height, current_map)  # temporary x ad y location of
            self.a_breadcrumb[aa].update_grid(temp_target_x, temp_target_y)

    def update_DiscoveryDrone(self):
        for ii in self.DiscoveryDrone:
            discovery_correct_long = self.DiscoveryDrone[ii].disc_long / 1e7
            discovery_correct_lat = self.DiscoveryDrone[ii].disc_lat / 1e7
            print('Discovery Long=', discovery_correct_long)
            print('Discovery Lat =', discovery_correct_lat)
            temp_UAV_x, temp_UAV_y = normalize_locs(discovery_correct_long, discovery_correct_lat, self.img_width,
                                                    self.img_height, current_map)  # temporary x ad y location of
            self.DiscoveryDrone[ii].update_grid(temp_UAV_x, temp_UAV_y)

    # These next two functions are used to plot the offset of the Discovery Drone (will mostly be for debugging)##
    def draw_DiscoveryDroneoffset(self):
        current_estimateTime = datetime.datetime.now(tz=pytz.utc).timestamp()
        if len(self.DiscoveryDroneoffset) > 0:
            try:
                for ii in self.DiscoveryDroneoffset:
                    i = self.DiscoveryDroneoffset[ii]
                    truex = i.grid_x_est_offset
                    truey = i.grid_y_est_offset
                color = 'green'
                estimate_timestamp = current_estimateTime - i.sensorTime
                if estimate_timestamp < 0.5:
                    DiscoveryDroneOffset_shape = self.plot.create_oval(truex - 10, truey - 10, truex + 10, truey + 10,
                                                                       outline=color,
                                                                       fill=color)
                    self.plot.tag_raise(
                        DiscoveryDroneOffset_shape)  # plot the expected Discovery Drone position with offset
            except:
                print("Error in the Discovery Drone Offset Dictionary")

    def update_DiscoveryDroneoffset(self):
        for ii in self.DiscoveryDroneoffset:
            try:
                temp_UAV_x, temp_UAV_y = normalize_locs(self.DiscoveryDroneoffset[ii].long_est_offset,
                                                        self.DiscoveryDroneoffset[ii].lat_est_offset, self.img_width,
                                                        self.img_height, current_map)  # temporary x ad y location of Dsicovery Drone
                self.DiscoveryDroneoffset[ii].update_grid(temp_UAV_x, temp_UAV_y)
            except:
                print('Error in the Discovery Drone Offset Dictionary')

    # This method updates the object variables with data pulled from C2 (communications methods)
    # rad_array is an array of radar_data objects
    # old is the index of the last (oldest) entry of the radar array, since the array is circular
    # new is the index of the first (newest) entry of the radar array, since the array is circular
    # drone_new is the index of the most receent entry in the radar array with drone data
    #   (this is necessary to draw the big circle in the right location, since not all radar messages
    #    will inherently contain a drone that was detected)
    # acoustics is an array of acoustic objects (with acoustic data ) from C2
    def update_log(self, pods, acoustic_targets, true_UAV, asterix_48, asterix_34, DiscoveryDrone, t_breadcrumb,
                   a_breadcrumb, radarEstimate, DiscoveryDroneoffset):
        # Storing the new acoustic data
        self.pods = pods
        # Updating the objects in the acoustic array with x and y locations of the acoustic pods
        self.update_acoustics()

        self.acoustic_targets = acoustic_targets  # Storing the information from the Pod Target Estimator
        self.update_acoustic_targets()  # updates the information that results from the

        self.true_UAV_mess = true_UAV  # Storing the True UAV position information
        self.update_True_UAV()

        self.radar_asterix_48 = asterix_48
        self.update_radar_48()

        self.radar_asterix_34 = asterix_34
        self.update_radar_34()

        self.DiscoveryDrone = DiscoveryDrone
        self.update_DiscoveryDrone()

        self.t_breadcrumb = t_breadcrumb
        self.update_t_breadcrumb()

        self.a_breadcrumb = a_breadcrumb
        self.update_a_breadcrumb()

        self.radarEstimate = radarEstimate
        self.update_radarEstimate()

        self.DiscoveryDroneoffset = DiscoveryDroneoffset
        self.update_DiscoveryDroneoffset()

        # self.breadcrumb = breadcrumb
        # self.update_breadcrumb()

    # This method (run in C2) is invoked if none of the objects we are expecting to receive a message
    # from actually send us a message (or conversely, if we simply do not receive any of the messages).
    # This turns all the entries in the asset status bar red to indicate we do not have any comms.
    def indicate_no_connection(self):
        # Setting all asset statuses to red (-1 is red)
        self.status = [-1, -1, -1, -1, -1]

    def waypoint(self):
        # Print a notification to the user in the console
        print("Opening Window")
        # Setting the mitigation drone to active (1)
        # self.openNewWindow()
        # lat, long, alt = self.openNewWindow()
        self.openNewWindow()
        # Updating the display
        self.runGUI()
        # return lat, long, alt

    def openNewWindow(self):
        self.newWindow = tk.Tk()
        # sets the title of the
        # Toplevel widget
        self.newWindow.title("Discovery Drone Waypoint")
        # sets the geometry of toplevel
        self.newWindow.geometry("400x400")
        # A Label widget to show in toplevel
        Label(self.newWindow,
              text="Input latitude, longitude, and Altitude").pack()

        # Entry boxes for lat, long, alt
        lat_input = Entry(self.newWindow)
        lat_input.pack()
        lat_input.insert(0, "Lat")
        long_input = Entry(self.newWindow)
        long_input.pack()
        long_input.insert(0, "Long")
        alt_input = Entry(self.newWindow)
        alt_input.pack()
        alt_input.insert(0, "alt")

        def save_waypoint():
            global lat_val
            global long_val
            global alt_val
            lat_val = lat_input.get()
            long_val = long_input.get()
            alt_val = alt_input.get()
            # print(lat_val, long_val, alt_val)

        # Button for saving data
        save_button = tk.Button(self.newWindow, text="Save waypoint", command=save_waypoint)
        save_button.pack()

    def rtb_control(self):
        global rtb_flag
        global rtb_count
        global rtb_cont
        rtb_count = rtb_count + 1
        if rtb_count % 2 == 0:
            rtb_cont = "disabled"
        elif rtb_count % 2 == 1:
            rtb_cont = "enabled"
        if rtb_flag == 1:
            rtb_flag = 0
        elif rtb_flag == 0:
            rtb_flag = 1

    def cam_control(self):
        global cam_flag
        global cam_count
        global cam_cont
        cam_count = cam_count + 1
        if cam_count % 2 == 0:
            cam_cont = "auto     "
        elif cam_count % 2 == 1:
            cam_cont = "manual"
        if cam_flag == 1:
            cam_flag = 0
        elif cam_flag == 0:
            cam_flag = 1

    def follow_me_control(self):
        global follow_me_flag
        global follow_count
        global follow_cont
        follow_count = follow_count + 1
        if follow_count % 2 == 0:
            follow_cont = "disabled"
        elif follow_count % 2 == 1:
            follow_cont = "enabled"
        if follow_me_flag == 1:
            follow_me_flag = 0
        elif follow_me_flag == 0:
            follow_me_flag = 1

    def toggle_true_breadcrumb(self):
        global status_t
        if status_t == 1:
            status_t = 0
        elif status_t == 0:
            status_t = 1

    def toggle_acoustic_breadcrumb(self):
        global status_a
        if status_a == 1:
            status_a = 0
        elif status_a == 0:
            status_a = 1

    def gnd_loc(self):
        # Print a notification to the user in the console
        print("Opening Window")
        # Setting the mitigation drone to active (1)
        # self.openNewWindow()
        # lat, long, alt = self.openNewWindow()
        self.openGndWindow()
        # Updating the display
        self.runGUI()
        # return lat, long, alt

    def openGndWindow(self):
        self.gndWindow = tk.Tk()
        # sets the title of the
        # Toplevel widget
        self.gndWindow.title("Ground Station Location")
        # sets the geometry of toplevel
        self.gndWindow.geometry("400x400")
        # A Label widget to show in toplevel
        Label(self.gndWindow,
              text="Input latitude, longitude, and Altitude").pack()

        # Entry boxes for lat, long, alt
        lat_input = Entry(self.gndWindow)
        lat_input.pack()
        lat_input.insert(0, "Lat")
        long_input = Entry(self.gndWindow)
        long_input.pack()
        long_input.insert(0, "Long")
        alt_input = Entry(self.gndWindow)
        alt_input.pack()
        alt_input.insert(0, "alt")

        def save_gnd_loc():
            global lat_gnd
            global long_gnd
            global alt_gnd
            lat_gnd = lat_input.get()
            long_gnd = long_input.get()
            alt_gnd = alt_input.get()
            self.gnd_pos = Label(self.gndWindow, text="Location Updated").pack()
            # self.gnd_pos.place(x=300, y=670)
            # print(lat_val, long_val, alt_val)

        # Button for saving data
        save_button = tk.Button(self.gndWindow, text="Save location", command=save_gnd_loc)
        save_button.pack()

    # def sel1(self):
    #     global gnd_direction
    #     gnd_direction = 1
    #
    # def sel2(self):
    #     global gnd_direction
    #     gnd_direction = 2
    #
    # def sel3(self):
    #     global gnd_direction
    #     gnd_direction = 3
    #
    # def sel4(self):
    #     global gnd_direction
    #     gnd_direction = 4

    def cam_orient(self):
        global gnd_direction
        print()


    def yaw1(self):
        global yaw
        yaw = 1

    def yaw2(self):
        global yaw
        yaw = 2

    def yaw3(self):
        global yaw
        yaw = 3

    def yaw4(self):
        global yaw
        yaw = 4

    def status_view(self):
        global gnd_view
        global yaw_view
        global gnd_direction
        global yaw
        if gnd_direction == 0:
            gnd_view = "Not Selected"
        elif gnd_direction == 1:
            gnd_view = "North            "
        elif gnd_direction == 2:
            gnd_view = "East             "
        elif gnd_direction == 3:
            gnd_view = "South            "
        elif gnd_direction == 4:
            gnd_view = "West             "
        if yaw == 0:
            yaw_view = "Not Selected"
        elif yaw == 1:
            yaw_view = "0                   "
        elif yaw == 2:
            yaw_view = "90                  "
        elif yaw == 3:
            yaw_view = "180                 "
        elif yaw == 4:
            yaw_view = "270                 "

    def save_speed(self):
        global speed_val
        global speed_changed
        global speed_count
        speed_val = self.speed.get()
        speed_changed = "Speed Updated"
        speed_count = speed_count + 1

    def toggle_true_dot(self):
        global status_true
        if status_true == 1:
            status_true = 0
        elif status_true == 0:
            status_true = 1

    def toggle_acoustic_dot(self):
        global status_acoustic
        if status_acoustic == 1:
            status_acoustic = 0
        elif status_acoustic == 0:
            status_acoustic = 1

    def toggle_radar_dot(self):
        global status_radar
        if status_radar == 1:
            status_radar = 0
        elif status_radar == 0:
            status_radar = 1

    def acoustic_msg(self):
        global toggle_acoustic
        global acoustic_stat
        if toggle_acoustic == 1:
            toggle_acoustic = 0
            acoustic_stat = "Off"
        elif toggle_acoustic == 0:
            toggle_acoustic = 1
            acoustic_stat = "On"

    def image_drone(self):
        global drone_img_processing
        if drone_img_processing == 1:
            drone_img_processing = 0
        elif drone_img_processing == 0:
            drone_img_processing = 1

    def image_ground(self):
        global ground_img_processing
        if ground_img_processing == 1:
            ground_img_processing = 0
        elif ground_img_processing == 0:
            ground_img_processing = 1

    def fm_sel1(self):
        global followme_select
        global fm_view
        followme_select = 1
        fm_view = "True UAV       "

    def fm_sel2(self):
        global followme_select
        global fm_view
        followme_select = 2
        fm_view = "Radar             "

    def fm_sel3(self):
        global followme_select
        global fm_view
        followme_select = 3
        fm_view = "Sensor Fusion"

    def ground_pic(self):
        global ground_pic_cont
        if ground_pic_cont == 1:
            ground_pic_cont = 0
        elif ground_pic_cont == 0:
            ground_pic_cont = 1

    def drone_pic(self):
        global drone_pic_cont
        if drone_pic_cont == 1:
            drone_pic_cont = 0
        elif drone_pic_cont == 0:
            drone_pic_cont = 1
    def Stillman_or_Athletics(self, map_select):
        global current_map
        current_map = map_select
        if map_select != "Close":
            if map_select == "Stillman":
                photo = Image.open("Wideview Background shortened no labels.PNG")
            elif map_select == "Athletic Fields":
                 #photo.close()
                photo = Image.open("Athletic Fields.png")
            elif map_select == "Krusty Krab":
                photo = Image.open("krusty krab.png")
            self.img = ImageTk.PhotoImage(photo)
            image_label = tk.Label(image=self.img)
            image_label.image = self.img
         #else:
            #Image.Image.close(self)
            #photo.kill()

    def stillman(self):
        global map_select
        map_select = 0
        photo = Image.open("Wideview Background shortened no labels.PNG")
        self.img = ImageTk.PhotoImage(photo)
        image_label = tk.Label(image=self.img)
        image_label.image = self.img

    def athletic_fields(self):
        global map_select
        map_select = 1
        photo = Image.open("Athletic Fields.png")
        self.img = ImageTk.PhotoImage(photo)
        image_label = tk.Label(image=self.img)
        image_label.image = self.img

    # def krusty_Krab(self):
    #     global map_select
    #     map_select = 2
    #     photo = Image.open("krusty krab.png")
    #     self.img = ImageTk.PhotoImage(photo)
    #     image_label = tk.Label(image=self.img)
    #     image_label.image = self.img


# This method normalizes latitude and longitude coordinates to x and y locations of the GUI map
# x is the longitude of the object in question
# y is the latitude of the object in question
# width is the width (in pixels/x-coordinates) of the GUI map image
# height is the height (in pixels/y-coordinates) of the GUI map image
# new_x (returned) is the integer x-coordinate on the GUI map of the object in question
# new_y (returned) is the integer y-coordinate on the GUI map of the object in question
def normalize_locs(x, y, width, height, map_select):
    # Getting the proportion of how far to the right that the object is
    # new_y = (y-39.007801)/(39.010327-39.007801)
    # new_y = (y-39.010327)/(39.007801 - 39.010327) #corrected to put the proper pod on top of the screen
    if map_select == "Athletic Fields":
        new_y = (y - 39.019531) / (39.013628 - 39.019531)
        new_x = ((-1 * x) - 104.884319) / (104.896911 - 104.884319)
    else:
        new_y = (y - 39.010335) / (39.006756 - 39.010335)  # corrected to put the proper pod on top of the screen
        # Getting the proportion of how far down that the object is (x-coordinates are inverted on the GUI, 0 is at the top)
        # new_x = ((-1*x)-104.878072)/(104.884606-104.878072)
        new_x = ((-1 * x) - 104.875085) / (104.884831 - 104.875085)
    # Getting the proportion of pixels of how far to the right the object is, and subsequently the y-coordinate
    new_y *= height

    # Getting the proportion in pixels of how far down the object is
    new_x *= width
    # Finding the actual x-coordinate of the drone
    new_x = width - new_x
    # Returning the new normalized x and y coordinates of the object
    return int(new_x), int(new_y)

# This method calculates the correct x and y locations of a drone on the GUI map
# rad_x is the x position on the GUI map of the radar
# rad_y is the y position of the radar on the GUI map
# range is the distance (in meters) of how far away the drone is from the radar
# az is the azimuth angle (in degrees) between the radar and the drone
# elev is the elevation angle (in degrees) between the radar and the drone
# width is the width (in pixels/x-coordinates) of the GUI map image
# height is the height (in pixels/y-coordinates) of the GUI map image
# drone_x (returned) is the x position of the drone on the GUI map
# drone_y (returned) is the y position of the drone on the GUI map
# def position_drone(rad_x, rad_y, range, az, elev, width, height):
#     # Converting given angles from degrees to radians
#     elev_angle = math.radians(elev)
#     az_angle = math.radians(az - 90)
#     # Calculating the distance based on land coordinates
#     # (if the drone was level with the radar, how far away it would be)
#     dist = range*math.cos(elev_angle)
#     # Calculating the height of the drone
#     alt = range*math.sin(elev_angle)
#     # Calculating the relative x distance on the screen for the drone
#     drone_x = ((dist*math.cos(az_angle))/559.45)
#     # Calculating the number of pixels of offset from the radar in the x direction
#     drone_x *= width
#     # Calculating the relative y distance on the screen for the drone
#     drone_y = ((dist*math.sin(az_angle))/267)
#     # Calculating the number of pixels of offset from the radar in the y direction
#     drone_y *= height
#     # Determining the actual x and y coordinates of the drone based on the radar position
#     drone_y += rad_y
#     drone_x += rad_x
#     # Return the drone x and y coordinates
#     return drone_x, drone_y