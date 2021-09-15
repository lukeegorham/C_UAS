# Imports to get the correct libraries and help the code run
import tkinter as tk
from tkinter import *
from PIL import ImageTk, Image
import threading
import socket
import sys
from time import sleep, time
import struct
import math
import cv2
import pickle
import numpy as np
import struct
import zlib
import time
# Global variable that indicates whether we have a camera feed for the GUI
# It must be a global because the receive_imagery function is outside the C2GUI class
camera = 1

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
    #image = ImageTk.PhotoImage(Image.open("Background.gif"))
    im = Image.open(r"C:\Users\C21Peter.Park\Documents\2021GUI\background.jpg")
    im1 = im.tobytes("xbm", "rgb")
    img = Image.frombuffer("L", (4, 4), im1, 'raw', "L", 0, 1)

    img2 = list(img.getdata())
    print(img2)
    # Updating what the GUI should look like
    program.runGUI()
    # Displaying an image from the discovery drone
    receive_imagery()
    # Refreshing the screen to display the updates given by runGUI
    program.window.update()
    # Returning the C2GUI object to the C2 code so it works with the same GUI object as it runs
    return program

# This class stores a GUI object, as well as all the data necessary to make the GUI display properly.
# The class also updates the information as needed based on inputs from C2.
class C2GUI():

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
        # Setting the discovery and mitigation as initially inactive (0)
        self.disc_active = 0
        self.mit_active = 0
        # Determining the initial text for the buttons
        self.disc_text = "Launch discovery"
        self.mit_text = "Launch mitigation"
        # Storing variables in a new class for time calculations
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
        self.create_widgets()

        self.acoustic_targets = [] #array that stores infromation from the

        self.true_UAV_mess = [] #array that stores true_UAV position information

    # This method creates the widgets (display boxes, buttons, etc.) that are displayed on the GUI
    # This only needs to run once on setup for the GUI
    def create_widgets(self):
        # Creating the frame for asset statuses
        self.top_frame = tk.LabelFrame(self.window, text="Asset Statuses")
        self.window.rowconfigure(1, weight=1)

        # Creating the label that displays the status of the camera
        self.cam = tk.Label(self.top_frame, text="Camera", bg=self.determine_color(0))
        # Creating the label that displays the status of the radar
        self.rad = tk.Label(self.top_frame, text="Radar", bg=self.determine_color(1))
        # Creating the label that displays the status of the discovery drone
        self.disc = tk.Label(self.top_frame, text="Discovery Drone", bg=self.determine_color(2))
        # Creating the label that displays the status of the mitigate drone
        self.mit = tk.Label(self.top_frame, text="Mitigate Drone", bg=self.determine_color(3))
        # Creating the label that displays the status of the acoustics
        self.acoustics = tk.Label(self.top_frame, text="Acoustics", bg=self.determine_color(4))

        # Creating the frame for event statuses (drone detection)
        self.left_frame = tk.LabelFrame(self.window, text="Event Statuses")
        self.window.rowconfigure(2, weight=1)

        # Creating the label that displays whether a drone has been detected
        self.detect = tk.Label(self.left_frame, text=self.determine_text(0)[0], bg=self.determine_text(0)[1])
        # Creating the label that displays the speed of the incoming drone
        self.speed = tk.Label(self.left_frame, text="Incoming drone speed: "+self.determine_text(1)[0]+" m/s", bg=self.determine_text(1)[1])
        # Creating the label that displays the time it will take for the drone to get to the FOB
        self.time = tk.Label(self.left_frame, text="Incoming drone time to FOB: "+self.determine_text(2)[0]+" sec", bg=self.determine_text(2)[1])

        # Creating the frame for the control buttons
        self.bottom_frame = tk.Frame(self.window)

        # Creating the button that launches and pauses/unpauses the discovery drone
        self.disc_button = tk.Button(self.bottom_frame, text=self.disc_text, command=self.pause_disc)
        # Creating the button that launches and pauses/unpauses the mitigate drone
        self.mit_button = tk.Button(self.bottom_frame, text=self.mit_text, command=self.pause_mit)
        # Creating the button that returns the drones to base
        self.rtb_button = tk.Button(self.bottom_frame, text="Return to Base", command=self.return_base)
        # Creating the button that launches both drones
        self.launch_button = tk.Button(self.bottom_frame, text="Launch Drones", command=self.launch)
        # Creating the button that allows the mitigate drone to mitigate the threat
        self.mitigate_button = tk.Button(self.bottom_frame, text="Mitigate Threat", command=self.mitigate)

        # Creating the frame for the map
        self.mapFrame = tk.Frame(self.window)
        # Opening the background image
        photo = Image.open("Background.gif")
        # Determining the size of the photo for future reference
        self.img_width, self.img_height = photo.size
        # Creating the canvas to display the map and locations of the drones, radar, and acoustic pods
        self.plot = tk.Canvas(self.mapFrame, height=self.img_height+1, width=self.img_width+1)
        # Packing the plot (allows us to display it)
        self.plot.pack()
        # Saving the image as several different tkinter objects so garbage collection doesn't delete the photo and it actually displays
        self.img = ImageTk.PhotoImage(photo)
        image_label = tk.Label(image=self.img)
        image_label.image = self.img
        # Displaying the background image
        pic = self.plot.create_image(0, 0, anchor=NW, image=self.img)

    # This method updates the actual display elements of the GUI based on inputs from the C2 code
    # The method does not actually receive the updates from C2, but rather simply looks at the object
    # memory locations (self.info, self.pods, etc.) to update the GUI accordingly
    def runGUI(self):
        # Updating status array from a global variable so we know if the camera is sending or not
        # This ensures the asset status sidebar is accurate
        self.status[0] = camera

        # Drawing the top frame (asset statuses)
        self.top_frame.grid(row=1, column=1, pady=10, padx=10, sticky=tk.W)
        # Drawing the camera status box and ensuring it is the right color
        self.cam.grid(row=2, column=1, pady=10, padx=10, columnspan=2, sticky=tk.W + tk.E)
        self.cam.configure(bg=self.determine_color(0))
        # Drawing the radar status box and ensuring it is the right color
        self.rad.grid(row=3, column=1, pady=10, padx=10, columnspan=2, sticky=tk.W + tk.E)
        self.rad.configure(bg=self.determine_color(1))
        # Drawing the discovery drone status box and ensuring it is the right color
        self.disc.grid(row=4, column=1, pady=10, padx=10, columnspan=2, sticky=tk.W + tk.E)
        self.disc.configure(bg=self.determine_color(2))
        # Drawing the mitigate drone status box and ensuring it is the right color
        self.mit.grid(row=5, column=1, pady=10, padx=10, columnspan=2, sticky=tk.W + tk.E)
        self.mit.configure(bg=self.determine_color(3))
        # Drawing the acoustic status box and ensuring it is the right color
        self.acoustics.grid(row=6, column=1, pady=10, padx=10, columnspan=2, sticky=tk.W + tk.E)
        self.acoustics.configure(bg=self.determine_color(4))

        # Drawing the left frame (event statuses)
        self.left_frame.grid(row=2, column=1, pady=10, padx=10, sticky=tk.W)
        # Drawing the drone detected box and ensuring it is the right color and displays the right text
        self.detect.grid(row=2, column=1, pady=10, padx=10, sticky=tk.W)
        self.detect.configure(text=self.determine_text(0)[0], bg=self.determine_text(0)[1])
        # Drawing the drone speed box and ensuring it is the right color and displays the right text
        self.speed.grid(row=3, column=1, pady=10, padx=10, sticky=tk.W)
        self.speed.configure(text="Incoming drone speed: "+self.determine_text(1)[0]+" m/s", bg=self.determine_text(1)[1])
        # Drawing the drone time to FOB box and ensuring it is the right color and displays the right text
        self.time.grid(row=4, column=1, pady=10, padx=10, sticky=tk.W)
        self.time.configure(text="Incoming drone time to FOB: "+self.determine_text(2)[0]+" sec", bg=self.determine_text(2)[1])

        # Drawing the bottom frame (control buttons)
        self.bottom_frame.grid(row=3, column=1, pady=10, padx=10, sticky=tk.W)
        # Drawing the launch discovery button and ensuring the displayed text is correct
        self.disc_button.grid(row=1, column=1, padx=5, pady=5, sticky=tk.W)
        self.disc_button.configure(text=self.disc_text)
        # Drawing the launch mitigation button and ensuring the displayed text is correct
        self.mit_button.grid(row=1, column=2, padx=5, pady=5, sticky=tk.E)
        self.mit_button.configure(text=self.mit_text)
        # Drawing the return to base button
        self.rtb_button.grid(row=2, column=1, columnspan=2, padx=5, pady=5, sticky=tk.E+tk.W)
        # Drawing the launch (both) drones button
        self.launch_button.grid(row=3, column=1, columnspan=2, padx=5, pady=5, sticky=tk.E + tk.W)
        # Drawing the mitigate threat button
        self.mitigate_button.grid(row=4, column=1, columnspan=2, padx=5, pady=5, sticky=tk.E + tk.W)

        # Getting the width and height of the background (parade field) image
        width = self.img_width
        height = self.img_height
        # Drawing the background picture
        pic = self.plot.create_image(0, 0, anchor=NW, image=self.img)
        # Ensuring there is at least one data entry in the radar log variable
        # before attempting to read the data and display it
        if(len(self.log)>=1):
            # Calculating radar x and y map coordinates
            radar_x, radar_y = normalize_locs(self.log[self.new].radarLong_deg, self.log[self.new].radarLat_deg, width, height)
            # Drawing the radar on the map
            rad = self.plot.create_rectangle(radar_x - 10, radar_y - 10, radar_x + 10, radar_y + 10, fill="#fcf51e")
            # Updating the variable that controls the asset status sidebar so we know the radar is sending data
            self.status[1] = 1
            # Ensuring there is at least one drone in the latest radar data object
            # before displaying the drones on the GUI
            if(self.log[self.new].numTgts > 0):
                # If a drone is detected, indicate it on the event status sidebar (1 is drone detected)
                self.info[0] = 1
                # Determine how many drones we need to display
                num_drones = int(self.log[self.new].numTgts)
                # Loop through each drone and display it
                for j in range(num_drones):
                    # Calculating the grid positions of each drone
                    temp_x, temp_y = position_drone(radar_x, radar_y, self.log[self.new].drones[j].range_m,
                                                    self.log[self.new].drones[j].azimuth_deg,
                                                    self.log[self.new].drones[j].elevation_deg, width-120, height)
                    # Storing the grid location for each drone into the drone objects
                    self.log[self.new].drones[j].store_grid_loc(temp_x, temp_y, width, height)
                    # Draw the drone on the GUI map
                    self.draw_drone()
            # If there is no drone in the radar data object
            else:
                # Create a circle with no size in the corner
                drone = self.plot.create_oval(0, 0, 0, 0)
                # Position the drone behind the GUI map image
                self.plot.tag_lower(drone)
                # Indicate no drone is detected on the event status bar (-1 is no drone)
                self.info[0] = -1
            # Position the radar square in front of the GUI map image
            self.plot.tag_raise(rad)
        # If we do not have a radar_data object (meaning we didn't receive a new radar message)
        else:
            # Indicating no drone is detected on the event status bar if we do not have a radar message
            # (This could possibly be updated to work with acoustics as well)
            self.status[1] = -1
        # Positioning the map on the overall GUI
        self.mapFrame.grid(row=1, column=3, rowspan=3, pady=5, padx=5, sticky=tk.W)
        # Drawing the acoustic pod on the GUI map
        self.draw_acoustics()
        self.draw_True_UAV_Pos()
        self.draw_acoustic_targets()
        # Receiving and displaying images received from the discovery drone
        # (This might need to eventually be put into the C2 code instead)
        receive_imagery()

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
        if box==0:
            # If no drone is detected
            if self.info[box] < 0:
                return ["No drone detected", "light green"]
            # If a drone is detected (The hexadecimal indicates a light red color)
            else:
                return ["Drone detected", "#ff5e5e"]
        # Displaying the drone speed/time to fob of the other two boxes
        else:
            # If no drone is detected
            if(self.info[box] < 0):
                return ["N/A", "light green"]
            # If a drone is detected (The hexadecimal indicates a light red color))
            else:
                return [str(self.info[box]), "#ff5e5e"]

    # Method that is called when a user presses the "Pause discovery" button
    # This is where you would call the drone control methods once they are implemented
    def pause_disc(self):
        # If the drone is already active
        if(self.disc_active == 1):
            # Print a notification to the user in the console
            print("Discovery paused!")
            # Setting the discovery drone to inactive (0)
            self.disc_active = 0
            self.disc_text = "Unpause discovery"
        # If the drone is inactive
        else:
            # Print a notification to the user in the console
            print("Discovery unpaused!")
            # Setting the discovery drone to active (1)
            self.disc_active = 1
            # Updating the text that the button displays
            self.disc_text = "Pause discovery"
        # Updating the display to ensure the changes to the button text go into effect
        self.runGUI()

    # Method that is called when a user presses the "Pause miigation" button
    # This is where you would call the drone control methods once they are implemented
    def pause_mit(self):
        if (self.mit_active == 1):
            # Print a notification to the user in the console
            print("Mitigation paused!")
            # Setting the mitigation drone to inactive (0)
            self.mit_active = 0
            # Updating the text that the button displays
            self.mit_text = "Unpause mitigation"
        # If the mitigation drone was already paused
        else:
            # Print a notification to the user in the console
            print("Mitigation unpaused!")
            # Setting the drone to active (1)
            self.mit_active = 1
            # Updating the text that the button displays
            self.mit_text = "Pause mitigation"
        # Updating the display to ensure the changes to the button text go into effect
        self.runGUI()

    # Method that is called when a user presses the "Return to base" button
    # This is where you would call the drone control methods once they are implemented
    def return_base(self):
        # Print a notification to the user in the console
        print("Drones returning to base")
        self.mit_active = 0
        self.disc_active = 0
        self.runGUI()

    # Method that is called when a user presses the "Launch drones" button
    # This is where you would call the drone control methods once they are implemented
    def launch(self):
        # Print a notification to the user in the console
        print("Launching Drones")
        # Setting both drones to active (1)
        self.mit_active = 1
        self.disc_active = 1
        # Updating the display
        self.runGUI()

    # Method that is called when a user presses the "Mitigate threat" button
    # This is where you would call the drone control methods once they are implemented
    def mitigate(self):
        # Print a notification to the user in the console
        print("Mitigating threat")
        # Setting the mitigation drone to active (1)
        self.mit_active = 1
        # Updating the display
        self.runGUI()

    # def draw_breadcrumb(self):
    #     global drone
    #     ind = 0
    #
    #     if (len(self.log) >= 1):
    #         # Looping through the radar data entries
    #         for j in self.log:
    #             # Ensuring there are drones to draw to prevent any errors
    #             if (len(j.drones) > 0):
    #                 # Looping through the drones in the radar data entry
    #                 for k in j.drones:
    #                     # Drawing the most recent entry as a large circle for the user
    #                     if (ind == self.drone_new):
    #                         # Drawing the actual circle on the GUI map
    #                         drone = self.plot.create_oval(k.grid_x - 10, k.grid_y - 10, k.grid_x + 10, k.grid_y + 10,
    #                                                       fill="red")
    #                     # Drawing smaller circles for older entries
    #                     else:
    #                         # Drawing the actual circle on the GUI map
    #                         drone = self.plot.create_oval(k.grid_x - 3, k.grid_y - 3, k.grid_x + 3, k.grid_y + 3,
    #                                                       fill="red")
    #                 # Placing the newly created circle in front of the GUI map on the GUI
    #                 self.plot.tag_raise(drone)
    #             # Increasing the index to indicate we cycled through an entry in the radar array
    #             ind += 1

    # Method that actually draws the drones on the GUI map based on the object's radar array
    def draw_drone(self):
        # Index of the drone that we are currently drawing (used for drone_new/ the big circle)
        global drone
        ind = 0

        # Ensuring there is radar data to begin with to prevent an error
        if(len(self.log)>=1):
            # Looping through the radar data entries
            for j in self.log:
                # Ensuring there are drones to draw to prevent any errors
                if(len(j.drones)>0):
                    # Looping through the drones in the radar data entry
                    for k in j.drones:
                        # Drawing the most recent entry as a large circle for the user
                        if(ind == self.drone_new):
                            # Drawing the actual circle on the GUI map
                            drone = self.plot.create_oval(k.grid_x - 10, k.grid_y - 10, k.grid_x + 10, k.grid_y + 10, fill="red")
                        # Drawing smaller circles for older entries
                        else:
                            # Drawing the actual circle on the GUI map
                            drone = self.plot.create_oval(k.grid_x - 3, k.grid_y - 3, k.grid_x + 3, k.grid_y + 3, fill="red")
                    # Placing the newly created circle in front of the GUI map on the GUI
                    self.plot.tag_raise(drone)
                # Increasing the index to indicate we cycled through an entry in the radar array
                ind += 1

    # This method draws the acoustic pods at the appropriate location on the GUI map
    # (You might want to field test this though, I'm not sure it 100% works in the field with real acoustic data)
    def draw_acoustics(self):
        # Ensuring there is actual acoustic data to display to prevent an error
        if(len(self.pods) > 0):
            # Indicating acoustics are good/sending data (1) in the asset status panel
            self.status[4] = 1
            # Looping through the acoustic pod entries
            int_computer_time_target = 0
            for j in self.pods:
                # Getting the recently calculated and stored x and y coordinates of the acoustic pod

                acoustic_x = j.grid_x
                acoustic_y = j.grid_y
                # If the pod hears a drone (AKA, this is a simulated Acoustic Target Message)
                if(j.msg_type == 1):
                    # Turn the pod green on the display
                    color = "blue"

                # If the pod hears anything else (not a drone)
                    #computer_time_target = time.clock() #gets the time when the target was detect
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
                points = [acoustic_x, acoustic_y - 8, acoustic_x - 12, acoustic_y + 12, acoustic_x + 12, acoustic_y + 12]
                # Actually draw the triangle on the GUI map
                pod = self.plot.create_polygon(points, fill=color)
                # Bring the acoustic pod triangle in front of the GUI map image
                self.plot.tag_raise(pod)
                # Finding the points to draw a circle around the pod and indicate its range
                # (You should probably change this to be more accurate or possibly even dynamically allocated)
                ac_rad = self.plot.create_oval(acoustic_x-70, acoustic_y-70, acoustic_x+70, acoustic_y+70, outline=color)
                # Bring the acoustic pod circle in front of the GUI map image
                self.plot.tag_raise(ac_rad)
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
            # Calculating the normalized acoustic locations in x and y coordinates on the GUI map
            temp_ac_x, temp_ac_y = normalize_locs(m.pod_long, m.pod_lat, self.img_width, self.img_height)
            # Storing the x and y locations in the actual acoustic objects
            m.update_grid(temp_ac_x, temp_ac_y)

    def draw_acoustic_targets(self):
        if (len(self.acoustic_targets) > 0):
            for jj in self.acoustic_targets:
                acousticx = jj.grid_estimator_x
                acousticy = jj.grid_estimator_y
            color = 'orange'
            target_shape = self.plot.create_oval(acousticx-10,  acousticy-10, acousticx+10, acousticy+10, outline=color, fill = color)
            self.plot.tag_raise(target_shape)

    def update_acoustic_targets(self):
        for jj in self.acoustic_targets:
            temp_target_x, temp_target_y = normalize_locs(jj.est_tgt_long, jj.est_tgt_lat, self.img_width,self.img_height)  # temporary x and y location of
            # target from a singular pod message
            jj.update_estimator_grid(temp_target_x, temp_target_y)

    def draw_True_UAV_Pos(self):
        if(len(self.true_UAV_mess) > 0):
            for ii in self.true_UAV_mess:
                truex = ii.grid_true_x
                truey = ii.grid_true_y
            color = 'purple'
            UAV_shape = self.plot.create_oval(truex-10, truey-10, truex+10, truey+10, outline=color, fill = color)
            self.plot.tag_raise(UAV_shape)

    def update_True_UAV(self):
        for ii in self.true_UAV_mess:
            temp_UAV_x, temp_UAV_y = normalize_locs(ii.UAV_long, ii.UAV_lat, self.img_width,self.img_height)  # temporary x ad y location of
            ii.update_true_grid(temp_UAV_x, temp_UAV_y)



    # This method updates the object variables with data pulled from C2 (communications methods)
    # rad_array is an array of radar_data objects
    # old is the index of the last (oldest) entry of the radar array, since the array is circular
    # new is the index of the first (newest) entry of the radar array, since the array is circular
    # drone_new is the index of the most receent entry in the radar array with drone data
    #   (this is necessary to draw the big circle in the right location, since not all radar messages
    #    will inherently contain a drone that was detected)
    # acoustics is an array of acoustic objects (with acoustic data ) from C2
    def update_log(self, rad_array, old, new, drone_new, acoustics, acoustic_targets, true_UAV):
        # Storing the new radar data
        self.log = rad_array
        # Saving the indices so the circular array runs properly and the drones and breadcrumb trails are correct
        self.old = old
        self.new = new
        self.drone_new = drone_new
        # Storing the new acoustic data
        self.pods = acoustics
        # Updating the objects in the acoustic array with x and y locations of the acoustic pods
        self.update_acoustics()

        self.acoustic_targets = acoustic_targets #Storing the information from the Pod Target Estimator
        self.update_acoustic_targets() #updates the information that results from the

        self.true_UAV_mess = true_UAV #Storing the True UAV position information
        self.update_True_UAV()

    # This method (run in C2) is invoked if none of the objects we are expecting to receive a message
    # from actually send us a message (or conversely, if we simply do not receive any of the messages).
    # This turns all the entries in the asset status bar red to indicate we do not have any comms.
    def indicate_no_connection(self):
        # Setting all asset statuses to red (-1 is red)
        self.status = [-1, -1, -1, -1, -1]

# This method normalizes latitude and longitude coordinates to x and y locations of the GUI map
# x is the longitude of the object in question
# y is the latitude of the object in question
# width is the width (in pixels/x-coordinates) of the GUI map image
# height is the height (in pixels/y-coordinates) of the GUI map image
# new_x (returned) is the integer x-coordinate on the GUI map of the object in question
# new_y (returned) is the integer y-coordinate on the GUI map of the object in question
def normalize_locs(x, y, width, height):
    # Getting the proportion of how far to the right that the object is
    #new_y = (y-39.007801)/(39.010327-39.007801)
    new_y = (y-39.010327)/(39.007801 - 39.010327) #corrected to put the proper pod on top of the screen
    # Getting the proportion of pixels of how far to the right the object is, and subsequently the y-coordinate
    new_y *= height
    # Getting the proportion of how far down that the object is (x-coordinates are inverted on the GUI, 0 is at the top)
    new_x = ((-1*x)-104.878072)/(104.884606-104.878072)
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

# This method receives imagery from the discovery drone and displays it in a popup window.
# You might want to consider moving it into C2 and integrating it there instead, as it thematically
# works better there. The code was mainly pulled off a website then altered to use UDP and our specific
# IP addresses and ports (https://gist.github.com/kittinan/e7ecefddda5616eab2765fdb2affed1b).
# def receive_imagery():
#     global camera
#     HOST = ''
#     PORT = 8485
#
#     s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     s.settimeout(1)
#     print('Socket created')
#
#     s.bind((HOST, PORT))
#     print('Socket now listening')
#
#     data = b""
#     payload_size = struct.calcsize(">L")
#     print("payload_size: {}".format(payload_size))
#     try:
#             while len(data) < payload_size:
#                 dataRcvd, addr = s.recvfrom(1000000)
#                 data += dataRcvd
#             print("Done Recv: {}".format(len(data)))
#             camera = 1
#             packed_msg_size = data[:payload_size]
#             data = data[payload_size:]
#             msg_size = struct.unpack(">L", packed_msg_size)[0]
#             print("msg_size: {}".format(msg_size))
#             while len(data) < msg_size:
#                 dataRcvd, addr = s.recvfrom(1000000)
#                 data += dataRcvd
#             frame_data = data[:msg_size]
#             data = data[msg_size:]
#
#             frame = pickle.loads(frame_data, fix_imports=True, encoding="bytes")
#             frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
#             cv2.imshow('ImageWindow', frame)
#             cv2.waitKey(1)
#     except:
#         print("Socket timed out")
#         camera = -1
# main()


# Old code from when the GUI was standalone (not integrated with C2)

        # # Getting locations from Comms (WILL BE UPDATED TO READ FROM SHARED MEMORY)
        # drone_loc = start_server()
        # # Ensuring the message is not empty
        # if(drone_loc != 0):
        #     # Making sure the radar asset status is set to good
        #     self.status[1] = 1
        #     # Determining the length of the radar message
        #     length = len(drone_loc)
        #     # Determining the format that the message was packed
        #     form = '<' + str(int(length/8)) + 'd'
        #     # Getting an array of radar data from the message
        #     locations = struct.unpack(form, drone_loc)
        #     # Creating a Radar_Data object to hold the new message
        #     msg = Radar_Data()
        #     # Filling the array if it is not full
        #     if(self.looped == 0):
        #         # Throwing the new message into a new slot at the end
        #         self.log.append(msg.store_info(locations))
        #         # Incrementing the recent variable to point to the latest data
        #         self.new += 1
        #         # Determining if the array was filled after this entry
        #         if(self.new>=self.logDepth):
        #             # Changing the boolean if the array was filled
        #             self.looped = 1
        #     # Storing data properly if the array is full
        #     else:
        #         # Incrementing the recent variable to point to the most recent data location
        #         #         self.new += 1
        #         #         # Looping to the beginning of the circular array after the last entry
        #         #         if (self.new > self.logDepth):
        #         #             self.new = 0
        #         #         # Determining the location of the oldest data
        #         #         self.old += self.new + 1
        #         #         # Looping the location of the oldest data at the end of the circular array
        #         #         if(self.old>self.logDepth):
        #         #             self.old = 0
        #         # Storing the newest data at the proper location
        #         self.log[self.new] = msg.store_info(locations)
        #     if(len(msg.drones)<=0):
        #         self.info[0] = -1
        # Loop that we run if we do not receive a radar message
        # else:
        #     # Setting the locations to -1 to ensure the GUI knows there was no data received
        #     locations = [-1, -1, -1, -1, -1, -1, -1]
        #     # Setting the color of the radar to red to indicate the radar is down
        #     self.status[1] = -1
        # Indicating that there is no drone detected in the event statuses
        # if(locations[6]<=0):
        #     self.info[0] = -1
        # # Indicating that a drone was detected in the event statuses
        # else:
        #     self.info[0] = 1

# def start_server():
#     """
#     Creates and starts the UDP server to grab all of our data. It binds to a port and continually listens.
#     :param shared_memory: the shared memory object across the server
#     :return: radar data message
#     """
#
#     # Defines our IP's
#     radar_ip = '192.168.1.61'
#     port_num = 55565
#     # radar_message_length = 72
#     server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     server_socket.bind(('', port_num))
#     server_socket.settimeout(1)
#     sleep(0.5)
#     # print("UDP SERVER: RUNNING")
#
#     try:
#         data, addr = server_socket.recvfrom(100000)
#
#         if addr[0]==radar_ip:
#             return data
#             # if len(data) != radar_message_length:
#             #     print("Incomplete message from radar")
#             #     return 0
#             # else:
#             #     return data
#         else:
#             print("UNRECOGNIZED IP: ", addr)
#             return 0
#     except:
#         print("Could not receive data from radar")
#         return 0