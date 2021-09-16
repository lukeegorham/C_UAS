-----------------------------
         GUI Master
-----------------------------

VARIABLES:
- camera ~ 
- lat_val ~ 
- long_val ~ 
- alt_val ~ 
- status_t ~ 
- status_a ~ 
- rtb_flag ~ 
- follow_me_flaG ~ 
- cam_flag ~ 
- cam_count ~ string 
- cam_cont ~ 
- disc_lat ~ 
- disc_long ~ 
- disc_alt ~ 
- gnd_direction ~ 
- yaw ~ 
- gnd_view ~ 
- yaw_view ~ 
- follow_count ~ 
- follow_cont ~ string
- rtb_count ~ 
- rtb_cont ~ string
- speed_val ~ 
- speed_changed ~ string
- speed_count ~ 
- status_radar ~ 
- status_acoustic ~ 
- status_true ~ 
- toggle_acoustic ~ 
- acoustic_stat ~ string
- drone_img_processing ~ 
- ground_img_processing ~ 
- followme_select ~ 
- fm_view ~ string
- ground_pic_cont ~ 
- drone_pic_cont ~ 

FUNCTIONS:
- run_main(rad_array, old, new) ~ initializes other variables and runs/returns GUI object
- normalize_locs(x, y, width, height) ~ normalizes lat/long to x/y locations on GUI map
? position_drone(rad_x, rad_y, range, az, elev, width, height) ~ calcs x/y location of drone on GUI map


C2GUI - OBJECT
- __init__
- print_hello ~ test func with random return
- create_widgets ~ displays each part of screen within GUI
- runGUI ~ updates display elements (not receive, just updates in memory and push to screen)
- determine_color ~ returns hex color of status boxes based on the box contents
- determine_text ~ returns text and overall status color code based on box contents
- draw_acoustics ~ draws acoustic pods at proper location on map
- update_acoustics ~ updates objects in acoustic array with x/y location of pods on GUI map
- draw_acoustic_targets ~ display target pod estimator info on GUI
- update_acoustic_targets ~ normalizes location from array of targets, saves x/y location
- draw_True_UAV_Pos
- update_True_UAV ~ normalizes location from true position, saves x/y location
- draw_Radar_Position ~ ______ pretty sure it draws radar targets on screen???
- update_radar_48 ~ ______ updates radar position in array
- draw_radar_34 ~ _______ draws radar targets on screen??? Maybe from a different freq???
- update_radar_34 ~ ______ updates radar position in array
- draw_radarEstimate ~ draws time-corrected radar estimate on screen from self.radarEstimate
- update_radarEstimate ~ normalizes location from data, saves x/y
- draw_DiscoveryDrone_Pos
? draw_breadcrumb ~ ???
? draw_breadcrumb (V2) ~ ???
? update_breadcrump ~ 
- draw_t_breadcrumb ~ assumes draws target breadcrumb (also does some timeCorr, Normz, etc)
- update_t_breadcrumb ~ assuming update breadcrumb, does a lot of other stuff too
- update_a_breadcrumb ~ ???
- update_DiscoveryDrone (not sure why not immediately following draw_DiscoveryDrone_Pos)
- update_DiscoveryDroneoffset - ???
- update_log ~ updates object vars w/ data from all sensors
- indicate_no_connection ~ sets status of each thing to -1
- waypoint ~ calls runGUI() subfunct in a "pretty" manner
- openNewWindow ~ opens new window with boxes to get lat/long/alt from user
	- save_waypoint ~ saves to variable for openNewWindow method
- rtb_control ~ toggle variable in memory and change display value in GUI
- cam_control ~ toggle variable in memory and change display value in GUI
- follow_me_control ~ toggle variable in memory and change display value in GUI
- toggle_true_breadcrumb ~ checkbox linked to variable in memory
- toggle_acoustic_breadcrumb ~ checkbox linked to variable in memory
- gnd_loc ~ call openNewWindow to update ground station location
- openGndWindow ~ opens new window with boxes to get lat/long/alt from user
	- save_gnd_loc ~ saves locaaiton to variables in memory
- sel1-4 ~ selects ground direction (N, E, S, w)
- yaw1-4 ~ selects yaw (1, 2, 3, 4)
- status_view ~ sets radio buttons for gnd_view and yaw
- save_speed ~ allows user to save speed of the UAV and remembers number of times updated
- toggle_true_dot
- toggle_acoustic_dot
- toggle_radar_dot
- acoustic_msg ~ sets acoustic_stat to "On"/"Off" and toggles toggle_acoustic
- image_drone ~ toggles drone_img_processing
- image_ground ~ toggles ground_img_processing
- fm_sel1-3 ~ changes fm_view between {"True UAV", "Radar", "Sensor Fusion"}
- ground_pic ~ toggles ground_pic_cont
- drone_pic ~ toggles drone_pic_cont



PROBLEMS:
- openNewWindow calls Tk() function, need only one Tk() call even if multiple windows, instead use 'Toplevel' (no parenthesis) https://stackoverflow.com/questions/23224574/tkinter-create-image-function-error-pyimage1-does-not-exist

- 
