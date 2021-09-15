#!/usr/bin/env python
import socket
from collections import namedtuple
import math
from threading import Timer, Thread
from command_control import refresh
import numpy as np

<<<<<<< HEAD
# radar_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# radar_udp_port_num = 55565
# radar_in.bind(('', radar_udp_port_num))
#
# print("Going to recieve data")
# while True:
#
# 	data, addr = radar_in.recvfrom(1024)
# 	print ("Message: ", data)


def Generate_New_Filter(radar_estimate_struct, radar_meas):  #name for radar message is radar_message
	current_state = State

	current_state.X = np.zeros(6)

	current_state.X[0] = radar_meas.range_m * math.cos(radar_meas.el_deg * deg2rad) * Math.sin(radar_meas.az_deg * deg2rad)
	# y pos
    current_state.X[1] = radar_meas.range_m * math.cos(radar_meas.el_deg * deg2rad)* Math.cos(radar_meas.az_deg * deg2rad)
	# z pos
    current_state.X[2] = radar_meas.range_m * math.sin(radar_meas.el_deg * deg2rad)
	# x velocity
    current_state.X[3] = -5
	# y velocity
    current_state.X[4] = 0
	# z velocity
=======





def generate_new_filter(radar_meas):  #name for radar message is radar_message
	current_state = state

    current_state.X = np.zeros(6)

    current_state.X[0] = radar_meas.range_m * math.cos(radar_meas.el_deg * deg2rad) * math.sin(
        radar_meas.az_deg * deg2rad)
    # y pos

    current_state.X[1] = radar_meas.range_m * math.cos(radar_meas.el_deg * deg2rad) * math.cos(radar_meas.az_deg
                                                                                               * deg2rad)
    # z pos
    current_state.X[2] = radar_meas.range_m * math.sin(radar_meas.el_deg * deg2rad)
    # x velocity
    current_state.X[3] = -5
    # y velocity
    current_state.X[4] = 0
    # z velocity
>>>>>>> iss7
    current_state.X[5] = 0



<<<<<<< HEAD

	current_state.P = np.zeros(6,6)
	# x variance
    current_state.P[0, 0] = 1
	# y variance
    current_state.P[1, 1] = 1
	# z variance
    current_state.P[2, 2] = 1
	# Vx variance
    current_state.P[3, 3] = 1
	# Vy variance
    current_state.P[4, 4] = 1
	# Vz variance
    current_state.P[5, 5] = 1


	current_state.current_time = radar_meas.time_unix_epoch_sec


	return current_state



def Update_With_Radar(tmpState, radar_estimate_struct, cur_r):
    dt = cur_r.time_unix_epoch_sec - tmpState.current_time
    #Set new time
    tmpState.current_time = cur_r.time_unix_epoch_sec

    # Propagate in time
    phi1 = {{1.0, 0.0, 0.0, dt, 0.0, 0.0},
         {0.0, 1.0, 0.0, 0.0, dt, 0.0},
         {0.0, 0.0, 1.0, 0.0, 0.0, dt},
         {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
         {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
         {0.0, 0.0, 0.0, 0.0, 0.0, 1.0}}

    PhiMat = phi1

    # Propagate states
    tmpState.X = PhiMat * tmpState.X


    weight = 0.1
    dt2 = weight * dt * dt
    dt3 = weight * 0.5 * dt * dt * dt
    dt4 = weight * 0.25 * dt * dt * dt * dt
    wgamma1 ={{dt4,0.0,0.0,dt3,0.0,0.0},
             {0.0,dt4,0.0,0.0,dt3,0.0},
             {0.0,0.0,dt4,0.0,0.0,dt3},
             {dt3,0.0,0.0,dt2,0.0,0.0},
             {0.0,dt3,0.0,0.0,dt2,0.0},
             {0.0,0.0,dt3,0.0,0.0,dt2}}
    WGammaMat = wgamma1
        #test
    #Propagate covariance matrix
    tmpState.P = PhiMat * tmpState.P * np.transpose(PhiMat) + WGammaMat

    # Update with measurement data
    x = tmpState.X[0]
    y = tmpState.X[1]
    z = tmpState.X[2]

    y_mat = np.zeros(3)
    # yel
    y_mat[0] = cur_r.el_deg * math.pi / 180
    # yaz
    y_mat[1] = cur_r.az_deg * math.pi / 180
    # yR
    y_mat[2] = cur_r.range_m

    # h matrix
    h = np.zeros(3)
    # elevation estimate
    h[0] = math.Asin(z / math.sqrt(x ** 2 + y ** 2 + z ** 2))
    # azmiuth estimate
    h[1] = math.Atan2(x, y)
    # Range estimate
    h[2] = math.sqrt(x ** 2 + y ** 2 + z ** 2)


    H = np.zeros(3, 6)

    # Hel
    H[0, 0] = (-1 * x * z) / (math.sqrt(x ** 2 + y ** 2) * (x ** 2 + y ** 2 + z ** 2))
    H[0, 1] = (-1 * y * z) / (math.sqrt(x ** 2 + y ** 2) * (x ** 2 + y ** 2 + z ** 2))
    H[0, 2] = math.sqrt(x ** 2 + y ** 2) / (x ** 2 + y ** 2 + z ** 2)
    #Haz
    H[1, 0] = y / (x ** 2 + y ** 2)
    H[1, 1] = -1 * x / (x ** 2 + y ** 2)
    #HR
    H[2, 0] = x / math.sqrt(x ** 2 + y ** 2 + z ** 2)
    H[2, 1] = y / math.sqrt(x ** 2 + y ** 2 + z ** 2)
    H[2, 2] = z / math.sqrt(x ** 2 + y ** 2 + z ** 2)
    #V
    V = np.zeros(3, 3)
    # el
    V[0, 0] = 1
    # az
    V[1, 1] = 1
    # R
    V[2, 2] = 1
    # phi

    # Calculate Kalman gain
    K = tmpState.P * np.transpose(H) * np.linalg.inv(H * tmpState.P * np.transpose(H) + V)


    I = np.identity(6)

    # Update covariance matrix
    tmpState.P = ((I - K * H) * tmpState.P * np.transpose(I - K * H) + K * V * np.transpose(K))

    #Update state
    tmpState.X = (tmpState.X + (K * (y_mat - h)))
    print(tmpState.X)
    print(tmpState.P)

    return tmpState


def Update_with_Radar_Position_Estimate(tmpState, radar_estimate_struct, cur_r):
    dt2 = weight * dt * dt
    dt3 = weight * 0.5 * dt * dt * dt
    dt4 = weight * 0.25 * dt * dt * dt * dt
    dt = cur_r.time_unix_epoch_sec - tmpState.current_time
    #Set new time
    tmpState.current_time = cur_r.time_unix_epoch_sec
    tarPos = np.zeros(3)
    #Convert from spherical to rectangular coordinates
    # x pos
    tarPos[0] = cur_r.range_m * math.cos(cur_r.el_deg * deg2rad)* math.sin(cur_r.az_deg * deg2rad)
    # y pos
    tarPos[1] = cur_r.range_m * math.cos(cur_r.el_deg * deg2rad)* math.cos(cur_r.az_deg * deg2rad)
    # z pos
    tarPos[2] = cur_r.range_m * math.sin(cur_r.el_deg * deg2rad)

    # Propagate in time
    phi1 = {{1.0, 0.0, 0.0, dt, 0.0, 0.0},
         {0.0, 1.0, 0.0, 0.0, dt, 0.0},
         {0.0, 0.0, 1.0, 0.0, 0.0, dt},
         {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
         {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
         {0.0, 0.0, 0.0, 0.0, 0.0, 1.0}}

    PhiMat = phi1

    # Propagate states
    tmpState.X = (PhiMat * tmpState.X)


    weight = 0.1
    wgamma1 ={{dt4,0.0,0.0,dt3,0.0,0.0},
             {0.0,dt4,0.0,0.0,dt3,0.0},
             {0.0,0.0,dt4,0.0,0.0,dt3},
             {dt3,0.0,0.0,dt2,0.0,0.0},
             {0.0,dt3,0.0,0.0,dt2,0.0},
             {0.0,0.0,dt3,0.0,0.0,dt2}}
    WGammaMat = wgamma1

    # Propagate covariance matrix
    tmpState.P = PhiMat * tmpState.P * np.transpose(PhiMat) + WGammaMat

    # Update with measurement data
    x = tmpState.X[0]
    y = tmpState.X[1]
    z = tmpState.X[2]

    y_mat = np.zeros(3)
    #yel
    y_mat[0] = tarPos[0]
    # yaz
    y_mat[1] = tarPos[1]
    # yR
    y_mat[2] = tarPos[2]

    # h matrix
    h = np.zeros(3)
    h[0] = x
    h[1] = y
    h[2] = z

    Htmp = { { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
             { 0.0, 1.0, 0.0, 0.0, 0.0, 0.0 },
             { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 } }
    H = Htmp

    #V
    V = np.zeros(3, 3)
    # el
    V[0, 0] = 5
    # az
    V[1, 1] = 5
    # R
    V[2, 2] = 5
    # phi

    # Calculate Kalman gain
    K = tmpState.P * np.transpose(H) * np.linalg.inv(H * tmpState.P * np.transpose(H) + V)
    print("Kalman gain")
    print(K)


    I = np.identity(6)

    # Update covariance matrix
    tmpState.P = ((I - K * H) * tmpState.P * np.transpose(I - K * H) + K * V * np.transpose(K))

    # Update state
    tmpState.X = (tmpState.X + (K * (y_mat - h)))
    print("States (x,y,z,Vx,Vy,Vz")
    print(tmpState.X)
    print("Covariance")
    print(tmpState.P)

    return tmpState
=======
	current_state.P = np.zeros(6, 6)
    # x variance
    current_state.P[0, 0] = 1
    # y variance
    current_state.P[1, 1] = 1
    # z variance
    current_state.P[2, 2] = 1
    # Vx variance
    current_state.P[3, 3] = 1
    # Vy variance
    current_state.P[4, 4] = 1
    # Vz variance
    current_state.P[5, 5] = 1

    current_state.current_time = radar_meas.time_unix_epoch_sec

    return current_state


	###### Comparing this to Brennan's converted code...... above this is done need to look below still


def Update_with_Radar_Position_Estimate(tmpState, cur_r)
	{
		dt = cur_r.time_unix_epoch_sec - tmpState.current_time

		#Set new time
		tmpState.current_time = cur_r.time_unix_epoch_sec
		tarPos = np.zeros(3);
		#Convert from spherical to rectangular coordinates
		# x pos
		tarPos[0] = cur_r.range_m * math.cos(cur_r.el_deg * deg2rad)* math.sin(cur_r.az_deg * deg2rad);
		# y pos
		tarPos[1] = cur_r.range_m * math.cos(cur_r.el_deg * deg2rad)* math.cos(cur_r.az_deg * deg2rad);
		# z pos
		tarPos[2] = cur_r.range_m * math.sin(cur_r.el_deg * deg2rad);

		# Propagate in time
		phi1 =
			{{1.0, 0.0, 0.0, dt, 0.0, 0.0},
			 {0.0, 1.0, 0.0, 0.0, dt, 0.0},
			 {0.0, 0.0, 1.0, 0.0, 0.0, dt},
			 {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
			 {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
			 {0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};

		PhiMat = phi1;

		# Propagate states
		tmpState.X = (PhiMat * tmpState.X)


		weight = 0.1;
		dt2 = weight * dt * dt;
		dt3 = weight * 0.5 * dt * dt * dt;
		dt4 = weight * 0.25 * dt * dt * dt * dt;
		wgamma1 ={{dt4,0.0,0.0,dt3,0.0,0.0},
				 {0.0,dt4,0.0,0.0,dt3,0.0},
				 {0.0,0.0,dt4,0.0,0.0,dt3},
				 {dt3,0.0,0.0,dt2,0.0,0.0},
				 {0.0,dt3,0.0,0.0,dt2,0.0},
				 {0.0,0.0,dt3,0.0,0.0,dt2}};
		WGammaMat = wgamma1;

		# Propagate covariance matrix
		tmpState.P = PhiMat * tmpState.P * np.transpose(PhiMat) + WGammaMat;

		# Update with measurement data
		x = tmpState.X[0];
		y = tmpState.X[1];
		z = tmpState.X[2];

		y_mat = np.zeros(3);
		#yel
		y_mat[0] = tarPos[0];
		# yaz
		y_mat[1] = tarPos[1];
		# yR
		y_mat[2] = tarPos[2];

		# h matrix
		h = np.zeros(3);

		h[0] = x;
		h[1] = y;
		h[2] = z;

		Htmp = { { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
				 { 0.0, 1.0, 0.0, 0.0, 0.0, 0.0 },
				 { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 } };
		H = Htmp;

		#V
		V = np.zeros(3, 3);
		# el
		V[0, 0] = 5;
		# az
		V[1, 1] = 5;
		# R
		V[2, 2] = 5;
		# phi

		# Calculate Kalman gain
		K = tmpState.P * np.transpose(H) * np.linalg.inv(H * tmpState.P * np.transpose(H) + V);
		print("Kalman gain");
		print(K);


		I = np.identity(6);

		# Update covariance matrix
		tmpState.P = ((I - K * H) * tmpState.P * np.transpose(I - K * H) + K * V * np.transpose(K));

		# Update state
		tmpState.X = (tmpState.X + (K * (y_mat - h)));
		print("States (x,y,z,Vx,Vy,Vz");
		print(tmpState.X);
		print("Covariance");
		print(tmpState.P);

		return tmpState;


	}
>>>>>>> iss7
