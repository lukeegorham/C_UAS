## Originally created by Class of 2019 C-UAS Capstone Team orignal file name (capstone2018_sensor_fusion2.py)
##Modified by C1C Ryan Schneider C/O 2021 Spring 2021



from collections import namedtuple
import math
import array
import numpy as np
from time import sleep
import utm
import datetime
import pytz

radar_estimate_struct = namedtuple("radar_estimate_struct", "time_unix_epoch_sec range_m az_deg el_deg pos_lat_deg"
                                                            "pos_long_deg pos_elev_m tar_lat_deg tar_long_deg"
                                                            "tar_alt_m")

state = namedtuple("state", "X P current_time")

udp_state = namedtuple("udp", "client ipEndPoint")

acoustic_estimate_struct = namedtuple("acoustic_estimate_struct", "x y z time")

deg2rad = math.pi / 180.0
k_filter = state
radar_estimate = radar_estimate_struct

acoustic_azymuth_left = 0.0
acoustic_azymuth_right = 0.0
filter_exists = False

shared_memory = None

# t1 = Timer(100, refresh())

message_length = 80

def generate_new_filter_radar(radarMeasurment):  # name for radar message is radar_message
    current_state = state

    current_state.X = np.zeros((6,1))

    #current_state.X[0] = radar_meas.range_m * math.cos(radar_meas.el_deg * deg2rad) * math.sin(
        #radar_meas.az_deg * deg2rad)
    current_state.X[0] = radarMeasurment[1]

    # y pos
    #current_state.X[1] = radar_meas.range_m * math.cos(radar_meas.el_deg * deg2rad) * math.cos(radar_meas.az_deg
    # * deg2rad)
    current_state.X[2] = radarMeasurment[2]
    # z pos
    #current_state.X[2] = radar_meas.range_m * math.sin(radar_meas.el_deg * deg2rad)
    current_state.X[3] = radarMeasurment[3]

    #print(current_state.X[3])
    # x velocity
    current_state.X[3] = radarMeasurment[4]
    # y velocity
    current_state.X[4] = radarMeasurment[5]
    # z velocity
    current_state.X[5] = radarMeasurment[6]

    current_state.P = np.zeros((6, 6))
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

    #current_state.current_time = radar_meas.time_unix_epoch_sec
    current_state.current_time = radarMeasurment[0]
    #print(current_state.current_time)
    #print(current_state.P)
    return current_state



def generate_new_filter_acoustic(acoustic_meas):  # name for radar message is radar_message
    current_state = state

    current_state.X = np.zeros((6,1))

    current_state.X[0] = acoustic_meas[1]

    # y pos
    current_state.X[1] = acoustic_meas[2]
    # z pos
    current_state.X[2] = acoustic_meas[3]
    # x velocity
    current_state.X[3] = acoustic_meas[4]
    # y velocity
    current_state.X[4] = acoustic_meas[5]
    # z velocity
    current_state.X[5] = acoustic_meas[6]

    current_state.P = np.zeros((6, 6))
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

    current_state.current_time = acoustic_meas[0]

    return current_state



def update_with_radar_position_estimate(tmp_state, cur_r, measFlag):
    cur_format ="{:.10f}".format(cur_r[0])
    measTime = float(cur_format)
    stateTime_format = "{:.10f}".format(tmp_state.current_time)
    stateTime = float(stateTime_format)
    dt = measTime - stateTime
    time_format = "{:.10f}".format(dt)
    dt = float(time_format)
    #print(tmp_state.current_time)
    #dt = measurement.last_radar_message - tmp_state.current_time
    weight = 0.1
    dt2 = weight * dt * dt
    dt3 = weight * 0.5 * dt * dt * dt
    dt4 = weight * 0.25 * dt * dt * dt * dt
    dt = measTime - stateTime

    # Set new time
    tmp_state.current_time = measTime
    tar_pos = np.zeros((3, 1))


    if measFlag == True:
    # Convert from spherical to rectangular coordinates
    # x pos
    #tar_pos[0][0] = cur_r.range_m * math.cos(cur_r.el_deg * deg2rad) * math.sin(cur_r.az_deg * deg2rad)
        tar_pos[0][0] = cur_r[1]
    # y pos
    #tar_pos[1][0] = cur_r.range_m * math.cos(cur_r.el_deg * deg2rad) * math.cos(cur_r.az_deg * deg2rad)
        tar_pos[1][0] = cur_r[2]
    # z pos
    #tar_pos[2][0] = cur_r.range_m * math.sin(cur_r.el_deg * deg2rad)
        tar_pos[2][0] = cur_r[3]
    # print("Tar pos", tar_pos)
    # Propagate in time
    phi1 = np.array([[1.0, 0.0, 0.0, dt, 0.0, 0.0],
                     [0.0, 1.0, 0.0, 0.0, dt, 0.0],
                     [0.0, 0.0, 1.0, 0.0, 0.0, dt],
                     [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                     [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                     [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])

    phi_mat = phi1
    # print("Before: ", tmp_state.X)
    # Propagate states
    #print(tmp_state.X)
    #tmp_state.X = np.dot(phi_mat, tmp_state.X)
    tmp_state.X = phi_mat @ tmp_state.X
    #print(tmp_state.X)
    # print("After: ", tmp_state.X)
    # print("dt: ", dt)
    wgamma1 = np.array([[dt4, 0.0, 0.0, dt3, 0.0, 0.0],
                        [0.0, dt4, 0.0, 0.0, dt3, 0.0],
                        [0.0, 0.0, dt4, 0.0, 0.0, dt3],
                        [dt3, 0.0, 0.0, dt2, 0.0, 0.0],
                        [0.0, dt3, 0.0, 0.0, dt2, 0.0],
                        [0.0, 0.0, dt3, 0.0, 0.0, dt2]])
    w_gamma_mat = wgamma1

    # Propagate covariance matrix
    tmp_state.P = np.add(np.dot(np.dot(phi_mat, tmp_state.P), np.transpose(phi_mat)), w_gamma_mat)
    # print("P: ", tmp_state.P)
    # Update with measurement data
    # print(tmp_state.X)
    x = tmp_state.X[0][0]
    # print(x)
    y = tmp_state.X[1][0]
    z = tmp_state.X[2][0]

    y_mat = np.zeros((3, 1))
    # yel
    y_mat[0] = tar_pos[0][0]
    # yaz
    y_mat[1] = tar_pos[1][0]
    # yR
    y_mat[2] = tar_pos[2][0]
    # print("ymat: ", y_mat)
    # h matrix
    h = np.zeros((3, 1))
    h[0][0] = x
    h[1][0] = y
    h[2][0] = z

    h_tmp = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
             [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
             [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]])
    H = h_tmp

    # v
    v = np.zeros((3, 3))
    # el
    v[0, 0] = 5
    # az
    v[1, 1] = 5
    # R
    v[2, 2] = 5


    if measFlag == True:
        # Calculate Kalman gain
        k = np.dot(np.dot(tmp_state.P, np.transpose(H)), np.linalg.inv(np.add(np.dot(np.dot(H, tmp_state.P), np.transpose(H)), v)))
        # print("Kalman gain")
        # print(k)

        i = np.identity(6)

        # Update covariance matrix
        tmp_state.P = np.add(np.dot(np.dot(np.subtract(i, np.dot(k , H)), tmp_state.P), np.transpose(np.subtract(i, np.dot(k, H)))), np.dot(np.dot(k, v), np.transpose(k)))
        # Update state
        # print("Before: ", tmp_state.X)
        # print("K: ", k)
        # print("y_mat: ", y_mat)
        # print("h: ", h)
        # print(np.dot(k, np.subtract(y_mat, h)))
        tmp_state.X = np.add(tmp_state.X, np.dot(k, np.subtract(y_mat, h)))
        #print(tmp_state.X)
        # print("After: ", tmp_state.X)
        # print("States (x,y,z,Vx,Vy,Vz")
        # print(tmp_state.X)
        # print("Covariance")
        # print(tmp_state.P)
        # sleep(15)
    return tmp_state



def triangulation(lmast_data, rmast_data):

    #recieve the azmuith and elevation data
    lmast_data = array.array('d', shared_memory.get_udp_lmst())
    rmast_data = array.array('d', shared_memory.get_udp_rmst())


    lmast_az = lmast_data[0]
    lmast_elv = lmast_data[1]
    rmast_az = rmast_data[0]
    rmast_elv = rmast_data[1]
    time = rmast_data[2]


    # input the gps location of the masts and command center
    lmast_lat = 0.0
    lmast_long = 0.0

    rmast_lat = 0.0
    rmast_long = 0.0

    cmd_ctr_lat = 39.009074
    cmd_ctr_long = -104.8826846


    # get the utm locations

    lmast_loc = utm.from_latlon(lmast_lat, lmast_long)
    rmast_loc = utm.from_latlon(rmast_lat, lmast_long)

    cmd_ctr_loc = utm.from_latlon(cmd_ctr_lat, cmd_ctr_long)

    #x,y,z array of the positions

    lmast_pos = np.array([lmast_loc[0], lmast_loc[1], 0.5])
    rmast_pos = np.array([rmast_loc[0], rmast_loc[1], 0.5])
    #unit vector for each mast directions

    lmast_unit = np.array([math.sin(lmast_az)*math.cos(lmast_elv), math.cos(lmast_az)*math.cos(lmast_elv), math.sin(lmast_elv)])
    rmast_unit = np.array([math.sin(rmast_az)*math.cos(rmast_elv), math.cos(rmast_az)*math.cos(rmast_elv), math.sin(rmast_elv)])
    m = (np.cross(np.subtract(lmast_pos, rmast_pos), rmast_unit))/(np.cross(lmast_unit,rmast_unit))
    #triangulated location still in utm
    tar_pos = lmast_pos + m*lmast_unit

    #center around control center
    acoustic_estimate = acoustic_estimate_struct(x=tar_pos[0] - cmd_ctr_loc[0], y=tar_pos[1] - cmd_ctr_loc[1], z=tar_pos[2], time=time)


    return acoustic_estimate


def update_with_acoustic_position_estimate(tmp_state, cur_r, measFlag):
    #
    # dt = cur_r.time - tmp_state.current_time
    # #dt = cur_r.last_radar_message - tmp_state.current_time
    #
    # weight = 0.1
    # dt2 = weight * dt * dt
    # dt3 = weight * 0.5 * dt * dt * dt
    # dt4 = weight * 0.25 * dt * dt * dt * dt
    # dt = cur_r.time_unix_epoch_sec - tmp_state.current_time
    #
    # # Set new time
    # tmp_state.current_time = cur_r[3]
    # tar_pos = np.zeros((3, 1))
    cur_format ="{:.10f}".format(cur_r[0])
    measTime = float(cur_format)
    stateTime_format = "{:.10f}".format(tmp_state.current_time)
    stateTime = float(stateTime_format)
    dt = measTime - stateTime
    time_format = "{:.10f}".format(dt)
    dt = float(time_format)
    #print(tmp_state.current_time)
    #dt = measurement.last_radar_message - tmp_state.current_time
    weight = 0.1
    dt2 = weight * dt * dt
    dt3 = weight * 0.5 * dt * dt * dt
    dt4 = weight * 0.25 * dt * dt * dt * dt
    dt = measTime - stateTime

    # Set new time
    tmp_state.current_time = measTime
    tar_pos = np.zeros((3, 1))

    if measFlag == True:
        # Convert from spherical to rectangular coordinates
        # x pos
        tar_pos[0][0] = cur_r[1]
        # y pos
        tar_pos[1][0] = cur_r[2]
        # z pos
        tar_pos[2][0] = cur_r[3]



    phi1 = np.array([[1.0, 0.0, 0.0, dt, 0.0, 0.0],
                     [0.0, 1.0, 0.0, 0.0, dt, 0.0],
                     [0.0, 0.0, 1.0, 0.0, 0.0, dt],
                     [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                     [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                     [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])

    phi_mat = phi1

    # Propagate states
    tmp_state.X = np.dot(phi_mat, tmp_state.X)

    wgamma1 = np.array([[dt4, 0.0, 0.0, dt3, 0.0, 0.0],
                        [0.0, dt4, 0.0, 0.0, dt3, 0.0],
                        [0.0, 0.0, dt4, 0.0, 0.0, dt3],
                        [dt3, 0.0, 0.0, dt2, 0.0, 0.0],
                        [0.0, dt3, 0.0, 0.0, dt2, 0.0],
                        [0.0, 0.0, dt3, 0.0, 0.0, dt2]])
    w_gamma_mat = wgamma1

    # Propagate covariance matrix
    tmp_state.P = np.add(np.dot(np.dot(phi_mat, tmp_state.P), np.transpose(phi_mat)), w_gamma_mat)


    # Update with measurement data
    x = tmp_state.X[0][0]
    # print(x)
    y = tmp_state.X[1][0]
    z = tmp_state.X[2][0]

    y_mat = np.zeros((3, 1))
    # yel
    y_mat[0] = tar_pos[0][0]
    # yaz
    y_mat[1] = tar_pos[1][0]
    # yR
    y_mat[2] = tar_pos[2][0]
    # h matrix
    h = np.zeros((3, 1))
    h[0][0] = x
    h[1][0] = y
    h[2][0] = z

    h_tmp = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
             [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
             [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]])
    H = h_tmp

    # v
    v = np.zeros((3, 3))
    # el
    v[0, 0] = 20     #these are a lot higher because we have less confidence in acoustics
    # az
    v[1, 1] = 20
    # R
    v[2, 2] = 20

    if measFlag == True:
        # Calculate Kalman gain
        k = np.dot(np.dot(tmp_state.P, np.transpose(H)), np.linalg.inv(np.dot(np.dot(np.dot(H, tmp_state.P), np.transpose(H)), v)))
        # print("Kalman gain")
        # print(k)

        i = np.identity(6)

        # Update covariance matrix
        tmp_state.P = np.dot(np.dot(np.subtract(i, np.dot(k , H)), tmp_state.P), np.transpose(np.add(np.subtract(i, np.dot(k, H)), np.dot(np.dot(k, v), np.transpose(k)))))
        # Update state
        # print("Before: ", tmp_state.X)
        # print("K: ", k)
        # print("y_mat: ", y_mat)
        # print("h: ", h)
        # print(np.dot(k, np.subtract(y_mat, h)))
        tmp_state.X = np.add(tmp_state.X, np.dot(k, np.subtract(y_mat, h)))
        # print("After: ", tmp_state.X)
        # print("States (x,y,z,Vx,Vy,Vz")
        # print(tmp_state.X)
        # print("Covariance")
        # print(tmp_state.P)
        # sleep(15)
    return tmp_state


def sensor_fusion(shd_mem): ## This function is unused in the updated version of the code by C1C Schneider and Class of 2021

    shared_memory = shd_mem
    radar_estimate_struct = namedtuple("radar_estimate_struct", "time_unix_epoch_sec az_deg el_deg pos_lat_deg"
                                                                "pos_long_deg pos_elev_m tar_lat_deg tar_long_deg"
                                                                "tar_alt_m acoustic_time")

    state = namedtuple("state", "X P current_time")

    acoustic_estimate_struct = namedtuple("acoustic_estimate_struct", "x y z time")

    udp_state = namedtuple("udp", "client ipEndPoint")

    deg2rad = math.pi / 180.0
    k_filter = state
    radar_estimate = radar_estimate_struct
    lmast_estimate = acoustic_estimate_struct
    rmast_estimate = acoustic_estimate_struct

    acoustic_azymuth_left = 0.0
    acoustic_azymuth_right = 0.0
    filter_exists = False
    acoustic_meaningful = False

    while True:

        if shared_memory.get_udp_radar() == b'':
            i = 1
        else:
            radar_sequence = array.array('d', shared_memory.get_udp_radar())

            radar_estimate.time_unix_epoch_sec = radar_sequence[0]
            radar_estimate.range_m = radar_sequence[1]
            radar_estimate.az_deg = radar_sequence[2]
            radar_estimate.el_deg = radar_sequence[3]
            radar_estimate.pos_lat_deg = radar_sequence[4]
            radar_estimate.pos_long_deg = radar_sequence[5]
            radar_estimate.pos_elev_m = radar_sequence[6]
            radar_estimate.tar_lat_deg = radar_sequence[7]
            radar_estimate.tar_long_deg = radar_sequence[8]
            radar_estimate.tar_alt_m = radar_sequence[9]

            if filter_exists is False:
                filter = generate_new_filter_radar(radar_estimate)
                filter_exists = True

            else:
                filter = update_with_radar_position_estimate(filter, radar_estimate)

        if shared_memory.get_udp_lmst() == b'' or shared_memory.get_udp_rmst() == b'':
            i = 1
        else:

            lmast_sequence = array.array('d', shared_memory.get_udp_lmast())
            rmast_sequence = array.array('d', shared_memory.get_udp_rmast())

            lmast_estimate.azmuith = lmast_sequence[0]
            lmast_estimate.elevation = lmast_sequence[1]
            lmast_estimate.time = lmast_sequence[2]

            rmast_estimate.azmuith = rmast_sequence[0]
            rmast_estimate.elevation = rmast_sequence[1]
            rmast_estimate.time = rmast_sequence[2]

            if lmast_estimate.azmuith == 179.8056 or lmast_estimate.elevation == 90 or rmast_estimate.azmuith == 179.8056 or rmast_estimate.elevation == 90:
                i = 1

            else:

                acoustic_estimate = triangulation()

                if filter_exists is False:

                    filter = generate_new_filter_acoustic(acoustic_estimate)
                    filter_exists = True;

                else:
                    filter = update_with_acoustic_position_estimate(filter, acoustic_estimate)

        while shared_memory.lock() == -1:
            sleep(0.0000001)
        shared_memory.add_sf_uav(filter)