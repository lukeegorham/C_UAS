# -*- coding: utf-8 -*-
"""
Created on Wed Oct 20 09:37:05 2021

@author: neil.rogers
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import traceback
import sys
import socket
import logging


def log_activity(msg, log=None):
    if log is None:
        logging.info(msg)
    else:
        log.info(msg)


def move_local(device, x, y, z, duration=1, log=None):
    log_activity(f"Local move with velocities {x},{y},{z} for {duration} seconds.", log)
    send_local_ned_velocity(device, x, y, z, duration)


def condition_yaw(device, heading, relative=False, log=None):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting
    the yaw using this function there is no way to return to the default yaw "follow direction
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see:
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """

    log_activity(f"Yaw to {heading} degrees (relative = {relative}).", log)

    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle

    # create the CONDITION_YAW command using command_long_encode()
    msg = device.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading,  # param 1, yaw in degrees
        0,  # param 2, yaw speed deg/s
        1,  # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)  # param 5 ~ 7 not used

    # send command to vehicle
    device.send_mavlink(msg)


def send_local_ned_velocity(device, velocity_x, velocity_y, velocity_z, duration=1):
    # To move up, down, left, right, you need to create a
    # vehicle.message_factory.set_position_target_local_ned_encode.
    # It will require a frame of mavutil.mavlink.MAV_FRAME_BODY_NED
    # (north, east, down reference).
    # You then add the required x,y and/or z velocities (in m/s) to the message.

    msg = device.message_factory.set_position_target_global_int_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
        #mavutil.mavlink.MAV_FRAME_BODY_FRD,  # frame, new reference
        0b0000111111000111,  # type_mask (only speeds enabled)
        #0b111000111000,  # type_mask (only speeds enabled) rogers
        0,  # lat_int - X Position in WGS84 frame in 1e7 * meters
        0,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0,  # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x,  # X velocity in NED frame in m/s
        velocity_y,  # Y velocity in NED frame in m/s
        velocity_z,  # Z velocity in NED frame in m/s
        0, 0, 0,  # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0, duration):
        device.send_mavlink(msg)
        time.sleep(1)


def connect_device(s_connection, baud=115200, log=None):
    log_activity("Connecting to device...", log)
    device = connect(s_connection, wait_ready=True, baud=baud)
    log_activity("Device connected.", log)
    log_activity(f"Device version: {device.version}, log=")
    return device


def arm_device(device, log=None, mode="GUIDED", n_reps=10):
    log_activity("Arming device...", log)
    wait = 1

    # "GUIDED" mode sets drone to listen
    # for our commands that tell it what to do...
    device.mode = VehicleMode(mode)
    while device.mode != mode:
        if wait > n_reps:
            log_activity("mode change timeout.", log)
            break

        log_activity(f"Switching to {mode} mode...", log)
        time.sleep(2)
        wait += 1
    wait = 1
    device.armed = True

    while not device.armed:
        if wait > n_reps:
            log_activity("arm timeout.", log)
            break
        log_activity("Waiting for arm...", log)
        time.sleep(1)

    log_activity(f"Device armed: {device.armed}.", log)

    return device.armed


def change_device_mode(device, mode, n_reps=10, log=None):
    wait = 0
    log_activity(f"Changing device mode from {device.mode} to {mode}...", log)

    device.mode = VehicleMode(mode)

    while device.mode != mode:
        if wait > n_reps:
            log_activity("mode change timeout.", log)
            return False
        device.mode = VehicleMode(mode)
        time.sleep(.5)
        wait += 1

    log_activity(f"Device mode = {device.mode}.", log)


def device_takeoff(device, altitude, log=None):
    log_activity("Device takeoff...", log)
    device.mode = VehicleMode("GUIDED")
    time.sleep(.5)
    device.simple_takeoff(altitude)
    device.airspeed = 3
    while device.armed \
            and device.mode == "GUIDED":
        log_activity(f"Current altitude: {device.location.global_relative_frame.alt}", log)
        if device.location.global_relative_frame.alt >= (altitude * .95):
            break
        time.sleep(.5)


def device_land(device, log=None):
    log_activity("Device land...", log)
    device.mode = VehicleMode("LAND")
    time.sleep(.5)
    while device.armed \
            and device.mode == "LAND":
        log_activity(f"Current altitude: {device.location.global_relative_frame.alt}", log)
        if device.location.global_relative_frame.alt <= 2:
            log_activity("Device has landed.", log)
            break
        time.sleep(.1)


def execute_flight_plan(device, n_reps=10, wait=1, log=None):
    if device.commands.count == 0:
        log_activity("No flight plan to execute.", log)
        return False

    log_activity("Executing flight plan...", log)
    # Reset mission set to first (0) waypoint
    device.commands.next = 0

    # Set mode to AUTO to start mission
    device.mode = VehicleMode("AUTO")
    time.sleep(.5)
    while device.mode != "AUTO":
        if wait > n_reps:
            log_activity("mode change timeout.", log)
            return False

        log_activity("Switching to AUTO mode...", log)
        time.sleep(1)
        wait += 1
    return True


def goto_point(device, lat, lon, speed, alt, log=None):
    #log_activity(f"Goto point: {lat}, {lon}, {speed}, {alt}...", log)
    print(speed)
    # set the default travel speed
    device.airspeed = speed
    
    point = LocationGlobalRelative(lat, lon, alt)

    device.simple_goto(point)
    print("abcdefghijklmnopqrstuvwxyz")
    while device.armed \
            and device.mode == "GUIDED":
        try:
            log_activity(f"Current altitude: {device.location.global_relative_frame.alt}", log)
            log_activity(f"Current lat: {device.location.global_relative_frame.lat}", log)
            log_activity(f"Current lon: {device.location.global_relative_frame.lon, log}")

            alt_percent = device.location.global_relative_frame.alt / alt
            lat_percent = device.location.global_relative_frame.lat / lat
            lon_percent = device.location.global_relative_frame.lon / lon

            log_activity(f"Relative position to destination: {alt_percent},{lat_percent}, {lon_percent}", log)

            if (0.99 <= alt_percent <= 1.1) \
                    and (.99 <= lat_percent <= 1.1) \
                    and (.99 <= lon_percent <= 1.1):
                break  # close enough - may never be perfectly on the mark
            print("Max, this is probably working. Time to test baby")
            time.sleep(1)
        except Exception as e:
            log_activity(f"Error on goto: {traceback.format_exception(*sys.exc_info())}", log)
            print("You done fucked up foo")
            raise
    #print("Hey stupid, the drone isn't armed")


def return_to_launch(device, log=None):
    log_activity("Device returning to launch...", log)
    device.mode = VehicleMode("RTL")
    time.sleep(.5)
    while device.armed \
            and device.mode == "RTL":
        log_activity(f"Current altitude: {device.location.global_relative_frame.alt}", log)
        if device.location.global_relative_frame.alt <= .01:
            log_activity("Device has landed.", log)
            break
        time.sleep(.5)


def set_home_usafa(device):
    msg = device.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,  # mav command
        0, #confirmation
        0, #1: use current location, 0: use specified location
        0, 0, 0, # 2-4 unused
        39.007808, -104.885737, 7148) #lat, lon, altitude
    # send command to vehicle on 1 Hz cycle
    device.send_mavlink(msg)