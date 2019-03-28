# This is a custom library for useful functions that wrap standard DroneKit basic functions
# Last Update: 23 - 09 - 2018 -- Luca Gemma, University of Trento - Italy

# ----------------------------------------------- Imports ------------------------------------------------------------ #

from __future__ import print_function
import time, math, threading, logging
from time import gmtime, strftime
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil # for command message definitions
from logging import FileHandler
from logging import Formatter

# ---------------------------------------- Parameters declaration ---------------------------------------------------- #


mininum_distance = 1.2  # minimum distance from the target to consider the target reached
print_skip_iterations = 20  # number of loop iterations skipped before printing the output
# setting the output format and log level for the logging


# ----------------------------------------- Function definitions ----------------------------------------------------- #


def arm_and_takeoff(tgt_altitude, vehicle, stop_action=[False], _id=0, log_level=logging.INFO):  # add a third input param "interrupt"

    """
    Takeoff function - taking off to a target altitudes tgt_altitude
    :param tgt_altitude: final altitude for the take-off
    :param vehicle: UAV to command
    :param log_level: logging level
    :return:
    """

    # set level of logging
    logging.basicConfig(format='\t[%(levelname)s] %(message)s'.expandtabs(_id * 40), level=logging.DEBUG)
    logging.getLogger().setLevel(log_level)

    print("\t-- Initializing the drone --".expandtabs(_id * 40))

    # timestamps on file for starting time of function invocation ------------------------------------------------------
    with open('log_data.txt', 'a') as the_file:
        the_file.write('\n----- Drone ' + str(_id) + '----- FUNCTION CALL: arm_and_takeoff(%s) -----  [%s]\n\n' % (str(tgt_altitude),strftime("%H:%M:%S", time.localtime())))
    # ------------------------------------------------------------------------------------------------------------------

    # a polling check for the status armable of the vehicle is required in order to arm it
    while not vehicle.is_armable:
        print("\tWaiting to initialize...".expandtabs(_id * 40))
        time.sleep(1)

    # setting the vehicle mode to GUIDED (required to take off) and arming motors
    print("\t-- Arming motors --".expandtabs(_id * 40))

    # timestamps on file -----------------------------------------------------------------------------------------------
    with open('log_data.txt', 'a') as the_file:
        the_file.write('-- Arming motors --  [%s]\n' % (strftime("%H:%M:%S", time.localtime())))
    # ------------------------------------------------------------------------------------------------------------------

    while not (vehicle.mode == VehicleMode("GUIDED") and vehicle.armed is True):
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True
        time.sleep(1)

    # a polling check for the status armed of the vehicle is required in order to take off
    while not vehicle.armed:
        print("\tWaiting for arming...".expandtabs(_id * 40))
        time.sleep(1)

    # taking off by calling the simple_takeoff() function
    print("\t-- Taking off --".expandtabs(_id * 40))

    # timestamps on file -----------------------------------------------------------------------------------------------
    with open('log_data.txt', 'a') as the_file:
        the_file.write('-- Taking off --  [%s]\n' % (strftime("%H:%M:%S", time.localtime())))
    # ------------------------------------------------------------------------------------------------------------------

    for j in range(1, 6):  # several consecutive repetitions of the command are needed to be sure it has been received
        vehicle.simple_takeoff(tgt_altitude)
        time.sleep(0.5)

    print_counter = 0

    # -- loop until target altitude is reached
    while True:

        if print_counter >= print_skip_iterations:
            logging.info('Altitude : ' + str(vehicle.location.global_relative_frame.alt))
            # timestamps on file -------------------------------------------------------------------------------------------
            with open('log_data.txt', 'a') as the_file:
                the_file.write(' Altitude: [%s]  [%s]\n' % (vehicle.location.global_relative_frame.alt,strftime("%H:%M:%S", time.localtime())))
            print_counter = 0

        # --------------------------------------------------------------------------------------------------------------

        altitudes = vehicle.location.global_relative_frame.alt

        if altitudes >= tgt_altitude * 0.8:  # acceptable altitude error: 10% of altitude
            print("\tAltitude reached".expandtabs(_id * 40))

            # timestamps on file ---------------------------------------------------------------------------------------
            with open('log_data.txt', 'a') as the_file:
                the_file.write(' Altitude reached [%s]\n' % (strftime("%H:%M:%S", time.localtime())))
            # ----------------------------------------------------------------------------------------------------------

            break
        print_counter += 1
        time.sleep(0.01)


def landing(vehicle, stop_action=[False], _id=0, log_level=logging.INFO):

    """
    Landing function - landing the vehicle
    :param vehicle: UAV to command
    :param log_level: logging level
    :return:
    """

    # set level of logging
    logging.basicConfig(format='\t[%(levelname)s] %(message)s'.expandtabs(_id * 40), level=logging.DEBUG)
    logging.getLogger().setLevel(log_level)

    print("\t-- Landing the drone --".expandtabs(_id * 40))

    # timestamps on file for starting time of function invocation ------------------------------------------------------
    with open('log_data.txt', 'a') as the_file:
        the_file.write('\n----- Drone ' + str(_id) + ' -- FUNCTION CALL: landing -----  [%s]\n\n' % (strftime("%H:%M:%S", time.localtime())))
    # ------------------------------------------------------------------------------------------------------------------

    for j in range(1, 3):  # several consecutive repetitions of the command are needed to be sure it has been received
        vehicle.mode = VehicleMode("LAND")
        time.sleep(0.5)

    print_counter = 0

    # -- loop until target altitude is reached
    while vehicle.armed:
        if print_counter >= print_skip_iterations:
            logging.info("Altitude: " + str(vehicle.location.global_relative_frame.alt))
            # timestamps on file -------------------------------------------------------------------------------------------
            with open('log_data.txt', 'a') as the_file:
                the_file.write(' Altitude: [%s]  [%s]\n' % (vehicle.location.global_relative_frame.alt,strftime("%H:%M:%S", time.localtime())))
            print_counter = 0

        # --------------------------------------------------------------------------------------------------------------

        print_counter += 1
        time.sleep(0.01)

    print("\tLanded with LAND".expandtabs(_id * 40))


def return_to_launch(vehicle, stop_action=[False], _id=0, log_level=logging.INFO):

    """
    RTL function - get the vehicle back to its home location and land
    :param vehicle: UAV  to command
    :param stop_action: flag, when set to True at runtime aborts the function
    :param _id: for stop_action. To be able to detect at runtime a change on stop_action, a mutable object has to be
                passed (in this case a list). Passing just a string or int will not work, as they are immutable objects
                in Python
    :param log_level: logging level
    :return:
    """

    # set level of logging
    logging.basicConfig(format='\t[%(levelname)s] %(message)s'.expandtabs(_id * 40), level=logging.DEBUG)
    logging.getLogger().setLevel(log_level)

    print("\t-- RTL the drone --".expandtabs(_id * 40))

    # timestamps on file for starting time of function invocation ------------------------------------------------------
    with open('log_data.txt', 'a') as the_file:
        the_file.write('\n----- Drone ' + str(_id) + '----- FUNCTION CALL: RTL -----  [%s]\n\n' % (strftime("%H:%M:%S", time.localtime())))
    # ------------------------------------------------------------------------------------------------------------------

    for j in range(1, 3): # several consecutive repetitions of the command are needed to be sure it has been received
        vehicle.mode = VehicleMode("RTL")
        time.sleep(0.5)

    print_counter = 0

    # -- loop until target altitude is reached
    while vehicle.armed:

        if stop_action[_id] is True:
            return

        if print_counter >= print_skip_iterations:
            logging.info("Altitude: " + str(vehicle.location.global_relative_frame.alt))
            # timestamps on file -------------------------------------------------------------------------------------------
            with open('log_data.txt', 'a') as the_file:
                the_file.write(' Altitude: [%s]  [%s]\n' % (vehicle.location.global_relative_frame.alt,strftime("%H:%M:%S", time.localtime())))
            print_counter = 0
        # --------------------------------------------------------------------------------------------------------------

        print_counter += 1
        time.sleep(0.01)

    print("\tLanded with RTL".expandtabs(_id * 40))


def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration, vehicle, stop_action=[False], _id=0, log_level=logging.INFO):

    """
    Move vehicle in direction based on specified velocity vectors
    :param velocity_x: x velocity in NED frame. Positive Forward
    :param velocity_y: y velocity in NED frame. Positive on the right
    :param velocity_z: z velocity in NED frame. Positive downward
    :param duration: move the UAV for 'duration' seconds
    :param vehicle: UAV to command
    :param stop_action: flag, when set to True at runtime aborts the function
    :param _id: for stop_action. To be able to detect at runtime a change on stop_action, a mutable object has to be
                passed (in this case a list). Passing just a string or int will not work, as they are immutable objects
                in Python
    :param log_level: logging level
    :return:
    """

    # set level of logging
    logging.basicConfig(format='\t[%(levelname)s] %(message)s'.expandtabs(_id * 40), level=logging.DEBUG)
    logging.getLogger().setLevel(log_level)

    logging.warning("-- Moving the drone with %s m/s forward - %s m/s on the right and %s m/s down --" % (str(velocity_x), str(velocity_y), str(velocity_z)))

    # timestamps on file for starting time of function invocation ------------------------------------------------------
    with open('log_data.txt', 'a') as the_file:
        the_file.write('\n----- Drone ' + str(_id) + '----- FUNCTION CALL: send_ned_velocity(%s,%s,%s) -----  [%s]\n\n' % (str(velocity_x), str(velocity_y), str(velocity_z), strftime("%H:%M:%S", time.localtime())))
    # ------------------------------------------------------------------------------------------------------------------

    # build the MAVLink command message for a velocity controlled movement
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame w.r.t to current heading of the drone
        #                                            (directions are considered w.r.t. current heading)
        #                                             MAV_FRAME_LOCAL_NED to specify w.r.t. direction
        #                                             parallel to North and East directions
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)     # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command on 10 Hz cycle (increase frequency for a more accurate movement, at the expense of performance)
    for x in range(0, duration*10):

        if stop_action[_id] is True:
            print("\tCAUTION: Command Aborted".expandtabs(_id * 40))
            return

        vehicle.send_mavlink(msg)

        # timestamps on file -------------------------------------------------------------------------------------------
        with open('log_data.txt', 'a') as the_file:
            the_file.write('\nTime : %s\n' % (strftime("%H:%M:%S", time.localtime())))
            the_file.write(' Global Location  (relative altitudes): ' + str(vehicle.location.global_relative_frame) + '\n')
            the_file.write(' Local Location : ' + str(vehicle.location.local_frame) + '\n')
            the_file.write(' Attitude : ' + str(vehicle.attitude) + '\n')
            the_file.write(' Velocity : ' + str(vehicle.velocity) + '\n')
            the_file.write(' Heading : ' + str(vehicle.heading) + '\n')
        # --------------------------------------------------------------------------------------------------------------

        time.sleep(0.1)


def get_distance_metres(aLocation1, aLocation2):

    """
    Returns the ground distance in metres between two LocationGlobal objects
    :param aLocation1: starting location
    :param aLocation2: ending location
    :return:
    """

    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    dlong_c = dlong*math.cos(math.radians(aLocation1.lat))
    return math.sqrt((dlat * dlat) + (dlong_c * dlong_c)) * 1.113195e5


def simple_goto_wait(wp, vehicle, stop_action=[False], _id=0, log_level=logging.INFO):

    """
    invoking the vehicle.simple_goto(wp1), displaying the remaining distance
    and ensuring to reach the target before proceeding with the code
    :param wp: waypoint for target location
    :param vehicle: UAV to command
    :param stop_action: flag, when set to True at runtime aborts the function
    :param _id: for stop_action. To be able to detect at runtime a change on stop_action, a mutable object has to be
                passed (in this case a list). Passing just a string or int will not work, as they are immutable objects
                in Python
    :param log_level: logging level
    :return:
    """

    # set level of logging
    logging.basicConfig(format='\t[%(levelname)s] %(message)s'.expandtabs(_id * 40), level=logging.DEBUG)
    logging.getLogger().setLevel(log_level)

    targetLocation = wp
    # timestamps on file for starting time of function invocation ------------------------------------------------------
    with open('log_data.txt', 'a') as the_file:
        the_file.write('\n----- Drone ' + str(_id) + '----- FUNCTION CALL: SIMPLE_GOTO_WAIT(%s) -----  [%s]\n\n' % (str(wp), strftime("%H:%M:%S", time.localtime())))
    # ------------------------------------------------------------------------------------------------------------------
    print('STOP ACTION = ' + str(stop_action))
    targetDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
    print_counter = 0

    for j in range(1, 8):
        vehicle.simple_goto(wp)
        time.sleep(0.5)

    # -- showing the remaining distance from the first waypoint and proceed when reached

    while vehicle.mode.name == "GUIDED":

        if stop_action[_id] is True:
            print("\tCAUTION: Command Aborted".expandtabs(_id * 40))
            return

        remainingDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        if print_counter >= print_skip_iterations:
            logging.info("Distance to target: " + str(remainingDistance))
            with open('log_data.txt', 'a') as the_file:
                the_file.write('Distance to target:' + str(remainingDistance)+ '\n')
            print_counter = 0

        # logging into csv GPS position
        # (commented, better to include in a listener as suggested below:
        #
        """
        @vehicle_1.on_message('SYSTEM_TIME')
        def listener(self, name, message):
            #print('\nListener: ' + str(self.location.global_relative_frame))
            #print('Listener: [GPS Time] ' + str(message.time_unix_usec))
            #print('Listener: [vehicle.time] ' + str(self.time) + '\n')
            #print('Listener: [Laptop Time] : %s\n' % (strftime("%H:%M:%S", time.localtime())))
            GPS_value[0] = str(self.location.global_relative_frame.lat)
            GPS_value[1] = str(self.location.global_relative_frame.lon)
            GPS_value[2] = str(self.location.global_relative_frame.alt)
            with open('GPS_log.csv', 'a') as the_file:
                the_file.write(timestamp + ';' + str(message.time_unix_usec) + ';' + str(
                    self.location.global_relative_frame.lat) + ';' + str(
                    self.location.global_relative_frame.lon) + ';' + str(
                    self.location.global_relative_frame.alt) + '\n')  # timestamp_laptop,timestamp_arducopter,gps_data (lat, lon, alt)
        """
        #
        # )
        """
        with open('GPS_log.csv', 'a') as the_file:
            the_file.write(strftime("%H:%M:%S", time.localtime()) + ';' + str(float(
                vehicle.location.global_relative_frame.lat)) + ';' + str(float(
                vehicle.location.global_relative_frame.lon)) + ';' + str(float(
                vehicle.location.global_relative_frame.alt)) + '\n')  # timestamp_laptop,timestamp_arducopter,gps_data (lat, lon, alt)
        """

        # if the target position is within a minimum_distance, set as target reached
        if remainingDistance <= mininum_distance:
            print("\tTarget reached".expandtabs(_id * 40))
            return

        print_counter += 1
        time.sleep(0.05)


def load_path(path):

    """
    load a .waypoints file, excluding the first two lines (ID file line and home location line)
    :param path: path to the .waypoints file
    :return:
    """

    firstline = True
    latitude = []
    longitude = []
    altitude = []
    condition_yaw_ = []
    for line in open(path, 'r'):
        # ---------------------------------------------------------------------------------------------------------
        if firstline is True:  # discard the first line of text
            firstline = False
            continue
        # -- parsing each line into latitude, longitude and altitude, index 0 is the home location
        s = line.rstrip()
        result = s.split('\t')
        latitude.append(result[8])
        longitude.append(result[9])
        altitude.append(result[10])
        condition_yaw_.append(result[11])
    return (latitude, longitude, altitude, condition_yaw_)


def condition_yaw(heading, vehicle, stop_action=[False], _id=0, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

def set_roi(location, vehicle, stop_action=[False], _id=0):
    # create the MAV_CMD_DO_SET_ROI command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
        0, #confirmation
        0, 0, 0, 0, #params 1-4
        location.lat,
        location.lon,
        location.alt
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)


# ------------------------------------------- FUNCTIONS NEVER TESTED ---------------------------------------------------


def find_wind(vehicle):

    """
    sense the wind direction
    :param vehicle: UAV to command
    :return:
    """

    vehicle.location.global_relative_frame = startingLocation
    vehicle.mode = VehicleMode("ALT_HOLD")  # ALT HOLD mode keep altitude, but the drone roll, pitch, yaw and position
    #                                         free to change.
    #                                         LOITER mode keep altitude and position (including heading)
    #                                         STABILIZE mode is the more free mode. suited for manual guide

    print("Floating...")
    for index in range(1, 11): # note: the range(min, max) count from min up to max-1
        print("%i " % index)
        print("%s" % vehicle.attitude)
        print("%s" % vehicle.location.global_frame)
        print("%s" % vehicle.location.global_relative_frame)
        print("%s" % vehicle.location.local_frame)
        time.sleep(1)

    print("\n")
    print("Wind Speed [computed over distance]: %s m/s" % (get_distance_metres(vehicle.location.global_relative_frame, startingLocation) / 10))
    print("\n")
    print("Wind Speed [computed from ground/airspeed] : %s m/s" % (vehicle.groundspeed - vehicle.airspeed))
    vehicle.mode = VehicleMode("GUIDED")
    print("Coming Back to Target")
    vehicle.simple_goto(startingLocation)
    print("\n")
    # CAUTION: !THIS METHOD IS UNRELIABLE, NOT TESTED YET, MAY NOT EVEN WORK PROPERLY!
    print("Drone Wind Direction: %s deg from North" % (vehicle.heading + 180))  # when coming back to its previous
    #                                                                             position, the drone heading indicates
    #                                                                             the wind direction + 180 degrees
    print("\n")
    time.sleep(5)  # 5 secs to come back after floating
# ----------------------------------------------------------------------------------------------------------------------
