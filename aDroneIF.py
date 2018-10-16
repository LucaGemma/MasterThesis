# ----------------------------------------------- Imports ------------------------------------------------------------ #

import time, sys, os, logging, datetime, select
import threading, multiprocessing, argparse 
import subprocess, dronekit, pymavlink, random
from logger import swarm_logger
from custom_library import drone_lib
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal

# ---------------------------------------- Drone object definition --------------------------------------------------- #


class aDroneIF(multiprocessing.Process):

    """
    Class which represents a drone interface. Connects to a physical UAV and sends MAVLink commands to it using
    DroneKit functions through the drone_lip.py custom library
    log_l : log level
    name : name of the class
    myid : unique id for the current UAV/drone interface
    my_control_port : communication port for sending MAVLink commands to the UAV
    my_sens_port : communication port for TCP streaming of data from the UAV to ground
    ns : variable with multiple field -
            ns.running -> True or False depending if the main loop of droneSwarmMng.py is running or not
            ns.CONTINUE -> type of keyboard input received: 0 -> no input, 1 -> 'exit', 2 -> detected drone command
            ns.CURRENT_TASK -> store the keyboard input
    de_l : unused
    d1 : unused, dictionary for data
    d2 : unused, dictionary for data
    action : list of dictionaries, one dictionary for each UAV, storing their pending actions and made as follows:

                {'action': 'take-off', 'altitude': '10', 'latitude': '', 'longitude': '', 'v_x':'', 'v_y':'', 'v_z':''}

    stop_action : flag that is dynamically set to True when the current action has to be aborted
    """

    def __init__(self, log_l, base_ip, init_ip, base_ctrl_port, base_sens_port, id, ns, d_e_l, d1, d2, action_list, stop_action, group = None, target = None, name = "aDrone", args = (), kwargs = {}):

        """
        Initialize internal parameters and call __init__ function of the parent class
        """

        # save inputs to local variables
        self.log_l = log_l
        self.name = name
        self.myid = id
        self.myip = base_ip
        self.my_control_port = str(base_ctrl_port + id * 10 + 1)
        self.my_sens_port = str(base_sens_port + id * 10 + 1)
        self.ns = ns
        self.de_l = d_e_l
        self.d1 = d1
        self.d2 = d2
        self.action = action_list
        self.stop_action = stop_action
        self.current_ps = 0
        self.next_ps = 0
        self.sens_dict = {'gas0': 0, 'gas1': 1, 'gas2': 2}
        swarm_logger.info('%s.%s init with ip: %s:%s|%s', self.name, self.myid, self.myip, self.my_control_port, self.my_sens_port)
        if self.log_l >= logging.DEBUG: print '[INFO] %s.%s init with ip: %s:%s|%s' % (self.name, self.myid, self.myip, self.my_control_port, self.my_sens_port)
        # open sockets towards corresponding UAV
        # control
        # self.vehicle = dronekit.connect(self.myip+":"+self.my_control_port, wait_ready=True)
        # sensing
        # launch a thread that manages sensing data
        super(aDroneIF, self).__init__(group, target, name, args, kwargs)

    def run(self):

        """
        Actions to be performed while the process aDroneIF is running:
        1) Connect to the UAV
        While the main loop of droneSwarmMng.py is running:
         2) Perform pending action
         3) Output received data
        """

        swarm_logger.info('%s.%s starting', self.name, self.myid)

        # 1) Connect to the UAV ----------------------------------------------------------------------------------------
        self.vehicle = connect("udp:" + self.myip + ":" + self.my_control_port, wait_ready=True)
        # --------------------------------------------------------------------------------------------------------------

        # wait until main loop initialization completes
        while not self.ns.running:
            time.sleep(0.25)
            continue

        # start
        swarm_logger.info('%s.%s running', self.name, self.myid)
        while self.ns.running:  # implement drone management
            #self.de_l() random.randint()
            # implement data collection from uav
            # GPS + flight parameters
            # sensors' data

            print("aDroneIF dictionary: ", self.action[self.myid])

            # 2) Perform pending action --------------------------------------------------------------------------------
            result = perform_action(self.action[self.myid], self.vehicle, self.stop_action, self.myid)

            if result != '':
                # perform_action() returns '' if the action was aborted due to changing of the stop_action value
                # in such a case, the dictionary "action" of the current action will not be updated
                self.action[self.myid] = result
            # ----------------------------------------------------------------------------------------------------------

            # 3) Output received data ----------------------------------------------------------------------------------
            time.sleep(0.3)
            self.d1[self.myid] = random.randint(1, 10)
            self.sens_dict['gas0'] = random.randint(11, 20)
            self.sens_dict['gas1'] = random.randint(11, 20)
            self.sens_dict['gas2'] = random.randint(11, 20)
            self.d2[self.myid] = self.sens_dict
            self.de_l[self.myid] = 1
            swarm_logger.debug('%s.%s - d1:%s d2:%s de_l:%s', self.name, self.myid, self.d1[self.myid], self.d2[self.myid], self.de_l[self.myid])
            if self.log_l == logging.DEBUG: print "[DEBUG] %s.%s - d1:%s d2:%s de_l:%s" % (self.name, self.myid, self.d1[self.myid], self.d2[self.myid], self.de_l[self.myid])
            # ----------------------------------------------------------------------------------------------------------
        swarm_logger.info('%s.%s closing', self.name, self.myid)
        if self.log_l >= logging.DEBUG: print "[INFO] %s.%s closing" % (self.name, self.myid)
        return

# -------------------------------------------------- Class End ------------------------------------------------------- #

# --------------------------------------------- Additional Functions ------------------------------------------------- #


def perform_action(action, vehicle, stop_action, _id):

    """
    make the vehicle perform the action, aborting when stop_action becomes True

    :param action: dictionary for the action to be performed, made as follows:

    {'action': 'take-off', 'altitude': '10', 'latitude': '', 'longitude': '', 'v_x':'', 'v_y':'', 'v_z':''}

    :param vehicle: vehicle object for sending MAVLink commands through DroneKit functions, using drone_lib.py library
    :param stop_action: flag that is dynamically set to True when the current action has to be aborted
    :param _id: id for the current aDroneIF object (i.e. the current vehicle for which perform_action() is invoked)
    :return: the function returns the new dictionary for the aDroneIF object or '' if the action was aborted
    """

    log = logging.INFO

    if action['action'] == "take-off":
        drone_lib.arm_and_takeoff(action['altitude'], vehicle, log)
        if stop_action[_id] is False:
            temp = action
            temp['action'] = "action complete"
        else:
            temp = ''
    elif action['action'] == "land":
        drone_lib.landing(vehicle, log)
        if stop_action[_id] is False:
            temp = action
            temp['action'] = "action complete"
        else:
            temp = ''
    elif action['action'] == "rtl":
        drone_lib.return_to_launch(vehicle, stop_action, _id, log)
        if stop_action[_id] is False:
            temp = action
            temp['action'] = "action complete"
        else:
            temp = ''
    elif action['action'] == "go-to":
        wp = LocationGlobalRelative(float(action['latitude']),
                                    float(action['longitude']),
                                    float(action['altitude']))
        drone_lib.simple_goto_wait(wp, vehicle, stop_action, _id, log)
        if stop_action[_id] is False:
            temp = action
            temp['action'] = "action complete"
        else:
            temp = ''
    elif action['action'] == "move forward":
        drone_lib.send_ned_velocity(1, 0, 0, 5, vehicle, stop_action, _id, log)
        if stop_action[_id] is False:
            temp = action
            temp['action'] = "action complete"
        else:
            temp = ''
    else:
        temp = action

    # if it entered with a stop_action True, the action triggering the flag is now performed, thus reset the flag
    if stop_action[_id] is True:
        stop_action[_id] = False

    return temp

