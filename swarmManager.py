# ----------------------------------------------- Imports ------------------------------------------------------------ #

import time, sys, os, logging, datetime, select, re
import threading, multiprocessing, argparse 
import subprocess, dronekit, pymavlink, random
from logger import swarm_logger
from custom_library import drone_lib
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from swarmGlobals import uav_n, ctrl_l, fname, log_l, ver, base_ip, init_ip, base_ctrl_port, base_sens_port, valid_pattern, detect_task, drone_missions_directory

# ----------------------------------- swarmManager object definition ------------------------------------------------- #


class swarmManager(multiprocessing.Process):

    """
    Class which represents the drone swarm manager. Load all the pre-programmed paths and build the proper set of
    dictionaries for every drone interface. Then, periodically update their dictionaries with the new pending actions
    and also manage for keyboard input commands. Keyboard commands have the highest priority and will be executed
    before any other pending action, even aborting the current one, in order to be executed immediately.
    log_l : log level
    uav_n : number of UAV in the swarm
    ns : variable with multiple field -
            ns.running -> True or False depending if the main loop of droneSwarmMng.py is running or not
            ns.CONTINUE -> type of keyboard input received: 0 -> no input, 1 -> 'exit', 2 -> detected drone command
            ns.CURRENT_TASK -> store the keyboard input
    d1 : unused
    de_l : unused
    d1 : unused, dictionary for data
    d2 : unused, dictionary for data
    action : list of dictionaries, one dictionary for each UAV, storing their pending actions and made as follows:

                {'action': 'take-off', 'altitude': '10', 'latitude': '', 'longitude': '', 'v_x':'', 'v_y':'', 'v_z':''}

    stop_action : flag that is dynamically set to True when the current action has to be aborted
    name : name of the class
    drones_actions_list : list of lists of dictionaries, one list of lists for every UAV, storing all the programmed
                          actions for that UAV
    """

    def __init__(self, log_l, uav_n, ns, d_l, d_e_l, d1, d2, action_list, stop_action, group = None, target = None, name = "swarmManager", args = (), kwargs = {}):

        """
        Initialize internal parameters and call __init__ function of the parent class
        """

        self.log_l = log_l
        self.uav_n = uav_n
        self.ns = ns
        self.dl = d_l
        self.de_l = d_e_l
        self.d1 = d1
        self.d2 = d2
        self.action = action_list
        self.stop_action = stop_action
        self.name = name
        self.drones_actions_list = []
        for i in range(self.uav_n):
            self.drones_actions_list.append([])

        swarm_logger.info('[%s] initialized', self.name)
        super(swarmManager, self).__init__(group, target, name, args, kwargs)

    def run(self):

        """
        Actions to be performed while the process swarmManager is running:
        1) Load all the pre-programmed paths from text files
        While the main loop of droneSwarmMng.py is running:
         2) Detect a keyboard input UAV command, abort current action for such UAV and perform the k. input command
         3) Update actions for every UAV
        """

        swarm_logger.info('[%s] starting', self.name)
        # wait until main loop initialization completes
        while not self.ns.running:
            time.sleep(0.25)
            continue
        # running control loop
        swarm_logger.info('[%s] running', self.name)
        current_drone = -1

        # 1) Load all the pre-programmed paths from text files ---------------------------------------------------------
        for i in range(self.uav_n):
            for line in open(drone_missions_directory + '/drone_[' + str(i) + ']_actions.txt', 'r'):
                s = line.rstrip()
                current_action = s.split(",")
                if current_action[1] == 'load_path':
                    firstline = True
                    (latitude, longitude) = drone_lib.load_path(str(current_action[2]))
                    for z in range(len(latitude)):
                        # discard first value, for it is the home location chosen when the waypoints file was created
                        if z == 0:
                            continue
                        s = 'drone' + str(i) + ',' + 'go-to,' + str(altitude) + ',' + str(latitude[z]) + ',' + str(longitude[z])
                        self.drones_actions_list[i].append(s)
                else:
                    self.drones_actions_list[i].append(s)
                    if current_action[1] == 'take-off':
                        altitude = current_action[2]
        # --------------------------------------------------------------------------------------------------------------

        while self.ns.running:
            """
            #print "[DEBUG] %s working" % (self.name)
            ## use the following to send messages to drones'ip directly from the manager
            #for d in self.dl:
            #	print "[INFO] Sending msg to %s" % (base_ip+str(d))
            ## use the following to send messages to drone processes
            #for ev in self.de_l:
            #	if ev is not 0:
            #		self.d1 = random.randint(1, 10)
            """

            # 2) Detect a keyboard input UAV command, abort current action for such UAV and perform the k. input command
            if self.ns.CURRENT_TASK != '':
                current_task = re.findall(valid_pattern, str(self.ns.CURRENT_TASK))
                split0 = None
                for p in current_task:
                    split0 = p.split()
                if split0 is not None:
                    current_drone = int(split0[0])
                    if current_drone > self.uav_n-1:
                        print "Invalid Drone Number"
                        self.ns.CURRENT_TASK = ''
                        current_drone = -1
                    else:
                        self.stop_action[current_drone] = True
            # ----------------------------------------------------------------------------------------------------------

            # 3) Update actions for every UAV --------------------------------------------------------------------------
            for i in range(self.uav_n):
                if self.stop_action[i] is True:
                    self.update_action(i, True)
                else:
                    if len(self.drones_actions_list[i]) > 0:
                        self.update_action(i, False, True)
            current_drone = -1
            # ----------------------------------------------------------------------------------------------------------
            time.sleep(0.1)

        # set the stop action of all UAV to True
        for i in range(self.uav_n):
            self.stop_action[i] = True
        swarm_logger.info('[%s] closing', self.name)
        if self.log_l >= logging.DEBUG: print "[INFO][%s] closing" % (self.name)
        return

    def update_action(self, i, kb_command = False, pending_action = False):

        """
        update the action dictionary for the i-th drone interface.
        If a keyboard command has been typed -> asynchronously update the drone action dictionary
        else -> if the drone is not busy, update its action dictionary

        :param i: drone interface to be updated
        :param kb_command: flag, True when a keyboard command has been typed for the current drone
        :param pending_action: flag, True when a pending action is available for the current drone
        :return: the function returns nothing
        """

        # if a keyboard command has been typed for the current drone
        if kb_command:
            if self.ns.CURRENT_TASK != '':
                print('ENTERED IN UPDATE_ACTION WITH KEYBOARD INTERRUPT')
                self.de_l[i] = 0
                current_task = self.ns.CURRENT_TASK.split(",")
                temp = self.action[i]
                print('current task: ' + str(current_task))
                temp['action'] = current_task[1]
                k = 2
                while k < len(current_task):
                    if k == 2:
                        temp['altitude'] = float(current_task[2])
                    elif k == 3:
                        temp['latitude'] = current_task[3]
                    elif k == 3:
                        temp['longitude'] = current_task[4]
                    elif k == 3:
                        temp['v_x'] = float(current_task[5])
                    elif k == 3:
                        temp['v_y'] = float(current_task[6])
                    elif k == 3:
                        temp['v_z'] = float(current_task[7])
                    k = k + 1
                self.action[i] = temp
                self.ns.CURRENT_TASK = ''

        # if the drone is not busy
        elif self.action[i]['action'] == "action complete":

            # if there is an available pending action
            if pending_action:
                current_action = self.drones_actions_list[i].pop(0)
                _current_action = current_action.split(",")
                temp = self.action[i]
                temp['action'] = _current_action[1]
                k = 2
                while k < len(_current_action):
                    if k == 2:
                        temp['altitude'] = float(_current_action[2])
                    elif k == 3:
                        temp['latitude'] = _current_action[3]
                    elif k == 4:
                        temp['longitude'] = _current_action[4]
                    elif k == 5:
                        temp['v_x'] = float(_current_action[5])
                    elif k == 6:
                        temp['v_y'] = float(_current_action[6])
                    elif k == 7:
                        temp['v_z'] = float(_current_action[7])
                    k = k + 1
                self.action[i] = temp
            swarm_logger.debug('[%s] drone:%s d1:%s d2:%s', self.name, i, self.d1[i], self.d2[i])
            if self.log_l == logging.DEBUG:
                print "[DEBUG][%s] drone:%s d1:%s d2:%s" % (self.name, i, self.d1[i], self.d2[i])
                print "[DEBUG][%s] drone:%s d2.gas0:%s" % (self.name, i, self.d2[i]['gas0'])
                print "[DEBUG] ACTION: drone %s action: %s " % (i, self.action[i])

        return


# -------------------------------------------------- Class End ------------------------------------------------------- #


