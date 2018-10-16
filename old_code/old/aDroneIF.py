## Import

import time, sys, os, logging, datetime, select
import threading, multiprocessing, argparse 
import subprocess, dronekit, pymavlink, random
from logger import swarm_logger
from custom_library import drone_lib
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal

## Drone object definition ##

class aDroneIF( multiprocessing.Process ):
    def __init__(self, log_l, base_ip, init_ip, base_ctrl_port, base_sens_port, id, ns, d_e_l, d1, d2, action_list, stop_action, group = None, target = None, name = "aDrone", args = (), kwargs = {}):
        # save inputs to local variables
        self.log_l = log_l
        self.name = name
        self.myid = id
        # -------------------------------- MODIFICA QUANDO L'IP E' DIVERSO ---------------------------------------------
        # self.myip = base_ip+str(id+init_ip)
        self.myip = base_ip
        # self.my_control_port = str(base_ctrl_port+init_ip)
        self.my_control_port = str(base_ctrl_port + id * 10 + 1)
        # --------------------------------------------------------------------------------------------------------------
        self.my_sens_port = str(base_sens_port+init_ip+id)
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
        swarm_logger.info('%s.%s starting', self.name, self.myid)
        self.vehicle = connect(self.myip + self.my_control_port, wait_ready=True)
        # wait until main loop initialization completes
        while not self.ns.running:
            time.sleep(0.25)
            continue
        # start
        swarm_logger.info('%s.%s running', self.name, self.myid)
        while self.ns.running: # implement drone management
            #self.de_l() random.randint()
            # implement data collection from uav
            # GPS + flight parameters
            # sensors' data

            print("aDroneIF dictionary: ", self.action[self.myid])
            self.action[self.myid] = perform_action(self.action[self.myid], self.vehicle, self.stop_action[self.myid])

            time.sleep(0.3)
            self.d1[self.myid] = random.randint(1, 10)
            self.sens_dict['gas0'] = random.randint(11, 20)
            self.sens_dict['gas1'] = random.randint(11, 20)
            self.sens_dict['gas2'] = random.randint(11, 20)
            self.d2[self.myid] = self.sens_dict
            self.de_l[self.myid] = 1
            swarm_logger.debug('%s.%s - d1:%s d2:%s de_l:%s', self.name, self.myid, self.d1[self.myid], self.d2[self.myid], self.de_l[self.myid])
            if self.log_l == logging.DEBUG: print "[DEBUG] %s.%s - d1:%s d2:%s de_l:%s" % (self.name, self.myid, self.d1[self.myid], self.d2[self.myid], self.de_l[self.myid])
        swarm_logger.info('%s.%s closing', self.name, self.myid)
        if self.log_l >= logging.DEBUG: print "[INFO] %s.%s closing" % (self.name, self.myid)
        return

## ------ Class End ------ ##


def perform_action(action, vehicle, stop_action):
    if action['action'] == "take-off":
        drone_lib.arm_and_takeoff(action['altitude'], vehicle)
        # time.sleep(1)
        temp = action
        temp['action'] = "action complete"
    elif action['action'] == "land":
        drone_lib.landing(vehicle)
        # time.sleep(1)
        temp = action
        temp['action'] = "action complete"
    elif action['action'] == "rtl":
        drone_lib.return_to_launch(vehicle, stop_action)
        # time.sleep(1)
        temp = action
        temp['action'] = "action complete"
    elif action['action'] == "go-to":
        standard_altitude = 8
        wp = LocationGlobalRelative(float(action['latitude']),
                                    float(action['longitude']),
                                    float(action['altitude']))
        drone_lib.simple_goto_wait(wp, vehicle, stop_action)
        # time.sleep(1)
        temp = action
        temp['action'] = "action complete"
    elif action['action'] == "move forward":
        drone_lib.send_ned_velocity(1, 0, 0, 5, vehicle, stop_action)
        # time.sleep(1)
        temp = action
        temp['action'] = "action complete"
    else:
        temp = action
    return temp