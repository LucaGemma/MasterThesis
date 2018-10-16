# ----------------------------------------------- Imports ------------------------------------------------------------ #

import time, sys, os, logging, datetime, select
import threading, multiprocessing, argparse 
import subprocess, dronekit, pymavlink, random
import aDroneIF, swarmManager
from logger import swarm_logger
from swarmGlobals import uav_n, ctrl_l, fname, log_l, ver, base_ip, init_ip, base_ctrl_port, base_sens_port, valid_pattern, detect_task
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from custom_library import drone_lib
import re

# ----------------------------------------- Additional Functions ----------------------------------------------------- #


def runCommand(command):

    """
    elaborate input command to detect a valid command or an exit command

    :param command: input command to be elaborated
    :return: the function returns a dictionary with the type of command and the raw typed command, made as follows:

    {'res':res, 'command':command}
    """

    print "!!!!!!!entering the runCommand Function!!!!!!"
    # check if a valid drone command has been typed (a valid command is made of: droneX,something,else)
    splitz = ['']
    res = 0
    current_task = re.findall(valid_pattern, str(command))
    for p in current_task:
        splitz = p.split()
        print "typed command to drone : {}" .format(splitz)
    if len(splitz) > 0 and splitz != ['']:
        res = 2
        print "detected valid command"
    # check if no string has been typed
    elif command == '' or command == '\n' or command == '\r':
        print "nothing typed, pass"
        pass
    # check if an exit command has been typed
    elif command == 'exit' or command == "exit":
        res = 1
        print "exit typed"
    # else, print the help
    else:
        print '-----------------\n\r',\
            'available command\n\r',\
            'exit - closes sockets and exit\n\r', \
            'droneX,action,altitude,latitude,longitude,v_x,v_y,v_z - give drone number X a command\n\r', \
            '-----------------\n\r'
        pass

    """
    -- TO DO: insert global command support. I.e. if the user types 'land' and not 'droneX,land', the land command
                has to be given to every drone
    elif command == 'land':
        res = 3
    elif command == 'moveforward':
        res = 4
    elif command == 'rtl':
        res = 5
    """

    return {'res': res, 'command': command}


# ---------------------------------------------- Main Script --------------------------------------------------------- #

if __name__ == "__main__":

    mgr = multiprocessing.Manager()
    drone_id_list = mgr.list()			# shared list of drone ids
    drone_evnt_list = mgr.list()        # shared list of drone events
    drone_d1_list = mgr.list()			# shared list for data exchange
    drone_d2_list = mgr.list()          # shared list for data exchange
    action_list = mgr.list()            # shared list of drone action dictionaries
    stop_action = mgr.list()            # shared list of flag, one for each drone
    #                                     True if the current action has to be aborted immediately
    d_l = []	 						# list of drone objects managed by main process
    g_var = mgr.Namespace()
    g_var.running = False				# True if the main loop is running, False elsewhere
    g_var.CONTINUE = 0                  # type of typed keyboard input
    g_var.CURRENT_TASK = ''             # raw typed keyboard input

    # add parsing input options
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--number_d', type=int, action='store', dest='number_d',
                         help='Set the number of drones in the swarm', default=3)
    parser.add_argument('-c', '--ctrl_law', type=int, action='store', dest='ctrl_law',
                     help='Select the swarm control law, 0:mapping (default), 1:fleet', default=0)
    parser.add_argument('-l', '--log_level', type=int, action='store', dest= 'loglevel',
                         help='Define log level: 2-debug, 1-info, default warning', default=logging.INFO)
    parser.add_argument('--version', action='version', version='%(prog)s '+ver)
    param_list = parser.parse_args()

    # initialize paring input dependent variables
    if param_list.loglevel == 2:
        log_l = logging.DEBUG
    elif param_list.loglevel == 1:
        log_l = logging.INFO
    elif param_list.loglevel == 0:
        log_l = logging.WARNING
    else:
        log_l = logging.INFO
    logging.info('Set log level to: %s', log_l)

    uav_n = param_list.number_d
    swarm_logger.info('Set size of swarm to: %s', uav_n)
    ctrl_l = param_list.ctrl_law
    swarm_logger.info('Set control law to: %s', ctrl_l)

    # initialize the remaining variables and all the aDroneIF processes and start them
    print '[INFO] Starting %s %s with params %d %d %d' % (fname, ver, log_l, uav_n, ctrl_l)
    time.sleep(1)

    try:
        with open("null.txt", "w") as null_f:
            for ping in range(uav_n):
                """
                # ping each drone to see if they've joined the network
                address = base_ip + str(ping+init_ip)
                res = subprocess.call(['ping', '-n', '1', address], stdout=null_f, stderr = null_f )
                if res == 0:
                    print "[INFO] ping to", address, "OK"
                    swarm_logger.info("ping to %s : OK", address)
                    d = aDrone(ping,args=(1,2,3))
                    drone_id_list.append(d)
                    d.start()
                elif res == 2:
                    print "[INFO] no response from", address
                    swarm_logger.info("ping to %s : NO-RESP", address)
                else:
                    print "[INFO] ping to", address, "failed!"
                    swarm_logger.info("ping to %s : FAIL", address)
                """

                # initialize every declared variable
                drone_evnt_list.append(0)
                drone_d1_list.append(0)
                drone_d2_list.append(0)
                stop_action.append(False)
                action_list.append({'action': 'action complete', 'altitude': '', 'latitude': '', 'longitude': '', 'v_x':'', 'v_y':'', 'v_z':''})

                # initialize and start aDroneIF processes
                d = aDroneIF.aDroneIF(log_l, base_ip, init_ip, base_ctrl_port, base_sens_port, ping, g_var, drone_evnt_list, drone_d1_list, drone_d2_list, action_list, stop_action)
                d_l.append(d)
                drone_id_list.append(ping)
                d.start()
                time.sleep(1)

        # initialize and start the swarmManager process
        sm_p = swarmManager.swarmManager(log_l, uav_n, g_var, drone_id_list, drone_evnt_list, drone_d1_list, drone_d2_list, action_list, stop_action)
        sm_p.start()

        # start the main endless loop, exit only when 'exit' is typed on keyboard
        g_var.running = True

        while g_var.running:
            in_c = sys.stdin.readline().strip()
            if not (in_c == '' or in_c == ""):
                swarm_logger.info("input: %s", in_c)
                temp_command = runCommand(in_c)
                g_var.CONTINUE = temp_command['res']

                # if 'exit' is typed
                if g_var.CONTINUE == 1:
                    g_var.running = 0
                # if a valid drone command is typed
                elif g_var.CONTINUE == 2:
                    g_var.CURRENT_TASK = temp_command['command']
            time.sleep(0.1)

    except KeyboardInterrupt:
        g_var.running = 0
        swarm_logger.warning('Keyboard interrupt! Safely Closing')
        print("Keyboard Interrupt! Safely Closing")

    except Exception, e:
        print 'Exception', e

    # close all aDroneIF processes
    for i in d_l:
        i.join()

    # close the swarmManager process
    sm_p.join()

    swarm_logger.info("Done")
    print 'End'

