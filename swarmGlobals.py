# ----------------------------------------------- Imports ------------------------------------------------------------ #

import logging
import re

# ---------------------------------------- Constants declaration ----------------------------------------------------- #

uav_n = 1				  # number of drones in the swarm - 1
ctrl_l = 0				  # swarm control law identifier
fname = "droneSwarmMng"	  # name of the log file
log_l = logging.WARNING	  # log level
ver = 'v0.1'			  # program version
base_ip = "127.0.0.1"	  # base ip address for ad-hoc network. previously: "192.168.0."
init_ip = 2               # last octet for ip used by drones, to obtain an ipv4 address base_ip.init_ip. previously: 101
base_ctrl_port = 14550	  # base ip:port from mavlink
base_sens_port = 5760	  # base ip:port for sensing task
valid_pattern = re.compile('(?<=drone)\d+')  # valid pattern for a keyboard input command, i.e. droneX,something,else
detect_task = re.compile('(?<=:)\w+')
drone_missions_directory = '../drone_missions'  # path to the drone missions folder, storing all the txt files for
#                                                 pre-programmed drone paths

