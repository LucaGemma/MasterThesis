# ----------------------------------------------- Imports ------------------------------------------------------------ #

import logging
import re
import threading

# ---------------------------------------- Constants declaration ----------------------------------------------------- #

uav_n = 1				  # number of drones in the swarm - 1
ctrl_l = 0				  # swarm control law identifier
z = 15
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
min_alt = 1.0            # minimum settable altitude
altitude_levels = [10, 16]
thresholds_dictionary = {'threshold_distance_max': 10, 'threshold_distance_medium': 5 , 'threshold_distance_min': 2, 'critical_temperature' : 30}

# Regular expression patterns for parsing incoming data
# compile a pattern, in this case: match every occurrence of a float number x in a "..0 : x .. hours:min:sec" pattern
pattern_gas_generico_0 = re.compile('(?<= 0 : )[-+]?\d+\.\d* \d+[:]\d+[:]\d+')
pattern_gas_NH3 = re.compile('(?<=NH3 : )[-+]?\d+\.\d* \d+[:]\d+[:]\d+')
pattern_gas_NO2 = re.compile('(?<=NO2 : )[-+]?\d+\.\d* \d+[:]\d+[:]\d+')
pattern_gas_C3H8 = re.compile('(?<=C3H8 : )[-+]?\d+\.\d* \d+[:]\d+[:]\d+')
pattern_gas_C4H10 = re.compile('(?<=C4H10 : )[-+]?\d+\.\d* \d+[:]\d+[:]\d+')
pattern_gas_CH4 = re.compile('(?<=CH4 : )[-+]?\d+\.\d* \d+[:]\d+[:]\d+')
pattern_gas_H2 = re.compile('(?<=H2 : )[-+]?\d+\.\d* \d+[:]\d+[:]\d+')
pattern_gas_C2H5O5 = re.compile('(?<=C2H5OH : )[-+]?\d+\.\d* \d+[:]\d+[:]\d+')
pattern_gas_CO_i2c = re.compile('(?<=CO : )[-+]?\d+\.\d* \d+[:]\d+[:]\d+')
pattern_TEMP = re.compile('(?<=TEMP : )[-+]?\d+\.\d* \d+[:]\d+[:]\d+')
pattern_gas_CO2_uart = re.compile('(?<=CO2 : )[-+]?\d+\.\d* \d+[:]\d+[:]\d+')

# Parameters initialization -------------------------------------------------------------------------------------------
global filename
global command
global data_ready
global plot_counter
global flag
global GPS_value
BUFFER_LENGTH = 512  # already tried: 256, 1024
sleep_time_between_communications = 0.05  # previously: 0.1
sleep_time = 0.1                # sleep time between each iteration of the main loop
gas_pairing_refresh_rate = 2  # 0.5 # sleep time b/w each call of the GPS_gas_pairing thread
data_ready = threading.Event()
gas_type = ['CO - [ADC]', 'NH3 - [I2C]', 'NO2 - [I2C]', 'C3H8 - [I2C]', 'C4H10 - [I2C]', 'CH4 - [I2C]',
            'H2 - [I2C]', 'C2H5O5 - [I2C]', 'CO - [I2C]', 'TEMP', 'CO2 - [UART]']
gas_value = []  # 0 is CO, then, respectively: 1 - NH3, NO2, C3H8, C4H10, CH4, H2, C2H5OH, CO [I2C]
#                 list of 8 tuples with (sum of total gas readings up to now, # of total gas readings, gas type)
#                 each tuple is updated as soon as a reading is available
x_data, y_data, plot_counter = [], [], []
width_of_x_axis = 100
# for i in range(11):
#     gas_value.append([-1.0, 0, 'Unavailable'])
#     y_data.append([])
#     x_data.append([])
#     plot_counter.append(0)
GPS_value = ['not available', 'not available', 'not available']
limy = 2
limy_NH3 = 2  # useful for setting the maximum value on the plot axis
limy_NO2 = 2
limy_C3H8 = 2
limy_C4H10 = 2
limy_CH4 = 2
limy_H2 = 2
limy_C2H5OH = 2
start = 0
absolute_pressure = 0
temperature = 0
split0 = [0]

TEMP_LIMIT = 50  #limit of temperature at which the drone activate the RTL
BATT_LIMIT = -100  #limit of battery percentage at with the drone activate the RTL

# -- Multiple queues: dataq stores the data from first tcp/ip connection (analog sensor)
#                    dataq2 stores the data from second tcp/ip connection (i2c sensor)

data_ready.clear()
flag = True

map_refreshrate = 7

waypoints_wait_time = [15, 15, 15, 15] #sleep time after each action

take_off_wait_time = [2.5, 0, 0, 0] #take-off time sleep
