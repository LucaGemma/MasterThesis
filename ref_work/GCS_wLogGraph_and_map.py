# GCS script: acquire gas measures, displays them on graphs and create an html geographical map
# Last Update: 09 - 06 - 2018 -- Luca Gemma, University of Trento - Italy

#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import socket, re, time, argparse, threading, sys, os, datetime, Queue
import matplotlib.pyplot as plt
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from time import gmtime, strftime
from custom_library import drone_lib
import csv
from gmplot import gmplot
from selenium import webdriver
import urllib
import urllib2
import dronekit


global filename
global plot_counter
global data_ready
BUFFER_LENGTH = 1024 #256
sleep_time_between_communications = 0.4 # previously: 0.1

# -- Functions and class definitions -----------------------------------------------------------------------------------
class socketListener(threading.Thread):
    def __init__(self, q, num=1):
        threading.Thread.__init__(self)
        self.num = num
        self.PORT = 46100 + self.num
        self.q = q
        # myhost = socket.gethostbyname_ex(socket.gethostname())
        # self.HOST = myhost[2][0]
        ip = os.popen("ifconfig wifi0 | grep 'inet addr' | awk '{print $2}' | awk -F : '{print $2}'").read()
        self.HOST = ip[0:-1]
        print('init-', self.HOST, '@', self.PORT)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind((self.HOST, self.PORT))
        self.s.listen(1)
        self.s.settimeout(20)
        self.active = 1
        self.ok = 0
        return

    def run(self):
        global plot_counter
        try:
            while self.active:
                print
                '[]Listening'
                try:
                    self.conn, self.addr = self.s.accept()
                    # print '[th-',self.num,'] Connected by', self.addr
                    myPrint('[th-', self.num, '] Connected by', self.addr)
                    self.ok = 1
                    self.conn.settimeout(20)  # 5
                except socket.timeout:
                    '[Exc-sL-acc', self.PORT, '] timeout'
                    pass
                except Exception, e:
                    print('[Exc-sL-acc', self.PORT, ']', e)
                    plot_counter = 0
                    return  # exit in case of unexpected exception
                while self.ok:
                    try:
                        data = self.conn.recv(BUFFER_LENGTH)  # 1024
                        if data == '' or data == '\n' or data == '\r':
                            pass
                        elif data == 'byebye' or data == 'byebye\n' or data == 'byebye\r' or data == 'byebye\n\r':
                            print('disconnected\n\r')
                            self.ok = 0
                            self.conn.close()
                        else:
                            # print data,'\n\r'
                            myPrint(data)
                            self.q.put(data)
                            self.conn.sendall('ack')
                    except Exception, e:
                        print('[Exc-sL-conn', self.PORT, ']', e)
                        self.ok = 0
                        self.conn.close()
                    time.sleep(sleep_time_between_communications)
                # print 'sleep'
                time.sleep(0.1)
            print('done\n\r')
        except Exception, e:
            print('[Exc-sL-', self.PORT, ']', e)
        print('exiting run\n\r')
        return

    def close(self):
        print('closing\n\r')
        self.s.close()
        self.active = 0
        return

class KeyboardPoller(threading.Thread):
    def run(self):
        global command
        global flag
        global data_ready
        while flag:
            if not data_ready.is_set():
                command = sys.stdin.readline()
                print('echoing->', command)
                data_ready.set()
            time.sleep(0.1)

    # Function that parses input command
def runCommand(command):
    res = 0
    # First check
    if command == '' or command == '\n' or command == '\r':
        pass
    # Close the system correctly
    elif command == 'exit\n':
        res = 1
    # print the help
    else:
        myPrint(command.rstrip())
        print('-----------------\n\r', \
        'available command\n\r', \
        'exit - closes sockets and exit\n\r', \
        '-----------------\n\r')
        pass
    # output
    return res

    # Function to convert system date-time into string
def dateToString():
    dt = datetime.datetime.today()
    month = str(dt.month) if dt.month > 9 else '0' + str(dt.month)
    day = str(dt.day) if dt.day > 9 else '0' + str(dt.day)
    hour = str(dt.hour) if dt.hour > 9 else '0' + str(dt.hour)
    min = str(dt.minute) if dt.minute > 9 else '0' + str(dt.minute)
    sec = str(dt.second) if dt.second > 9 else '0' + str(dt.second)
    res = {'year': str(dt.year), 'month': month, 'day': day, 'hour': hour,
            'min': min, 'sec': sec}
    return res

# Function to handle the double output:
# monthly log file and sys.stdout
def myPrint(*argv):
    global filename
    outf = open(filename, 'a')
    #outs = sys.stdout
    dt = dateToString()
    line = ('[' + dt['year'] + '/' + dt['month'] + '/' + dt['day'] + '-' + dt['hour'] + ':'
            + dt['min'] + ':' + dt['sec'] + ']')
    for item in argv:
        item = str(item).replace('\n', '')
        item = str(item).replace('\r', '')
        line += ' ' + str(item)
    line += '\n'
    try:
        #outs.write(line)
        outf.write(line)
    except Exception, e:
        print('[Exc-myPrint]', e)
    finally:
        #outs.flush()
        outf.close()
## ---------------------------------------------------------------------------------------------------------------------

# -- Set up parsing option  --------------------------------------------------------------------------------------------
parser = argparse.ArgumentParser(description='Ground Control Station')
parser.add_argument('--connect1',
                    help="vehicle 1 connection target string. e.g:--connect1 192.168.1.15:14550.")
parser.add_argument('--tcpip1',
                    help="tcp ip vehicle 1 connection target string. e.g:--tcpip1 192.168.1.15:5760")

args = parser.parse_args()

connection_string_1 = args.connect1

tcp_ip_1_string = args.tcpip1

data_ready = threading.Event()
## ---------------------------------------------------------------------------------------------------------------------

# -- Plot a figure with multiple subplots inside -----------------------------------------------------------------------
f, axarr = plt.subplots(4, 2, sharex=True)
f.suptitle('Gas Data')
plt.ion()

title = [['CO - [ADC]', 'NH3 - [I2C]'], ['NO2 - [I2C]', 'C3H8 - [I2C]'], ['C4H10 - [I2C]', 'CH4 - [I2C]'],
         ['H2 - [I2C]', 'C2H5O5 - [I2C]']]

for i in range(len(axarr)):
    for j in range(len(axarr[i])):
        axarr[i, j].grid()
        axarr[i, j].set_title(title[i][j])
        axarr[i, j].axis([0, 10, -0.5, 2])

plt.ion()
## ---------------------------------------------------------------------------------------------------------------------

## Parameters initialization -------------------------------------------------------------------------------------------

global filename
global command
global data_ready
global plot_counter
global flag
gas_type = ['CO - [ADC]', 'NH3 - [I2C]', 'NO2 - [I2C]', 'C3H8 - [I2C]', 'C4H10 - [I2C]', 'CH4 - [I2C]',
         'H2 - [I2C]', 'C2H5O5 - [I2C]']
gas_value = [] # 0 is CO, then, respectively: 1 - NH3, NO2, C3H8, C4H10, CH4, H2, C2H5OH
               # list of 8 tuples with (sum of total gas readings up to now, # of total gas readings, gas type)
               # each tuple is updated as soon as a reading is available
for i in range(8):
    gas_value.append([0.0, 0, 'Unavailable'])

global GPS_value
GPS_value = ['not available','not available','not available']
limy = 2
limy_NH3 = 2                    # useful for setting the maximum value on the plot axis
limy_NO2 = 2
limy_C3H8 = 2
limy_C4H10 = 2
limy_CH4 = 2
limy_H2 = 2
limy_C2H5OH = 2
plot_counter = 0
start = 0
sleep_time = 0.1                # sleep time between each iteration of the main loop
gas_pairing_refresh_rate = 0.5  # sleep time b/w each call of the GPS_gas_pairing thread
map_drawing_refresh_rate = 5    # sleep time b/w each call of the map_refresh thread
minimum_plotting_distance = 20  # minimum distance for a marker to be plotted on the map

# -- Multiple queues: dataq stores the data from first tcp/ip connection (analog sensor)
##                  dataq2 stores the data from second tcp/ip connection (i2c sensor)
dataq = Queue.Queue()
dataq2 = Queue.Queue()
data_ready.clear()
flag = True

# Regular expression patterns for parsing incoming data
# compile a pattern, in this case: match every occurrence of a float number x in a "..0 : x .. hours:min:sec" pattern
pattern_gas_generico_0 = re.compile('(?<=0 : )[-+]?\d+\.\d* \d+[:]\d+[:]\d+')
pattern_gas_NH3 = re.compile('(?<=NH3 : )[-+]?\d+\.\d* \d+[:]\d+[:]\d+')
pattern_gas_NO2 = re.compile('(?<=NO2 : )[-+]?\d+\.\d* \d+[:]\d+[:]\d+')
pattern_gas_C3H8 = re.compile('(?<=C3H8 : )[-+]?\d+\.\d* \d+[:]\d+[:]\d+')
pattern_gas_C4H10 = re.compile('(?<=C4H10 : )[-+]?\d+\.\d* \d+[:]\d+[:]\d+')
pattern_gas_CH4 = re.compile('(?<=CH4 : )[-+]?\d+\.\d* \d+[:]\d+[:]\d+')
pattern_gas_H2 = re.compile('(?<=H2 : )[-+]?\d+\.\d* \d+[:]\d+[:]\d+')
pattern_gas_C2H5O5 = re.compile('(?<=C2H5OH : )[-+]?\d+\.\d* \d+[:]\d+[:]\d+')
## ---------------------------------------------------------------------------------------------------------------------

# -- Output log --------------------------------------------------------------------------------------------------------

dt = dateToString()
filename = './logGCS_' + dt['year'] + dt['month'] + dt['day'] + '-' + dt['hour'] + dt['min'] + dt['sec'] + '.log'
## ---------------------------------------------------------------------------------------------------------------------

# -- Instantiate Threads -----------------------------------------------------------------------------------------------
# multiple socketListener: first one is for analog sensor, second one is for i2c sensor
sockList1 = socketListener(dataq, num=1)
sockList1.start()
#sockList2 = socketListener(dataq2, num=11)
#sockList2.start()
kl = KeyboardPoller()
kl.start()
## ---------------------------------------------------------------------------------------------------------------------

# -- Using Selenium to access browser visualization for html map -------------------------------------------------------

driver = webdriver.Firefox()
## ---------------------------------------------------------------------------------------------------------------------

# -- Connecting and creating csv files ---------------------------------------------------------------------------------
print("\nConnecting to vehicle 1 on: %s" % connection_string_1)
vehicle_1 = connect(connection_string_1, wait_ready=True, heartbeat_timeout=60)

with open('gas_data_log.csv', 'a') as the_file:
    the_file.write(
        'Time [Laptop]' + ';' + 'Time [BBBlue]' + ';' + 'Gas Type;' + 'Gas Data' + '\n')  # timestamp_laptop,timestamp_beaglebone,0,gas_sensor_data

with open('GPS_log.csv', 'a') as the_file:
    the_file.write(
        'Time [Laptop]' + ';' + 'Time [GPS]' + ';' + 'Latitude' + 'Longitude' + 'Altitude [relative]' + '\n')  # timestamp_laptop,timestamp_arducopter,gps_data (lat, lon, alt)

with open('Pairing_log.csv', 'a') as the_file:
    the_file.write(
        'Time [Laptop]' + ';' + 'Gas Type' + ';' + 'Gas Data' + ';' + 'Latitude' + ';' + 'Longitude' + ';' + 'Altitude [relative]' + '\n')  # timestamp_laptop,timestamp_arducopter,gps_data (lat, lon, alt)
## ---------------------------------------------------------------------------------------------------------------------

# -- Place map ---------------------------------------------------------------------------------------------------------
gmap = gmplot.GoogleMapPlotter(vehicle_1.location.global_relative_frame.lat,vehicle_1.location.global_relative_frame.lon, 17)
gmap.coloricon = "file://C:/Users/PC/Downloads/markers/%s.png"

previous_location = LocationGlobalRelative(-34.364114, 149.166022, 30)
## ---------------------------------------------------------------------------------------------------------------------

# -- Periodically pair GPS and gas measures ----------------------------------------------------------------------------
def GPS_gas_pairing():
    t = threading.Timer(gas_pairing_refresh_rate, GPS_gas_pairing)
    t.daemon = True
    t.start()
    for x in gas_value:
        if x[2] != 'Unavailable': # check if at least one reading is available
            if x[1] != 0:
                gas_value_average = float(x[0]/x[1]) # compute the average of all readings up to now
            else:
                gas_value_average = 0
            #print('Pairing GPS with measures: ' + '\n[Time]    [Gas Type]     [Gas Value]     [Latitude]          [Longitude]         [Altitude]')
            #print(strftime("%H:%M:%S", time.localtime()) + '   ' + str(x[2]) + '      ' + str(gas_value_average)
             #     + '      ' + str(GPS_value[0]) + '  ' + str(GPS_value[1]) + '  ' + str(GPS_value[2]) + '\n')

            with open('Pairing_log.csv', 'a') as the_file:
                the_file.write(strftime("%H:%M:%S", time.localtime()) + ';' + str(x[2]) + ';' + str(gas_value_average) + ';' + str(
                    GPS_value[0]) + ';' + str(
                    GPS_value[1]) + ';' + str(
                    GPS_value[2]) + '\n')  # timestamp_laptop,average_gas_data,gps_data (lat, lon, alt)

            if x[2] == gas_type[0]:
                """
                current_location = LocationGlobalRelative(float(GPS_value[0]), float(GPS_value[1]), 10)
                distance_from_previous_location = drone_lib.get_distance_metres(current_location, previous_location)
                if distance_from_previous_location > minimum_plotting_distance:
                    gmap.marker(GPS_value[0], GPS_value[1], title="Gas Avg: " + str(gas_value_average) + " - GPS:" + GPS_value[0] + ";" + GPS_value[1])

                    # inserting marker with scaling colours
                """
                """
                    if CO_gas_avg > 550 or C3H8_gas_avg > 750 or NH3_gas_avg > 550:
                        gmap.marker(float(splitted[1]), float(splitted[2]), color='#FC6355',
                                    title="CO : " + str(CO_gas_avg) + " [ppm] -- C3H8 : " + str(
                                        C3H8_gas_avg) + " [ppm] -- NH3 : " + str(NH3_gas_avg) + " [ppm]")
                    elif (CO_gas_avg > 450 and CO_gas_avg <= 550) or (C3H8_gas_avg > 550 and C3H8_gas_avg <= 750) or (
                            NH3_gas_avg > 450 and NH3_gas_avg <= 550):
                        gmap.marker(float(splitted[1]), float(splitted[2]), color='#EF9D3F',
                                    title="CO : " + str(CO_gas_avg) + " [ppm] -- C3H8 : " + str(
                                        C3H8_gas_avg) + " [ppm] -- NH3 : " + str(NH3_gas_avg) + " [ppm]")
                    else:
                        gmap.marker(float(splitted[1]), float(splitted[2]), color='#5680FC',
                                    title="CO : " + str(CO_gas_avg) + " [ppm] -- C3H8 : " + str(
                                        C3H8_gas_avg) + " [ppm] -- NH3 : " + str(NH3_gas_avg) + " [ppm]")
                """
                """
                    #previous_location = current_location
                """
## ---------------------------------------------------------------------------------------------------------------------

# -- Periodically refresh the map --------------------------------------------------------------------------------------
def map_refresh():
    t = threading.Timer(map_drawing_refresh_rate, map_refresh)
    t.daemon = True
    t.start()
    gmap.draw("gas_map.html")
    driver.refresh()
## ---------------------------------------------------------------------------------------------------------------------

# -- Pairing the gas value and the GPS coords periodically -------------------------------------------------------------
GPS_gas_pairing()
# ----------------------------------------------------------------------------------------------------------------------

# -- Refreshing the gas map periodically -------------------------------------------------------------------------------
map_refresh()
# ----------------------------------------------------------------------------------------------------------------------

# -- Endless loop: exit by typing 'exit' -------------------------------------------------------------------------------
while flag:

    # ------------ DRONE 1 ------------ #


    if data_ready.is_set():
        print('echo->', command)
        res = runCommand(command)
        if res > 0:
            flag = 0
        data_ready.clear()

    timestamp = strftime("%H:%M:%S", time.localtime())

    # -- CO [analog sensor] -----------------------------------------------------------------------------
    try:
        val = dataq.get(0)  # non-blocking get
        # print 'plot', plot_counter, '--', val, ';'
        try:
            # finding matches in data
            result_gas_generico_0 = re.findall(pattern_gas_generico_0, str(val))

            for p in result_gas_generico_0:
                split0 = p.split()
                print('gas sensor 0: ' + str(split0[0]) + ' at time: ' + str(split0[1]))
                with open('gas_data_log.csv', 'a') as the_file:
                    the_file.write(timestamp + ';' + split0[1] + ';' + 'CO [Analog];' + str("%.3f" % float(split0[0])) + '\n')  # timestamp_laptop,timestamp_beaglebone,0,gas_sensor_data
                gas_value[0][0] = gas_value[0][0] + float(split0[0])
                gas_value[0][1] = gas_value[0][1] + 1
                gas_value[0][2] = gas_type[0]
            ret = float(split0[0])

        except ValueError:
            print('ValueError')
            ret = 0
        finally:
            # plt.scatter(plot_counter,ret)
            if start == 0:
                start = 1
                plot_counter = 1
            axarr[0, 0].scatter(plot_counter, ret)
            if ret > limy: limy = ret + 0.3
            #if ret < limy / 2: limy = limy / 2
            axarr[0, 0].axis([0, 10, -0.5, limy])
            # axarr[0, 0].ylim([0, limy])
            # axarr[0, 0].xlim([0,plot_counter+1])
            # plt.axis([0, plt_counter+1, 300, 500])
        """
        WARNING: REPLACED ALL val2 WITH val FROM THIS POINT ON
        except Queue.Empty:
            pass
        except Exception, e:
            print('Exception', e)
            pass
        try:
            val = dataq.get(0)  # non-blocking get
        """
        #val = dataq2.get(0)
        # print 'plot', plot_counter, '--', val, ';'
        # -- NH3 [i2c sensor] ---------------------------------------------------------------------------
        try:
            # finding matches in data
            result_NH3 = re.findall(pattern_gas_NH3, str(val))
            for p in result_NH3:
                split_NH3 = p.split()
                #print('NH3 : ' + str(split_NH3[0]) + ' at time: ' + str(split_NH3[1]))
                with open('gas_data_log.csv', 'a') as the_file:
                    the_file.write(timestamp + ';' + split_NH3[1] + ';' + 'NH3 [I2C];' + str("%.3f" % float(split_NH3[0])) + '\n')  # timestamp_laptop,timestamp_beaglebone,0,gas_sensor_data
                gas_value[1][0] = gas_value[1][0] + float(split_NH3[0])
                gas_value[1][1] = gas_value[1][1] + 1
                gas_value[1][2] = gas_type[1]
            # ret = int(val)
            ret_NH3 = float(split_NH3[0])
        except ValueError:
            print('ValueError')
            ret_NH3 = 0
        finally:
            axarr[0, 1].scatter(plot_counter, ret_NH3)
            if ret_NH3 > limy_NH3: limy_NH3 = ret_NH3 + 1
            #if ret_NH3 < limy_NH3 / 2: limy_NH3 = limy_NH3 / 2
            axarr[0, 1].axis([0, 10, -0.5, limy_NH3])
            #axarr[0, 1].ylim([0, limy_NH3])
            #axarr[0, 1].xlim([0,plot_counter+1])
            # plt.axis([0, plt_counter+1, 300, 500])
        # -- NO2 [i2c sensor] ----------------------------------------------------------------------------
        try:
            #finding matches in data
            result_NO2 = re.findall(pattern_gas_NO2, str(val))
            for p in result_NO2:
                split_NO2 = p.split()
                #print('NO2 : ' + str(split_NO2[0]) + ' at time: ' + str(split_NO2[1]))
                with open('gas_data_log.csv', 'a') as the_file:
                    the_file.write(timestamp + ';' + split_NO2[1] + ';' + 'NO2 [I2C];' + str("%.3f" % float(split_NO2[0])) + '\n')  # timestamp_laptop,timestamp_beaglebone,0,gas_sensor_data
                gas_value[2][0] = gas_value[2][0] + float(split_NO2[0])
                gas_value[2][1] = gas_value[2][1] + 1
                gas_value[2][2] = gas_type[2]
            ret_NO2 = float(split_NO2[0])
        except ValueError:
            print('ValueError')
            ret_NO2 = 0
        finally:
            axarr[1, 0].scatter(plot_counter, ret_NO2)
            if ret_NO2 > limy_NO2: limy_NO2 = ret_NO2 + 1
            #if ret_NO2 < limy_NO2 / 2: limy_NO2 = limy_NO2 / 2
            axarr[1, 0].axis([0, 10, -0.5, limy_NO2])
            #axarr[1, 0].ylim([0, limy_NO2])
            #axarr[1, 0].xlim([0, plot_counter + 1])
        # -- C3H8 [i2c sensor] ----------------------------------------------------------------------------
        try:
            #finding matches in data
            result_C3H8 = re.findall(pattern_gas_C3H8, str(val))
            for p in result_C3H8:
                split_C3H8 = p.split()
                print('C3H8 : ' + str(split_C3H8[0]) + ' at time: ' + str(split_C3H8[1]))
                with open('gas_data_log.csv', 'a') as the_file:
                    the_file.write(timestamp + ';' + split_C3H8[1] + ';' + 'C3H8 [I2C];' + str("%.3f" % float(split_C3H8[0])) + '\n')  # timestamp_laptop,timestamp_beaglebone,0,gas_sensor_data
                gas_value[3][0] = gas_value[3][0] + float(split_C3H8[0])
                gas_value[3][1] = gas_value[3][1] + 1
                gas_value[3][2] = gas_type[3]
            ret_C3H8 = float(split_C3H8[0])
        except ValueError:
            print('ValueError')
            ret_C3H8 = 0
        finally:
            axarr[1, 1].scatter(plot_counter, ret_C3H8)
            if ret_C3H8 > limy_C3H8: limy_C3H8 = ret_C3H8 + 1
            #if ret_C3H8 < limy_C3H8 / 2: limy_C3H8 = limy_C3H8 / 2
            axarr[1, 1].axis([0, 10, -0.5, limy_C3H8])
            #axarr[1, 1].ylim([0,limy_C3H8])
            #axarr[1, 1].xlim([0, plot_counter + 1])
        # -- C4H10 [i2c sensor] ----------------------------------------------------------------------------
        try:
            #finding matches in data
            result_C4H10 = re.findall(pattern_gas_C4H10, str(val))
            for p in result_C4H10:
                split_C4H10 = p.split()
                print('C4H10 : ' + str(split_C4H10[0]) + ' at time: ' + str(split_C4H10[1]))
                with open('gas_data_log.csv', 'a') as the_file:
                    the_file.write(timestamp + ';' + split_C4H10[1] + ';' + 'C4H10 [I2C];' + str("%.3f" % float(split_C4H10[0])) + '\n')  # timestamp_laptop,timestamp_beaglebone,0,gas_sensor_data
                gas_value[4][0] = gas_value[4][0] + float(split_C4H10[0])
                gas_value[4][1] = gas_value[4][1] + 1
                gas_value[4][2] = gas_type[4]
            ret_C4H10 = float(split_C4H10[0])
        except ValueError:
            print('ValueError')
            ret_C4H10 = 0
        finally:
            axarr[2, 0].scatter(plot_counter, ret_C4H10)
            if ret_C4H10 > limy_C4H10: limy_C4H10 = ret_C4H10 + 1
            #if ret_C4H10 < limy_C4H10 / 2: limy_C4H10 = limy_C4H10 / 2
            axarr[2, 0].axis([0, 10, -0.5, limy_C4H10])
            #axarr[2, 0].ylim([0,limy_C4H10])
            #axarr[2, 0].xlim([0, plot_counter + 1])
        # -- CH4 [i2c sensor] -----------------------------------------------------------------------------
        try:
            #finding matches in data
            result_CH4 = re.findall(pattern_gas_CH4, str(val))
            for p in result_CH4:
                split_CH4 = p.split()
                print('CH4 : ' + str(split_CH4[0]) + ' at time: ' + str(split_CH4[1]))
                with open('gas_data_log.csv', 'a') as the_file:
                    the_file.write(timestamp + ';' + split_CH4[1] + ';' + 'CH4 [I2C];' + str("%.3f" % float(split_CH4[0])) + '\n')  # timestamp_laptop,timestamp_beaglebone,0,gas_sensor_data
                gas_value[5][0] = gas_value[5][0] + float(split_CH4[0])
                gas_value[5][1] = gas_value[5][1] + 1
                gas_value[5][2] = gas_type[5]
            ret_CH4 = float(split_CH4[0])
        except ValueError:
            print('ValueError')
            ret_CH4 = 0
        finally:
            axarr[2, 1].scatter(plot_counter, ret_CH4)
            if ret_CH4 > limy_CH4: limy_CH4 = ret_CH4 + 1
            #if ret_CH4 < limy_CH4 / 2: limy_CH4 = limy_CH4 / 2
            axarr[2, 1].axis([0, 10, -0.5, limy_CH4])
            #axarr[2, 1].ylim([0,limy_CH4])
            #axarr[2, 1].xlim([0, plot_counter + 1])
        # -- H2 [i2c sensor] -----------------------------------------------------------------------------
        try:
            #finding matches in data
            result_H2 = re.findall(pattern_gas_H2, str(val))
            for p in result_H2:
                split_H2 = p.split()
                #print('H2 : ' + str(split_H2[0]) + ' at time: ' + str(split_H2[1]))
                with open('gas_data_log.csv', 'a') as the_file:
                    the_file.write(timestamp + ';' + split_H2[1] + ';' + 'H2 [I2C];' + str("%.3f" % float(split_H2[0])) + '\n')  # timestamp_laptop,timestamp_beaglebone,0,gas_sensor_data
                gas_value[6][0] = gas_value[6][0] + float(split_H2[0])
                gas_value[6][1] = gas_value[6][1] + 1
                gas_value[6][2] = gas_type[6]
            ret_H2 = float(split_H2[0])
        except ValueError:
            print('ValueError')
            ret_H2 = 0
        finally:
            axarr[3, 0].scatter(plot_counter, ret_H2)
            if ret_H2 > limy_H2: limy_H2 = ret_H2 + 1
            #if ret_H2 < limy_H2 / 2: limy_H2 = limy_H2 / 2
            axarr[3, 0].axis([0, 10, -0.5, limy_H2])
            #axarr[3, 0].ylim([0,limy_H2])
            #axarr[3, 0].xlim([0, plot_counter + 1])
        # -- C2H5OH [i2c sensor] --------------------------------------------------------------------------
        try:
            #finding matches in data
            result_C2H5O5 = re.findall(pattern_gas_C2H5O5, str(val))
            for p in result_C2H5O5:
                split_C2H5O5 = p.split()
                #print('C2H5O5 : ' + str(split_C2H5O5[0]) + ' at time: ' + str(split_C2H5O5[1]))
                with open('gas_data_log.csv', 'a') as the_file:
                    the_file.write(timestamp + ';' + split_C2H5O5[1] + ';' + 'C2H5O5 [I2C];' + str("%.3f" % float(split_C2H5O5[0])) + '\n')  # timestamp_laptop,timestamp_beaglebone,0,gas_sensor_data
                gas_value[7][0] = gas_value[7][0] + float(split_C2H5O5[0])
                gas_value[7][1] = gas_value[7][1] + 1
                gas_value[7][2] = gas_type[7]
            ret_C2H5OH = float(split_C2H5O5[0])
        except ValueError:
            print('ValueError')
            ret_C2H5OH = 0
        finally:
            axarr[3, 1].scatter(plot_counter, ret_C2H5OH)
            if ret_C2H5OH > limy_C2H5OH: limy_C2H5OH = ret_C2H5OH + 1
            #if ret_C2H5OH < limy_C2H5OH / 2: limy_C2H5OH = limy_C2H5OH / 2
            axarr[3, 1].axis([0, 10, -0.5, limy_C2H5OH])
            #axarr[3, 1].ylim([0,limy_C2H5O5])
            #axarr[3, 1].xlim([0, plot_counter + 1])
    except Queue.Empty:
        pass
    except Exception, e:
        print('Exception', e)
        pass
    finally:
        if start == 1:
            plot_counter += 1
        plt.xlim([0, plot_counter + 1])
        plt.pause(0.05)
    time.sleep(0.1)

    #print("Autopilot capabilities")
    #print('Time : %s\n' % (strftime("%H:%M:%S", time.localtime())))


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
    print(" Global Location: %s" % vehicle_1.location.global_frame)
    print(" Global Location (relative altitude): %s" % vehicle_1.location.global_relative_frame)
    print(" Local Location: %s" % vehicle_1.location.local_frame)
    print("EKF OK?: %s" % vehicle_1.ekf_ok)
    print("Mode: %s" % vehicle_1.mode.name)  # settable
    #print(" Armed: %s" % vehicle.armed)    # settable
    print("Attitude: %s" % vehicle_1.attitude)
    print("Velocity: %s" % vehicle_1.velocity)
    print("Heading: %s" % vehicle_1.heading)
    """

    # -- logging data into .txt file -----------------------------------------------------------------------------------

    with open('data_log.txt', 'a') as the_file:
        the_file.write('-- Drone 1 --\n\n')
        the_file.write('Time : %s\n' % (strftime("%H:%M:%S", time.localtime())))
        the_file.write('GPS : ' + str(vehicle_1.gps_0) + '\n\n')
        the_file.write('Global Location : ' + str(vehicle_1.location.global_frame) + '\n\n')
        the_file.write(
            'Global Location  (relative altitude): ' + str(vehicle_1.location.global_relative_frame) + '\n\n')
        the_file.write('Local Location : ' + str(vehicle_1.location.local_frame) + '\n\n')
        the_file.write('EKF OK? : ' + str(vehicle_1.ekf_ok) + '\n\n')
        the_file.write('Mode : ' + str(vehicle_1.mode.name) + '\n\n')
        the_file.write('Attitude : ' + str(vehicle_1.attitude) + '\n\n')
        the_file.write('Velocity : ' + str(vehicle_1.velocity) + '\n\n')
        the_file.write('Heading : ' + str(vehicle_1.heading) + '\n\n')
    ## -----------------------------------------------------------------------------------------------------------------

    # ------------ DRONE 2 ---------------------------------------------------------------------------------------------

    ## -----------------------------------------------------------------------------------------------------------------
    #print("\n --- END OF THIS ITERATION - sleep of " + str(sleep_time) +" seconds ---\n")
    time.sleep(sleep_time)  # sleep between consecutive iterations
## ---------------------------------------------------------------------------------------------------------------------

# -- Close vehicle objects and connections before exiting script -------------------------------------------------------
print("\nClosing all connections and vehicle objects")
vehicle_1.close()
# close the connection
#conn_1.close()
"""
if connection_string_2:
    vehicle_2.close()
    # close the connection
    conn_2.close()
"""
print("Completed")

sockList1.close()
#sockList2.close()

sockList1.join(True)
#sockList2.join(True)

# Exit
print('bye-bye')
sys.exit(0)
## ---------------------------------------------------------------------------------------------------------------------





