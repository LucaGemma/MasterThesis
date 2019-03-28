# ----------------------------------------------- Imports ------------------------------------------------------------ #
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time, sys, os, logging, datetime, select
import threading, multiprocessing, argparse
import subprocess, dronekit, pymavlink, random
import aDroneIF, swarmManager, heatmap
#from logger import swarm_logger, swarm_logger_file_handler, filename_gas_data_log, filename_GPS_log, filename_Pairing_log
#from swarmGlobals import uav_n, ctrl_l, fname, log_l, ver, base_ip, init_ip, base_ctrl_port, base_sens_port, valid_pattern, detect_task, altitude_levels, thresholds_dictionary
from swarmGlobals import *
from logger import *
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from custom_library import drone_lib
import re
import socket, Queue
from time import gmtime, strftime
from subprocess import Popen, PIPE
import csv


# -- Functions and class definitions -----------------------------------------------------------------------------------


class socketListener(threading.Thread):
    def __init__(self, q, num=46101):
        threading.Thread.__init__(self)
        self.num = num
        self.PORT = self.num
        self.q = q
        # myhost = socket.gethostbyname_ex(socket.gethostname())
        # self.HOST = myhost[2][0]
        ip = os.popen("ifconfig wifi0 | grep 'inet addr' | awk '{print $2}' | awk -F : '{print $2}'").read()
        self.HOST = ip[0:-1]
        print('init-', self.HOST, '@', self.PORT)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #self.s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #self.s2.connect(('192.168.0.2', 46143))
        #self.s.bind(('127.0.0.1', self.PORT))
        self.s.bind((self.HOST, self.PORT))
        self.s.listen(1)
        self.s.settimeout(20)
        self.active = 1
        self.ok = 0
        self.first_connection = True
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
                        timestamp_ = strftime("d %H:%M:%S d", time.localtime())
                        if self.first_connection is True:
                            self.conn.send(str(timestamp_))
                            self.first_connection = False
                            time.sleep(0.5)
                        data = self.conn.recv(BUFFER_LENGTH)  # 1024
                        #self.s2.send(timestamp)
                        if data == '' or data == '\n' or data == '\r':
                            pass
                        elif data == 'byebye' or data == 'byebye\n' or data == 'byebye\r' or data == 'byebye\n\r':
                            print('disconnected\n\r')
                            self.ok = 0
                            self.conn.close()
                        else:
                            # print data,'\n\r'
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


def parse_gas_data(_dataq, _data_ready, _drone_d1_list, _id, ns, vehicle_1, filename_gas_data_log, filename_GPS_log):

    """

    :param _dataq: queue where received data is stored
    :param _data_ready: unused
    :param _drone_d1_list: list of drone dictionaries
    :param _id: current drone id
    :param ns: True if the main loop is running, False elsewhere
    :param vehicle_1: represents the current vehicle
    :return:
    """
    #print("\t---------- STARTING PARSE GAS DATA -------".expandtabs((_id + 2) * 40))
    GPS_value[0] = str(vehicle_1.location.global_relative_frame.lat)
    GPS_value[1] = str(vehicle_1.location.global_relative_frame.lon)
    GPS_value[2] = str(vehicle_1.location.global_relative_frame.alt)
    dataq = _dataq
    data_ready = _data_ready
    print_counter = 0


    while ns.running is False:
        time.sleep(0.5)

    while ns.running:

        timestamp = strftime("%H:%M:%S", time.localtime())

        # -- CO [analog sensor] -----------------------------------------------------------------------------
        try:
            val = dataq.get(0)  # non-blocking get
            # print 'plot', plot_counter, '--', val, ';'
            try:
                # finding matches in data
                result_gas_generico_0 = re.findall(pattern_gas_generico_0, str(val))
                split0 = [-1.0, 'no time']
                for p in result_gas_generico_0:
                    split0 = p.split()
                    string_output = '\tgas sensor 0: ' + str(split0[0]) + ' at time: ' + str(split0[1])
                    #print(string_output.expandtabs((_id + 2) * 40))
                    with open(filename_gas_data_log, 'a') as the_file:
                        the_file.write(timestamp + ';' + split0[1] + ';' + 'CO [Analog];' + str(
                            "%.3f" % float(split0[0])) + '\n')  # timestamp_laptop,timestamp_beaglebone,0,gas_sensor_data
                    gas_value[0][0] = gas_value[0][0] + float(split0[0])
                    gas_value[0][1] = gas_value[0][1] + 1
                    gas_value[0][2] = gas_type[0]
                ret = float(split0[0])
            except ValueError:
                print('\tValueError'.expandtabs((_id + 2) * 40))

            # -- NH3 [i2c sensor] ---------------------------------------------------------------------------
            try:
                # finding matches in data
                split_NH3 = ['no value', 'no time']
                result_NH3 = re.findall(pattern_gas_NH3, str(val))
                split_NH3 = [-1.0, 'no time']
                for p in result_NH3:
                    split_NH3 = p.split()
                    string_output = '\tNH3 : ' + str(split_NH3[0]) + ' at time: ' + str(split_NH3[1])
                    #print(string_output.expandtabs((_id + 2) * 40))
                    with open(filename_gas_data_log, 'a') as the_file:
                        the_file.write(timestamp + ';' + split_NH3[1] + ';' + 'NH3 [I2C];' + str(
                            "%.3f" % float(split_NH3[0])) + '\n')  # timestamp_laptop,timestamp_beaglebone,0,gas_sensor_data
                    gas_value[1][0] = gas_value[1][0] + float(split_NH3[0])
                    gas_value[1][1] = gas_value[1][1] + 1
                    gas_value[1][2] = gas_type[1]
                # ret = int(val)
                ret_NH3 = float(split_NH3[0])
            except ValueError:
                print('\tValueError'.expandtabs((_id + 2) * 40))
            # -- NO2 [i2c sensor] ----------------------------------------------------------------------------
            try:
                # finding matches in data
                split_NO2 = ['no value', 'no time']
                result_NO2 = re.findall(pattern_gas_NO2, str(val))
                split_NO2 = [-1.0, 'no time']
                for p in result_NO2:
                    split_NO2 = p.split()
                    string_output = '\tNO2 : ' + str(split_NO2[0]) + ' at time: ' + str(split_NO2[1])
                    #print(string_output.expandtabs((_id + 2) * 40))
                    with open(filename_gas_data_log, 'a') as the_file:
                        the_file.write(timestamp + ';' + split_NO2[1] + ';' + 'NO2 [I2C];' + str(
                            "%.3f" % float(split_NO2[0])) + '\n')  # timestamp_laptop,timestamp_beaglebone,0,gas_sensor_data
                    gas_value[2][0] = gas_value[2][0] + float(split_NO2[0])
                    gas_value[2][1] = gas_value[2][1] + 1
                    gas_value[2][2] = gas_type[2]
                ret_NO2 = float(split_NO2[0])
            except ValueError:
                print('\tValueError'.expandtabs((_id + 2) * 40))
            # -- C3H8 [i2c sensor] ----------------------------------------------------------------------------
            try:
                # finding matches in data
                split_C3H8 = ['no value', 'no time']
                result_C3H8 = re.findall(pattern_gas_C3H8, str(val))
                split_C3H8 = [-1.0, 'no time']
                for p in result_C3H8:
                    split_C3H8 = p.split()
                    string_output = '\tC3H8 : ' + str(split_C3H8[0]) + ' at time: ' + str(split_C3H8[1])
                    #print(string_output.expandtabs((_id + 2) * 40))
                    with open(filename_gas_data_log, 'a') as the_file:
                        the_file.write(timestamp + ';' + split_C3H8[1] + ';' + 'C3H8 [I2C];' + str("%.3f" % float(
                            split_C3H8[0])) + '\n')  # timestamp_laptop,timestamp_beaglebone,0,gas_sensor_data
                    gas_value[3][0] = gas_value[3][0] + float(split_C3H8[0])
                    gas_value[3][1] = gas_value[3][1] + 1
                    gas_value[3][2] = gas_type[3]
                ret_C3H8 = float(split_C3H8[0])
            except ValueError:
                print('\tValueError'.expandtabs((_id + 2) * 40))
            # -- C4H10 [i2c sensor] ----------------------------------------------------------------------------
            try:
                # finding matches in data
                split_C4H10 = ['no value', 'no time']
                result_C4H10 = re.findall(pattern_gas_C4H10, str(val))
                split_C4H10 = [-1.0, 'no time']
                for p in result_C4H10:
                    split_C4H10 = p.split()
                    string_output = '\tC4H10 : ' + str(split_C4H10[0]) + ' at time: ' + str(split_C4H10[1])
                    #print(string_output.expandtabs((_id + 2) * 40))
                    with open(filename_gas_data_log, 'a') as the_file:
                        the_file.write(timestamp + ';' + split_C4H10[1] + ';' + 'C4H10 [I2C];' + str("%.3f" % float(
                            split_C4H10[0])) + '\n')  # timestamp_laptop,timestamp_beaglebone,0,gas_sensor_data
                    gas_value[4][0] = gas_value[4][0] + float(split_C4H10[0])
                    gas_value[4][1] = gas_value[4][1] + 1
                    gas_value[4][2] = gas_type[4]
                ret_C4H10 = float(split_C4H10[0])
            except ValueError:
                print('\tValueError'.expandtabs((_id + 2) * 40))

            # -- CH4 [i2c sensor] -----------------------------------------------------------------------------
            try:
                # finding matches in data
                split_CH4 = ['no value', 'no time']
                result_CH4 = re.findall(pattern_gas_CH4, str(val))
                split_CH4 = [-1.0, 'no time']
                for p in result_CH4:
                    split_CH4 = p.split()
                    string_output = '\tCH4 : ' + str(split_CH4[0]) + ' at time: ' + str(split_CH4[1])
                    #print(string_output.expandtabs((_id + 2) * 40))
                    with open(filename_gas_data_log, 'a') as the_file:
                        the_file.write(timestamp + ';' + split_CH4[1] + ';' + 'CH4 [I2C];' + str(
                            "%.3f" % float(split_CH4[0])) + '\n')  # timestamp_laptop,timestamp_beaglebone,0,gas_sensor_data
                    gas_value[5][0] = gas_value[5][0] + float(split_CH4[0])
                    gas_value[5][1] = gas_value[5][1] + 1
                    gas_value[5][2] = gas_type[5]
                ret_CH4 = float(split_CH4[0])
                temp = _drone_d1_list[_id]
                temp['CH4'] = ret_CH4
                _drone_d1_list[_id] = temp
            except ValueError:
                print('\tValueError'.expandtabs((_id + 2) * 40))
            # -- H2 [i2c sensor] -----------------------------------------------------------------------------
            try:
                # finding matches in data
                split_H2 = ['no value', 'no time']
                result_H2 = re.findall(pattern_gas_H2, str(val))
                split_H2 = [-1.0, 'no time']
                for p in result_H2:
                    split_H2 = p.split()
                    string_output = '\tH2 : ' + str(split_H2[0]) + ' at time: ' + str(split_H2[1])
                    #print(string_output.expandtabs((_id + 2) * 40))
                    with open(filename_gas_data_log, 'a') as the_file:
                        the_file.write(timestamp + ';' + split_H2[1] + ';' + 'H2 [I2C];' + str(
                            "%.3f" % float(split_H2[0])) + '\n')  # timestamp_laptop,timestamp_beaglebone,0,gas_sensor_data
                    gas_value[6][0] = gas_value[6][0] + float(split_H2[0])
                    gas_value[6][1] = gas_value[6][1] + 1
                    gas_value[6][2] = gas_type[6]
                ret_H2 = float(split_H2[0])
            except ValueError:
                print('\tValueError'.expandtabs((_id + 2) * 40))
            # -- C2H5OH [i2c sensor] --------------------------------------------------------------------------
            try:
                # finding matches in data
                split_C2H5O5 = ['no value', 'no time']
                result_C2H5O5 = re.findall(pattern_gas_C2H5O5, str(val))
                split_C2H5O5 = [-1.0, 'no time']
                for p in result_C2H5O5:
                    split_C2H5O5 = p.split()
                    string_output = '\tC2H5O5 : ' + str(split_C2H5O5[0]) + ' at time: ' + str(split_C2H5O5[1])
                    #print(string_output.expandtabs((_id + 2) * 40))
                    with open(filename_gas_data_log, 'a') as the_file:
                        the_file.write(timestamp + ';' + split_C2H5O5[1] + ';' + 'C2H5O5 [I2C];' + str("%.3f" % float(
                            split_C2H5O5[0])) + '\n')  # timestamp_laptop,timestamp_beaglebone,0,gas_sensor_data
                    gas_value[7][0] = gas_value[7][0] + float(split_C2H5O5[0])
                    gas_value[7][1] = gas_value[7][1] + 1
                    gas_value[7][2] = gas_type[7]
            except ValueError:
                print('\tValueError'.expandtabs((_id + 2) * 40))

            # -- CO [i2c sensor] ---------------------------------------------------------------------------
            try:
                # finding matches in data
                split_CO_i2c = ['no value', 'no time']
                result_CO_i2c = re.findall(pattern_gas_CO_i2c, str(val))
                split_CO_i2c = [-1.0, 'no time']
                for p in result_CO_i2c:
                    split_CO_i2c = p.split()
                    string_output = '\tCO [I2C] : ' + str(split_CO_i2c[0]) + ' at time: ' + str(split_CO_i2c[1])
                    #print(string_output.expandtabs((_id + 2) * 40))
                    with open(filename_gas_data_log, 'a') as the_file:
                        the_file.write(timestamp + ';' + split_CO_i2c[1] + ';' + 'CO [I2C];' + str(
                            "%.3f" % float(
                                split_CO_i2c[0])) + '\n')  # timestamp_laptop,timestamp_beaglebone,0,gas_sensor_data
                    gas_value[8][0] = gas_value[8][0] + float(split_CO_i2c[0])
                    gas_value[8][1] = gas_value[8][1] + 1
                    gas_value[8][2] = gas_type[8]
                # ret = int(val)
                ret_CO_i2c = float(split_CO_i2c[0])
                temp = _drone_d1_list[_id]
                temp['CO'] = ret_CO_i2c
                _drone_d1_list[_id] = temp
            except ValueError:
                print('\tValueError'.expandtabs((_id + 2) * 40))

            # -- TEMP -----------------------------------------------------------------------------
            try:
                # finding matches in data
                TEMP = ['no value', 'no time']
                result_TEMP = re.findall(pattern_TEMP, str(val))
                TEMP = [-1.0, 'no time']
                for p in result_TEMP:
                    TEMP= p.split()
                    string_output = '\tTEMP : ' + str(TEMP[0]) + ' at time: ' + str(TEMP[1])
                    #print(string_output.expandtabs((_id + 2) * 40))
                    with open(filename_gas_data_log, 'a') as the_file:
                        the_file.write(timestamp + ';' + TEMP[1] + ';' + 'TEMP;' + str(
                            "%.3f" % float(TEMP[0])) + '\n')  # timestamp_laptop,timestamp_beaglebone,0,gas_sensor_data
                    gas_value[9][0] = gas_value[9][0] + float(TEMP[0])
                    gas_value[9][1] = gas_value[9][1] + 1
                    gas_value[9][2] = gas_type[9]
                ret_TEMP = float(TEMP[0])
                #print(_drone_d1_list[_id])
                temp = _drone_d1_list[_id]
                temp['temperature'] = ret_TEMP
                _drone_d1_list[_id] = temp
            except ValueError:
                print('\tValueError'.expandtabs((_id + 2) * 40))

            # -- CO2 [UART sensor] -----------------------------------------------------------------------------
            try:
                # finding matches in data
                split_CO2_uart = ['no value', 'no time']
                result_CO2_uart = re.findall(pattern_gas_CO2_uart, str(val))
                split_CO2_uart = [-1.0, 'no time']
                for p in result_CO2_uart:
                    split_CO2_uart = p.split()
                    string_output = '\tCO2 - UART : ' + str(split_CH4[0]) + ' at time: ' + str(split_CH4[1])
                    #print(string_output.expandtabs((_id + 2) * 40))
                    with open(filename_gas_data_log, 'a') as the_file:
                        the_file.write(timestamp + ';' + split_CO2_uart[1] + ';' + 'CO2 [UART];' + str(
                            "%.3f" % float(split_CO2_uart[0])) + '\n')  # timestamp_laptop,timestamp_beaglebone,0,gas_sensor_data
                    gas_value[10][0] = gas_value[10][0] + float(split_CO2_uart[0])
                    gas_value[10][1] = gas_value[10][1] + 1
                    gas_value[10][2] = gas_type[10]
                ret_CO2_uart = float(split_CO2_uart[0])
                temp = _drone_d1_list[_id]
                temp['CO2'] = ret_CO2_uart
                _drone_d1_list[_id] = temp
            except ValueError:
                print('\tValueError'.expandtabs((_id + 2) * 40))
            # -- H2 [i2c sensor] -----------------------------------------------------------------------------

        except Queue.Empty:
            pass
        except Exception, e:
            print('Exception', e)
            pass
        finally:
            time.sleep(0.01)

        # print("Autopilot capabilities")
        # print('Time : %s\n' % (strftime("%H:%M:%S", time.localtime())))

        # create a listener to catch each SCALED PRESSURE message and store the last value of pressure and temperature
        #@vehicle_1.on_message('SCALED_PRESSURE')
        #def listener(self, name, message):
            """
            print('absolute pressure     [hPa] : %s' % message.press_abs)
            print('differential pressure [hPa] : %s' % message.press_diff)
            print('temperature  [0.01 Celsius] : %s \n' % message.temperature)
            """
            #absolute_pressure = message.press_abs
            #temperature = message.temperature

        # logging Pressure and Temperature
        with open(filename_gas_data_log, 'a') as the_file:
            the_file.write(
                timestamp + ';' + str(0) + ';' + ';' + ';' + str(absolute_pressure) + ';' + str(temperature) + '\n')

        @vehicle_1.on_message('SYSTEM_TIME')
        def listener(self, name, message):
            GPS_value[0] = str(self.location.global_relative_frame.lat)
            GPS_value[1] = str(self.location.global_relative_frame.lon)
            GPS_value[2] = str(self.location.global_relative_frame.alt)
            with open(filename_GPS_log, 'a') as the_file:
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
        print(" Armed: %s" % vehicle.armed)    # settable
        print("Attitude: %s" % vehicle_1.attitude)
        print("Velocity: %s" % vehicle_1.velocity)
        print("Heading: %s" % vehicle_1.heading)
        """
        """
        print("\nAirspeed: %s" % vehicle_1.airspeed)
        print("Groundspeed: %s" % vehicle_1.groundspeed)
        print("Wind speed: %s" % (vehicle_1.groundspeed - vehicle_1.airspeed))
        print("Pressure: %s" % absolute_pressure)
        print("Temperature: %s \n" % temperature)
        """
        # print("Estimated Temperature [Celsius]: %s" % vehicle_1.parameters['TEMP'])
        # take the minimum b/w 35 Celsius and this param, as the barometer tends to be off by quite
        # a large margin, often even above 30 degrees. note: overestimate a temp leads to increase the estimated altitude
        # for the barometer

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
        #_drone_d1_list[_id] += 1
        if print_counter >= (2/sleep_time):
            #print("\tgas_parse_data d1: ".expandtabs((_id + 2) * 40) + str(_drone_d1_list[0]))
            print_counter = 0
        print_counter += 1
        time.sleep(sleep_time)  # sleep between consecutive iterations

    print("\t--- PARSING ENDED ---".expandtabs((_id + 2) * 40))
    time.sleep(2)


# ----------------------------------------- Additional Functions ----------------------------------------------------- #


def runCommand(command):

    """
    elaborate input command to detect a valid command or an exit command

    :param command: input command to be elaborated
    :return: the function returns a dictionary with the type of command and the raw typed command, made as follows:

    {'res':res, 'command':command}
    """

    print "- entering the runCommand function -"
    # check if a valid drone command has been typed (a valid command is made of: droneX,something,else)
    splitz = ['']
    res = 0
    current_task = re.findall(valid_pattern, str(command))
    for p in current_task:
        splitz = p.split()
        #print "typed command to drone : {}" .format(splitz)
    if len(splitz) > 0 and splitz != ['']:
        res = 2
        #print "detected valid command"
    # check if no string has been typed
    elif command == '' or command == '\n' or command == '\r':
        #print "nothing typed, pass"
        pass
    # check if an exit command has been typed
    elif command == 'exit' or command == "exit":
        res = 1
        #print "exit typed"
    # else, print the help
    elif command == 'land' or command == 'rtl':
        res = 3
        #print "landing all the drones"
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

    with open(filename_Pairing_log, 'w') as the_file:
        the_file.write(
            'Time [Laptop]' + ';' + 'Gas Type' + ';' + 'Gas Data [average]' + ';' + 'Latitude' + ';' + 'Longitude' + ';' + 'Altitude [relative];' + 'Pressure [hectoPascal];' + 'Temperature [0.01 Celsius degree]'  + '\n')






    mgr = multiprocessing.Manager()
    drone_id_list = mgr.list()          # shared list of drone ids
    drone_evnt_list = mgr.list()        # shared list of drone events
    drone_d1_list = mgr.list()          # shared list for data exchange
    drone_d2_list = mgr.list()          # shared list for data exchange
    action_list = mgr.list()            # shared list of drone action dictionaries
    stop_action = mgr.list()            # shared list of flag, one for each drone
    #                                     True if the current action has to be aborted immediately
    d_l = []                            # list of drone objects managed by main process
    s_l = []
    g_l = []
    dataq = []

    g_var = mgr.Namespace()
    g_var.running = False               # True if the main loop is running, False elsewhere
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
    parser.add_argument('--sport',
                        help="vehicle 1 streaming port. e.g:--sport 46101.")

    param_list = parser.parse_args()

    sport = param_list.sport

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

    swarm_logger.setLevel(log_l)

    swarm_logger_file_handler.setLevel(log_l)

    uav_n = param_list.number_d
    swarm_logger.info('Set size of swarm to: %s', uav_n)
    ctrl_l = param_list.ctrl_law
    swarm_logger.info('Set control law to: %s', ctrl_l)

    # .csv creation [GAS SENSORS and GPS]
    # ------------------------------------------------------------------------------------
    for i in range(uav_n):
        # filename = '../logs/logGCS_' + dt['year'] + dt['month'] + dt['day'] + '-' + dt['hour'] + dt['min'] + dt['sec'] + '.log'
        filename_gas_data_log.append(
            '../logs/gas_data_log_' + 'UAV_' + str(i) + '_' + dt['year'] + dt['month'] + dt['day'] + '-' + dt['hour'] +
            dt['min'] + dt['sec'] + '.csv')
        filename_GPS_log.append(
            '../logs/GPS_log_' + 'UAV_' + str(i) + '_' + dt['year'] + dt['month'] + dt['day'] + '-' + dt['hour'] + dt[
                'min'] + dt['sec'] + '.csv')

        with open(filename_gas_data_log[i], 'a') as the_file:
            the_file.write(
                'Time [Laptop]' + ';' + 'Time [BBBlue]' + ';' + 'Gas Type;' + 'Gas Data' + ';' + 'Pressure [hectoPascal];' + 'Temperature [0.01 Celsius degree]' + '\n')  # timestamp_laptop,timestamp_beaglebone,0,gas_sensor_data

        with open(filename_GPS_log[i], 'a') as the_file:
            the_file.write(
                'Time [Laptop]' + ';' + 'Time [GPS]' + ';' + 'Latitude' + 'Longitude' + 'Altitude [relative]' + '\n')
            #        timestamp_laptop,timestamp_arducopter,gps_data (lat, lon, alt)


    # initialize the remaining variables and all the aDroneIF processes and start them
    logging.info('Starting %s %s with params %d %d %d' % (fname, ver, log_l, uav_n, ctrl_l))
    time.sleep(1)

    for i in range(11):
        gas_value.append([-1.0, 0, 'Unavailable'])
        y_data.append([])
        x_data.append([])
        plot_counter.append(0)

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
                drone_d1_list.append({'CO': -1.0, 'CH4': -1.0, 'temperature': -1.0})
                drone_d2_list.append({'altitude': 0.0, 'latitude': 0.0, 'longitude': 0.0, 'battery': -1})
                stop_action.append(False)
                action_list.append({'action': 'action complete', 'altitude': '', 'latitude': '', 'longitude': '', 'v_x':'', 'v_y':'', 'v_z':'', 'condition_yaw':'', 'wait': 0})

                # initialize and start aDroneIF processes
                d = aDroneIF.aDroneIF(log_l, base_ip, init_ip, base_ctrl_port, base_sens_port, ping, g_var, drone_evnt_list, drone_d1_list, drone_d2_list, action_list, stop_action)
                d_l.append(d)
                drone_id_list.append(ping)
                d.start()
                time.sleep(1)


        for ping in range(uav_n):
            # -- Instantiate Threads -----------------------------------------------------------------------------------------------
            # multiple socketListener: first one is for analog sensor, second one is for i2c sensor
            dataq.append(Queue.Queue())
            sockList1 = socketListener(dataq[ping], num=(int(sport) + 10 * ping))
            sockList1.daemon = True
            s_l.append(sockList1)
            sockList1.start()

            # -- Connecting  ---------------------------------------------------------------------------------
            connection_string_1 = str(int((base_ctrl_port)+10*ping)+2)
            logging.info("\nConnecting to vehicle 1 on: %s" % connection_string_1)
            vehicle = connect("udp:" + base_ip + ":" + connection_string_1, wait_ready=True)

            gas_parser = threading.Thread(target=parse_gas_data, args=(dataq[ping], data_ready, drone_d1_list, ping, g_var, vehicle, filename_gas_data_log[ping], filename_GPS_log[ping]))
            gas_parser.daemon = True
            #GPS_gas_pairing(drone_d1_list, drone_d2_list, ping)

            g_l.append(gas_parser)
            gas_parser.start()


        # initialize and start the swarmManager process
        sm_p = swarmManager.swarmManager(log_l, uav_n, g_var, drone_id_list, drone_evnt_list, drone_d1_list, drone_d2_list, action_list, stop_action, TEMP_LIMIT,BATT_LIMIT)
        sm_p.start()
        # initialize plot map process
        hm_p = heatmap.plotmap(altitude_levels, thresholds_dictionary, log_l, uav_n, g_var, drone_id_list, drone_evnt_list, drone_d1_list, drone_d2_list)
        hm_p.start()


        #sockList1.start()

        #gas_parser.start()
        # ---------------------------------------------------------------------------------------------------------------------

        # -- Pairing the gas value and the GPS coords periodically -------------------------------------------------------------
        #GPS_gas_pairing()
        # ----------------------------------------------------------------------------------------------------------------------

        # start the main endless loop, exit only when 'exit' is typed on keyboard
        g_var.running = True

        while g_var.running:
            #print("drone_d1_list[0]: " + str(drone_d1_list[0]))
            #print("ALTITUDE: " + str(d_l[0].d2[0]))
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
                #command to land all the drones together
                elif g_var.CONTINUE == 3:
                    g_var.CURRENT_TASK = temp_command['command']
                    #print "g_var.continue set to 3: LANDING ALL THE DRONES"
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
    hm_p.join()

    # -- Close vehicle objects and connections before exiting script -------------------------------------------------------
    print("\nClosing all connections and vehicle objects")

    for i in s_l:
        i.close()
        i.join(True)

    gas_parser.join()
    print('bye-bye')
    #vehicle_1.close()
    print("Completed")
    #sys.exit(0)

    swarm_logger.info("Done")
    print 'End'

