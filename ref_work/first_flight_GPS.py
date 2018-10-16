import time, math, signal, argparse, threading, logging, re, socket, sys
from time import gmtime, strftime
from custom_library import drone_lib
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil # for command message definitions

#-- logging definition
logging.basicConfig(level=logging.DEBUG,
                    format='[%(levelname)s] (%(threadName)-10s) %(message)s',
                    )


#-- thread for gas reading
def gas_reading():

    logging.debug('Starting gas reading thread')
    split = re.split(':', tcpip_string)
    TCP_IP1 = split[0]
    TCP_PORT_1 = int(split[1])
    BUFFER_SIZE = 60
    q_max = 10
    s_1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s_1.bind((TCP_IP1, TCP_PORT_1))
    s_1.listen(q_max)

    # compile a pattern, in this case: match every occurrence of a float number x in a "..0 : x .. hours:min:sec" pattern
    pattern_gas_generico_0 = re.compile('(?<=0 : )[-+]?\d+\.\d* \d+[:]\d+[:]\d+')

    # compile a pattern, in this case: every occurrence of a float number x in a "..1 : x .. hours:min:sec" pattern
    pattern_gas_generico_1 = re.compile('(?<=1 : )[-+]?\d+\.\d* \d+[:]\d+[:]\d+')

    #-- CSV file creation ------------------------------------------------------------------------------------------
    logging.debug('CSV file creation')
    """
    with open('gas_data_log.csv', 'a') as the_file:
        the_file.write('Time [Laptop]' + ';' + 'Time [BBBlue]' + ';' + 'Sensor Number;' + 'Sensor Data' + '\n') #timestamp_laptop,timestamp_beaglebone,0,gas_sensor_data
    """

    logging.debug('Start reading data')
    while 1:
        data = conn_1.recv(BUFFER_SIZE)
        if not data: break
        data_str = str(data) # cast to a string the incoming data (just for safety reasons)

        #-- find all occurrences of the previously specified patterns in the incoming data -------------------------
        result_gas_generico_0 = re.findall(pattern_gas_generico_0, data_str)

        result_gas_generico_1 = re.findall(pattern_gas_generico_1, data_str)

        timestamp = strftime("%H:%M:%S", time.localtime())

        #-- print all the occurrences for gas sensor 0 -------------------------------------------------------------
        for p in result_gas_generico_0:
            split0 = p.split()
            logging.debug('gas sensor 0: ' + str(split0[0]) + ' at time: ' + str(split0[1]))
            """
            with open('gas_data_log.csv', 'a') as the_file:
                the_file.write(timestamp + ';' + split0[1] + ';' + '0;' + split0[0] + '\n') #timestamp_laptop,timestamp_beaglebone,0,gas_sensor_data
            """

        #-- print all the occurrences for gas sensor 1 -------------------------------------------------------------
        for p in result_gas_generico_1:
            split1 = p.split()
            logging.debug('gas sensor 1: ' + str(split1[0]) + ' at time: ' + str(split1[1]))
            """
            with open('gas_data_log.csv', 'a') as the_file:
                the_file.write(timestamp + ';' + split1[1] + ';' + '1;' + split1[0] + '\n') #timestamp_laptop,timestamp_beaglebone,0,gas_sensor_data
            """



#--------------------------------- MAIN PROGRAM -------------------------------------#


# --------------------------- MODIFY HERE THE PATH TO THE FILE--------------------------------------------

firstline = True
latitude = []
longitude = []
altitude = []

for line in open(r'C:\Users\PC\Downloads\WinPython-64bit-2.7.13.1Zero\grill_park_2.waypoints', 'r'):
                                                                                                      #  test batteria al Grill Park: grill_park_2.waypoints
                                                                                                      # test batteria al Parco Oltrecastello: parco_di_oltrecastello_distance_test.waypoints
                                                                                                      # parco_oltrecastello_distance_conservativo_26_m
                                                                                                      # test GPS: parco_di_oltrecastello_path.waypoints

    # ---------------------------------------------------------------------------------------------------------
    if firstline == True:  # discard the first line of text
        firstline = False
        continue
    # -- parsing each line into latitude, longitude and altitude, index 0 is the home location
    # -- WARNING! : SET IT PROPERLY DURING STARTUP
    s = line.rstrip()
    result = s.split('\t')
    latitude.append(result[8])
    longitude.append(result[9])
    altitude.append(result[10])



#-- Connect to the vehicle

parser = argparse.ArgumentParser(description='First Flight - Arm, Takeoff and Loiter')
parser.add_argument('--connect')
parser.add_argument('--tcpip')
args = parser.parse_args()

connection_string = args.connect
tcpip_string = args.tcpip

print("Connection to the vehicle on %s"%connection_string)
vehicle = connect(connection_string, wait_ready=True)

#-- start gas_reading thread if the script is launched with a TCP/IP connection string
if tcpip_string:
    threading1 = threading.Thread(target=gas_reading)
    threading1.daemon = True
    threading1.start()

#-- taking off at altitudes [m]
altitudes = 4 #4 #2.5
print("-- Arming and taking off at {} meters --".format(altitudes))

signal.signal(signal.SIGINT, signal.default_int_handler) # interrupt handler

print("FENCE_ENABLE: %s" % vehicle.parameters['FENCE_ENABLE'])
print("FENCE_TYPE: %s" % vehicle.parameters['FENCE_TYPE'])
print("FENCE_ACTION: %s" % vehicle.parameters['FENCE_ACTION'])
print("FENCE_ALT_MAX: %s" % vehicle.parameters['FENCE_ALT_MAX'])
print("FENCE_RADIUS: %s" % vehicle.parameters['FENCE_RADIUS'])
print("FENCE_MARGIN: %s" % vehicle.parameters['FENCE_MARGIN'])
print("RTL_ALT: %s" % vehicle.parameters['RTL_ALT'])

vehicle.parameters['FENCE_ENABLE'] = 0
vehicle.parameters['FENCE_ALT_MAX'] = 11
vehicle.parameters['FENCE_RADIUS'] = 35
vehicle.parameters['FENCE_MARGIN'] = 2
vehicle.parameters['RTL_ALT'] = 600

print("FENCE_ENABLE: %s" % vehicle.parameters['FENCE_ENABLE'])
print("FENCE_TYPE: %s" % vehicle.parameters['FENCE_TYPE'])
print("FENCE_ACTION: %s" % vehicle.parameters['FENCE_ACTION'])
print("FENCE_ALT_MAX: %s" % vehicle.parameters['FENCE_ALT_MAX'])
print("FENCE_RADIUS: %s" % vehicle.parameters['FENCE_RADIUS'])
print("FENCE_MARGIN: %s" % vehicle.parameters['FENCE_MARGIN'])
print("RTL_ALT: %s" % vehicle.parameters['RTL_ALT'])

time.sleep(6)

try:
    #-- taking off at target altitudes
    drone_lib.arm_and_takeoff(altitudes, vehicle)
    time.sleep(6)
    #vehicle.mode = VehicleMode("LAND")

    #-- creation of a listener for system time: each time ArduCopter updates the system time, it prints on video and on CVS file the GPS data
    #@vehicle.on_message('SYSTEM_TIME')
    #def listener(self, name, message):
        #timestamp = strftime("%H:%M:%S", time.localtime())
        #print('\nListener: ' + str(self.location.global_relative_frame))
        #gps_timestamp = gmtime(message.time_unix_usec/1000000.)
        #gps_hour = (gps_timestamp.tm_hour+1) % 24
        #gps_min = gps_timestamp.tm_min
        #gps_sec = gps_timestamp.tm_sec
        #print('Listener: [GPS Time] : %d:%d:%d' % (gps_hour, gps_min, gps_sec));
        #print('Listener: [Laptop Time] : %s\n' % (strftime("%H:%M:%S", time.localtime())))
        #with open('GPS_log.csv', 'a') as the_file:
        #	the_file.write(timestamp + ';' + str(gps_hour)+ ':' + str(gps_min) + ':' + str(gps_sec) + ';' + str(self.location.global_relative_frame.lat) + ';' + str(self.location.global_relative_frame.lon) + ';' + str(self.location.global_relative_frame.alt)+ '\n') #timestamp_laptop,timestamp_arducopter,gps_data (lat, lon, alt)

    print("Drone 1 - Start Patrolling")
    airspeed            = 1  # 7
    groundspeed         = 5
    # print("-- Setting the airspeed to {} m/s --".format(airspeed))
    #vehicle.airspeed    = airspeed
    print("-- Setting the groundspeed to {} m/s --".format(groundspeed))
    vehicle.groundspeed = groundspeed
    distance_counter = 0  # increment each time it reach a waypoint                                          # JUST ADDED

    while True:                                                                                             # JUST ADDED
        for j in range(len(latitude)):
            if j == 0:  # discard first value, for it is the home location chosen when the waypoints file was created
                continue
            targetLocation = LocationGlobalRelative(float(latitude[j]), float(longitude[j]),
                                                altitudes)
            drone_lib.simple_goto_wait(targetLocation, vehicle)
            distance_counter = distance_counter + 1                                                         # JUST ADDED
            print("--- WAYPOINT " + str(distance_counter) + " REACHED ---")
            print("going to the next waypoint")
            time.sleep(3)                                                                                 # JUST ADDED
            #drone1_target += 1

    # -- landing
    # print("-- Landing with an RTL command --")
    # vehicle.mode = VehicleMode("RTL")

    print("-- Landing with a LAND command --")
    vehicle.mode = VehicleMode("LAND")

    #time.sleep(5)


    #-- landing
    #print("-- Landing with an RTL command --")
    #vehicle.mode = VehicleMode("RTL")

    time.sleep(300)

    #--close connection
    vehicle.close()
except KeyboardInterrupt:
    print('!! Keyboard Interrupt !! - CAUTION: LANDING!')
    vehicle.mode = VehicleMode("LAND")
    time.sleep(50)
    vehicle.close()
    sys.exit()
