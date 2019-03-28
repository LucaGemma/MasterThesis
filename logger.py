# ----------------------------------------------- Imports ------------------------------------------------------------ #
import logging, datetime
from swarmGlobals import uav_n, ctrl_l, fname, log_l, ver, base_ip, init_ip, base_ctrl_port, base_sens_port
from logging import FileHandler
from logging import Formatter
from custom_library import drone_lib

# ----------------------------------------- Additional Functions ----------------------------------------------------- #


def dateToString():

    """
    convert to a string the current date, including year, month, day, hour, minutes and seconds

    :return: the function returns a dictionary with the date data, made as follows:

    {'year': str(dt.year), 'month': month, 'day': day, 'hour': hour, 'min': min, 'sec': sec}
    """

    dt = datetime.datetime.today()
    month = str(dt.month) if dt.month > 9 else '0'+str(dt.month)
    day = str(dt.day) if dt.day > 9 else '0'+str(dt.day)
    hour = str(dt.hour) if dt.hour > 9 else '0'+str(dt.hour)
    min = str(dt.minute) if dt.minute > 9 else '0'+str(dt.minute)
    sec = str(dt.second) if dt.second > 9 else '0'+str(dt.second)
    res = {'year': str(dt.year), 'month': month, 'day': day, 'hour': hour, 'min': min, 'sec': sec}
    return res

# ---------------------------------------- Parameters declaration ---------------------------------------------------- #

filename_gas_data_log = []
filename_GPS_log = []
dt = dateToString()
f = '../logs/'+fname+'_'+ver+'_'+dt['year']+dt['month']+dt['day']+'-'+dt['hour']+dt['min']+dt['sec']+'.log'
filename_Pairing_log = '../logs/Pairing_log_' + dt['year'] + dt['month'] + dt['day'] + '-' + dt['hour'] + dt['min'] + dt['sec'] + '.txt'
LOG_FORMAT = ("[%(asctime)s][%(levelname)s]: %(message)s in %(pathname)s:%(lineno)d")
LOG_LEVEL = logging.DEBUG

swarm_logger = logging.getLogger("droneSwarmMng.swarm")
swarm_logger.setLevel(LOG_LEVEL)
swarm_logger_file_handler = FileHandler(f)
swarm_logger_file_handler.setLevel(LOG_LEVEL)
swarm_logger_file_handler.setFormatter(Formatter(LOG_FORMAT))
swarm_logger.addHandler(swarm_logger_file_handler)
