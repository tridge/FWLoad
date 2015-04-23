'''
handle log files for factory load
'''

import util, time, os, glob, serial, logging, power_control
from config import *

current_logdir = None
ftdi_device = None

def mkdir_p(dir):
    '''like mkdir -p'''
    if not dir:
        return
    if dir.endswith("/"):
        mkdir_p(dir[:-1])
        return
    if os.path.isdir(dir):
        return
    mkdir_p(os.path.dirname(dir))
    os.mkdir(dir)

def new_log_dir():
    '''create a new log directory for a run'''
    global current_logdir
    current_logdir = None
    dirname = "logs/%s" % time.strftime("%Y-%m-%d")
    mkdir_p(dirname)
    highest = None
    for i in range(1, 1000000):
        dir = os.path.join(dirname, 'run%u' % i)
        if not os.path.exists(dir):
            current_logdir = dir
            mkdir_p(dir)
            return dir
    util.failure("Failed to find log directory")
    

def get_log_dir():
    '''get the current log directory'''
    global current_logdir
    if not current_logdir:
        new_log_dir()
    return current_logdir

def new_tlog(basename, extension='tlog'):
    '''get a new tlog name in log directory'''
    global current_logdir
    if current_logdir is None:
        return basename + "." + extension
    for i in range(1, 1000000):
        tlog = os.path.join(current_logdir, '%s-%u.' % (basename, i)) + extension
        if not os.path.exists(tlog):
            return tlog
    util.failure("Unable to create new tlog for %s" % basename)
    
def get_ftdi():
    global ftdi_device
    if not ftdi_device:
        ftdi_devices = glob.glob(FTDI_POWER)
        if len(ftdi_devices) != 1:
            util.failure("Must be exactly 1 FTDI device - %u found" % len(ftdi_devices))
        ftdi_device = serial.Serial(ftdi_devices[0], rtscts=False)
        power_control.on()
    return ftdi_device

def get_timestamp():
    return time.strftime("%H:%M:%S", time.localtime())

class SerialHandler(logging.StreamHandler):
    """
    A handler class which allows the logging stream to
    be written to a serial port
    """
    def emit(self, record):
        try:
            msg = self.format(record)
            dev = get_ftdi()
            dev.write(msg)
            dev.write('\r\n')
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            self.handleError(record)

def reopen_logfile():
    '''reopen the logfile to match new log directory'''
    global log_fh, log, log_formatter
    if log_fh is not None:
        log.removeHandler(log_fh)
    log_fh = logging.FileHandler(os.path.join(get_log_dir(), "run.log"))
    log_fh.setLevel(logging.DEBUG)
    log_fh.setFormatter(log_formatter)
    log.addHandler(log_fh)

# create a logger for 'testjig'
log = logging.getLogger('testjig')
log.setLevel(logging.DEBUG)

# create file handler which logs debug messages in reopen_logfile() above
log_fh = None

# create a console handler with a higher log level
log_ch = logging.StreamHandler()
log_ch.setLevel(logging.DEBUG)

# create a serial handler for output using a FTDI
log_ser = SerialHandler()
log_ser.setLevel(logging.DEBUG)

# create a formatter and add it to the handlers
log_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
log_ch.setFormatter(log_formatter)
log_ser.setFormatter(log_formatter)

# add the handlers to the logger
log.addHandler(log_ch)
log.addHandler(log_ser)

def error(str):
    log.error(str)

def info(str):
    log.info(str)

def debug(str):
    log.debug(str)

def critical(str):
    log.critical(str)
