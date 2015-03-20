'''
handle log files for factory load
'''

import util, time, os, glob, serial, logging
from config import *

current_logdir = None
ftdi_device = None

def new_log_dir():
    '''create a new log directory for a run'''
    global current_logdir
    current_logdir = None
    dirname = "logs/%s" % time.strftime("%Y-%m-%d")
    util.mkdir_p(dirname)
    highest = None
    for i in range(1, 1000000):
        dir = os.path.join(dirname, 'run%u' % i)
        if not os.path.exists(dir):
            current_logdir = dir
            util.mkdir_p(dir)
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
            stream = self.stream
            ftdi_device = get_ftdi()
            ftdi_device.write(msg)
            ftdi_device.write('\r\n')
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            self.handleError(record)

# create a logger for 'testjig'
log = logging.getLogger('testjig')
log.setLevel(logging.DEBUG)

# create file handler which logs debug messages
print(get_log_dir())
log_fh = logging.FileHandler(os.path.join(get_log_dir(), "run.log"))
log_fh.setLevel(logging.DEBUG)

# create a console handler with a higher log level
log_ch = logging.StreamHandler()
log_ch.setLevel(logging.DEBUG)

# create a serial handler for output using a FTDI
log_ser = SerialHandler()
log_ser.setLevel(logging.DEBUG)

# create a formatter and add it to the handlers
log_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
log_fh.setFormatter(log_formatter)
log_ch.setFormatter(log_formatter)
log_ser.setFormatter(log_formatter)

# add the handlers to the logger
log.addHandler(log_fh)
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
