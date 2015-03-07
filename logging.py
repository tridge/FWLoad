'''
handle log files for factory load
'''

import util, time, os

current_logdir = None

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
    
