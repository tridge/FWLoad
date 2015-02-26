
mavlist = None

def expect_callback(e):
    '''called when waiting for a expect pattern'''
    global mavlist
    if mavlist is not None:
        for mav in mavlink:
            mav.recv_match(type='HEARTBEAT', timeout=0.1)

def expect_setup_callback(e, callback):
    '''setup a callback that is called once a second while waiting for
       patterns'''
    import pexpect
    def _expect_callback(pattern, timeout=e.timeout):
        tstart = time.time()
        while time.time() < tstart + timeout:
            try:
                ret = e.expect_saved(pattern, timeout=1)
                return ret
            except pexpect.TIMEOUT:
                e.expect_user_callback(e)
                pass
        print("Timed out looking for %s" % pattern)
        raise pexpect.TIMEOUT(timeout)

    e.expect_user_callback = callback
    e.expect_saved = e.expect
    e.expect = _expect_callback


def expect_callback_mav(new_mavlist):
    global mavlist
    mavlist = new_mavlist
    
