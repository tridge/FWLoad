
# see http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python

class bcolors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    CLEARSCREEN = '\033[2J'

def print_fail(msg):
    '''print a failure message'''
    print(bcolors.FAIL + msg + bcolors.ENDC)

def print_green(msg):
    '''print a green message'''
    print(bcolors.GREEN + bcolors.BOLD + msg + bcolors.ENDC)

def print_blue(msg):
    '''print a blue message'''
    print(bcolors.BLUE + bcolors.BOLD + msg + bcolors.ENDC)

def clear_screen():
    '''clear screen'''
    print(bcolors.CLEARSCREEN + bcolors.ENDC)


if __name__ == '__main__':
    print_green("test in green")
    print("no colour")
    print_fail('''
=====================
| TEST FAILED       |
=====================
''')
    
    
