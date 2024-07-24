import os

from datetime import datetime
import json
import yaml

if os.name == 'nt':
    # Windows
    import msvcrt
else:
    # Posix (Linux, OS X)
    import sys
    import termios
    import atexit
    from select import select

__all__ = ['KeyboardIn', 'double_check_with_user', 'write_recovery_file', 'check_recovery_file',
           'log_error', 'get_defined_tests_list', 'get_email_creds', 'rm_recovery_file',
           'append_to_save_file']


class KeyboardIn:
    def __init__(self):
        if os.name != 'nt':
            # Save the terminal settings
            self.fd = sys.stdin.fileno()
            self.new_term = termios.tcgetattr(self.fd)
            self.old_term = termios.tcgetattr(self.fd)

            # New terminal setting unbuffered
            self.new_term[3] = (self.new_term[3] & ~termios.ICANON & ~termios.ECHO)
            termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new_term)

            # Support normal-terminal reset at exit
            atexit.register(self.set_normal_term)

    def set_normal_term(self):
        if os.name != 'nt':
            termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old_term)

    def new_event(self):
        if os.name == 'nt':
            return msvcrt.kbhit()
        else:
            dr, dw, de = select([sys.stdin], [], [], 0)
            return dr != []

    def getkey(self):
        if os.name == 'nt':
            return msvcrt.getch().decode('utf-8')
        else:
            return sys.stdin.read(1)


def double_check_with_user(kb, prompt):
        print(f"{prompt} (y/n)")
        while True:
            if kb.new_event():
                cmd = kb.getkey()
                match str(cmd):
                    case 'y':
                        return True
                    case 'n':
                        return False
                    case _:
                        pass


### FILE READ AND WRITE HANDLING ###

# Configuration info
RECOVERY_FILE = 'data/recovery.txt'
ERROR_FILE = 'data/errors.log'


def from_yaml(filename):
    with open(filename) as f:
        data = yaml.safe_load(f)
    return data


def from_json(filename):
    with open(filename) as f:
        data = json.load(f)
    return data


def get_defined_tests_list():
    test_defs = from_yaml('test_defs.yaml')
    return list(test_defs.keys())


def get_email_creds():
    # Read email server credentials and test connection
    with open('email_creds.txt', 'r') as f:
        source_email = f.readline().rstrip()
        email_password = f.readline().rstrip()
    return source_email, email_password


def check_recovery_file(kb):
    if os.path.exists(RECOVERY_FILE):
        if double_check_with_user(kb, "Would you like to resume the last test that did not complete?"):
            with open(RECOVERY_FILE, 'r') as f:
                test_to_load = f.readline().rstrip()
                available_tests = get_defined_tests_list()
                if not test_to_load in available_tests:
                    print(f"Recovered test {test_to_load} is not defined, resumption cancelled.")
                else:
                    test_name = test_to_load
                    curr_step_index = int(f.readline().rstrip())
                    save_file_name = f.readline().rstrip()
                    return test_name, curr_step_index, save_file_name
        else:
            os.remove(RECOVERY_FILE)
    return None, 10000, None


def write_recovery_file(test, curr_step, save_file):
    with open(RECOVERY_FILE, 'w') as f:
        f.write(f"{test}\n{curr_step}\n{save_file}")


def rm_recovery_file():
    os.remove(RECOVERY_FILE)


def log_error(e):
    # Need this try block as we're running this in multiple processes with potential for collision
    try:
        with open(ERROR_FILE, 'a') as f:
            f.write(f"\n\nError at {datetime.utcnow()}\n")
            f.write(str(e))
    except Exception as e:
        print(e)
        print(f"Tried to write: {str(e)}")


def append_to_save_file(file_name, msg):
    file = f"data/{file_name}"
    with open(file, 'a') as f:
        f.write(msg)

