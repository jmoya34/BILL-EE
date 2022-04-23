from ..TCP.server import *
from inputs import devices
import time
from control_mapping import XboxController


if __name__ == '__main__':
   cntrl_input = read_inputs()
   time.sleep(.01)
