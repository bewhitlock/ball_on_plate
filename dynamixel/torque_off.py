from dynamixel_sdk import *
import time

# Motor config
DXL_IDA = 1
DXL_IDB = 2
PORT = '/dev/ttyUSB0'
BAUDRATE = 2000000
PROTOCOL = 2.0

ADDR_MODE     = 11 #address of mode setting
ADDR_TORQUE   = 64 #address of torque enable
ADDR_GOAL_POS = 116 #address of position request
ADDR_GOAL_VEL = 104 #address of velocity request
ADDR_GOAL_CUR = 102 #address of current request
ADDR_POS      = 132 #address of position readout
ADDR_VEL      = 128 #address of velocity readout
ADDR_CUR      = 126 #address of current readout

POS_MAX = 4095 #previously initialized on motor in EEPROM
VEL_TO_RPM = 0.229 # unitless integer times this is RPM
CUR_TO_mA  = 2.69 # unitless integer times this is current in mA

# initialization
port = PortHandler(PORT)
pkt  = PacketHandler(PROTOCOL)
if not port.openPort() or not port.setBaudRate(BAUDRATE):
    print("Failed to open port or set baudrate")
    quit()

# helper functions
def torque(on=True):
    pkt.write1ByteTxRx(port, DXL_IDA, ADDR_TORQUE, 1 if on else 0)
    pkt.write1ByteTxRx(port, DXL_IDB, ADDR_TORQUE, 1 if on else 0)

# Finish program
torque(False)
port.closePort()
print("\nDone!")
