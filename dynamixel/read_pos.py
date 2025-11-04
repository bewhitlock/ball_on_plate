from dynamixel_sdk import *
import time
import numpy as np

# Motor config
AID = 1
BID = 2
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

# helper function
def torque(on=True, ID = 1):
    pkt.write1ByteTxRx(port, ID, ADDR_TORQUE, 1 if on else 0)


#reading function
def checkPosition(ID=1):
    for attempt in range(3):
        try:
            val, comm_result, hw_error = pkt.read4ByteTxRx(port, ID, ADDR_POS)
        except IndexError:
            print(f"IndexError on ID {ID} — bad response packet.")
            time.sleep(0.05)
            continue

        if comm_result == COMM_SUCCESS:
            if val > 0x7FFFFFFF:
                val -= 0x100000000
            return np.int32(val)
        else:
            print(f"Comm error on ID {ID}: {pkt.getTxRxResult(comm_result)}")
            time.sleep(0.05)
    return -1


#reading sequence
torque(False, AID)
torque(False, BID)

print("Press ctrl+c to end the program.")
ainit = checkPosition(AID)
binit = checkPosition(BID)
try:
    while True:
        print(f"A position: {(checkPosition(AID) - ainit):.3f} | B position: {(checkPosition(BID) - binit):.3f}")
        time.sleep(0.1)
except KeyboardInterrupt:
    pass


# Finish program
torque(False)
port.closePort()
print("\nDone!")
