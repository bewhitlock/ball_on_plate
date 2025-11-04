from dynamixel_sdk import *
import time
import numpy as np

# Motor config
IDA = 1
IDB = 2
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

AMIN = -150
AMAX = 300
BMIN = -250
BMAX = 300
# initialization
port = PortHandler(PORT)
pkt  = PacketHandler(PROTOCOL)
groupBulkWrite = GroupBulkWrite(port, pkt)
if not port.openPort() or not port.setBaudRate(BAUDRATE):
    print("Failed to open port or set baudrate")
    quit()


# helper functions
def torque(on=True):
    pkt.write1ByteTxRx(port, IDA, ADDR_TORQUE, 1 if on else 0)
    pkt.write1ByteTxRx(port, IDB, ADDR_TORQUE, 1 if on else 0)

def mode(m):   #3=pos, 1=vel, 0=cur
    torque(False)
    pkt.write1ByteTxRx(port, IDA, ADDR_MODE, m)
    pkt.write1ByteTxRx(port, IDB, ADDR_MODE, m)
    torque(True)

# control functions
def setPosition(goalA, goalB):
    param_goalA = [
        DXL_LOBYTE(DXL_LOWORD(goalA)),
        DXL_HIBYTE(DXL_LOWORD(goalA)),
        DXL_LOBYTE(DXL_HIWORD(goalA)),
        DXL_HIBYTE(DXL_HIWORD(goalA))
    ]
    param_goalB = [
        DXL_LOBYTE(DXL_LOWORD(goalB)),
        DXL_HIBYTE(DXL_LOWORD(goalB)),
        DXL_LOBYTE(DXL_HIWORD(goalB)),
        DXL_HIBYTE(DXL_HIWORD(goalB))
    ]
    for attempt in range(2): #make 2 attempts.
        groupBulkWrite.clearParam()
        param_resultA = groupBulkWrite.addParam(IDA, ADDR_GOAL_POS, 4, param_goalA)
        param_resultB = groupBulkWrite.addParam(IDB, ADDR_GOAL_POS, 4, param_goalB)
        if((not param_resultA) or (not param_resultB)):
            print("bulk param write failed")
        comm_result = groupBulkWrite.txPacket()
        if comm_result == COMM_SUCCESS:
            print("bulk write worked")
            break
        else:
            print("comm error!")
            groupBulkWrite.clearParam()
            time.sleep(0.01)
            
#reading functions
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


#demo sequence

mode(3)
Ainit = checkPosition(IDA)
Binit = checkPosition(IDB)
print(f"ainit == {Ainit} | binit == {Binit}")
print(f"pos A == {(checkPosition(IDA)-Ainit)}")
print(f"pos B == {(checkPosition(IDB)-Binit)}")


for i in range(4):
    usrA = int(input("enter A goto:"))
    usrB = int(input("enter B goto:"))
    if (( ((usrA) > AMIN) and ((usrA) < AMAX) ) and ( ((usrB) > BMIN) and ((usrB) < BMAX) )):
        setPosition((usrA + Ainit), (usrB + Binit))
    else:
        print("out of bounds request")
# Finish program
time.sleep(10)
torque(False)
port.closePort()
print("\nDone!")