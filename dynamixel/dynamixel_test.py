from dynamixel_sdk import *
import time
import numpy as np

# Motor config
DXL_ID = 1
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
    pkt.write1ByteTxRx(port, DXL_ID, ADDR_TORQUE, 1 if on else 0)

def mode(m):   #3=pos, 1=vel, 0=cur
    torque(False)
    pkt.write1ByteTxRx(port, DXL_ID, ADDR_MODE, m)
    torque(True)

# control functions
def setPosition(deg):
    goal = int((deg % 360) * POS_MAX / 360)
    for attempt in range(2): #make 2 attempts.
        comm_result, hw_error = pkt.write4ByteTxRx(port, DXL_ID, ADDR_GOAL_POS, goal)
        if comm_result == COMM_SUCCESS:
            break
        else:
            print("comm error!")
            time.sleep(0.01)
    

def setVelocity(rpm):
    val = int(rpm / VEL_TO_RPM)
    pkt.write4ByteTxRx(port, DXL_ID, ADDR_GOAL_VEL, val) #val & 0xFFFFFFFF

def setTorque(mA):
    val = int(mA / CUR_TO_mA)
    for attempt in range(2): #make 2 attempts
        comm_result, hw_error = pkt.write2ByteTxRx(port, DXL_ID, ADDR_GOAL_CUR, val) #val & 0xFFFF
        if comm_result == COMM_SUCCESS:
            break
        else:
            print("comm error!")
            time.sleep(0.01)

#reading functions
def checkPosition():
    val = np.int16(0)
    for attempt in range(2): #make 2 attempts
        val, comm_result, hw_error = pkt.read4ByteTxRx(port, DXL_ID, ADDR_POS)
        if comm_result == COMM_SUCCESS:
            break
        else:
            print("comm error!")
            time.sleep(0.01)
    return ((float(val) / float(4095)) * 2 * 3.14159) #in rads

def checkCurrent():
    val = 0;
    for attempt in range(2): #make 2 attempts
        val, comm_result, hw_error = pkt.read2ByteTxRx(port, DXL_ID, ADDR_CUR)
        if comm_result == COMM_SUCCESS:
            break
        else:
            print("comm error!")
            time.sleep(0.01)

    if (val > 32767): # if msb is high, then subtract the msb(sign bit)
        val -= 65536
    return (val * CUR_TO_mA) #in mA

def checkVel():
    val = 0;
    for attempt in range(2): #make 2 attempts
        val, comm_result, hw_error = pkt.read4ByteTxRx(port, DXL_ID, ADDR_VEL)
        if comm_result == COMM_SUCCESS:
            break
        else:
            print("comm error!")
            time.sleep(0.01)
    return (((float(val) * VEL_TO_RPM)/60)* 2 * 3.14159) #in rads per second

def calcVel():
    si = checkPosition()
    t0 = time.time()
    time.sleep(0.025)
    sf = checkPosition()
    t1 = time.time()
    return ((sf - si)/(t1-t0)) #in rads per second


#demo sequence
print("\nPOSITION TEST")
print("\nGoing to 90 degrees")
mode(3)
setPosition(90)
time.sleep(1)
print(f"position == {checkPosition()} (rads)")
time.sleep(5)
print("\nGoing to 180 degrees")
setPosition(180)
time.sleep(1)
print(f"position == {checkPosition()} (rads)")
time.sleep(5)
print("\nGoing to 0 degrees")
setPosition(0)
time.sleep(1)
print(f"position == {checkPosition()} (rads)")
time.sleep(5)

print("\n\nVELOCITY TEST")
print("\nSetting velocity to 50 RPM")
mode(1)
setVelocity(50)
time.sleep(1)
print(f"calculated velocity == {calcVel()} rads/s")
print(f"read velocity == {checkVel()} rads/s")
time.sleep(1)
print("\nSetting velocity to -50 RPM")
setVelocity(-50)
time.sleep(1)
print("\nSetting velocity to 0 RPM")
setVelocity(0)
time.sleep(1)

print("\n\nTORQUE TEST")
print("\nSetting current to 200mA")
mode(0)
setTorque(200)
for i in range(4):
    time.sleep(1)
    print(f"current == {checkCurrent()} (mA)")
print("\nSetting current to -200mA")
setTorque(-200)
for i in range(4):
    time.sleep(1)
    print(f"current == {checkCurrent()} (mA)")
print("\nSetting current to 0mA")
setTorque(0)
print(f"position == {checkPosition()} (rads)")
for i in range(4):
    time.sleep(1)
    print(f"current == {checkCurrent()} (mA)")


# Finish program
torque(False)
port.closePort()
print("\nDone!")
