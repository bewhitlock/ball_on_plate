from dynamixel_sdk import *
import time
import numpy as np
import cv2 as cv
import numpy as np

# --- Camera calibration ---
#mtx = np.array([[1231.53955, 0., 703.463229],
 #[0., 1235.03337, 426.147508],
 #[0., 0., 1.]])
#dist = np.array([[ 0.02869969, -0.24915749, -0.01115483, -0.00071546,  0.23742651]])

# Motor config
IDA = 1
IDB = 2
PORT = '/dev/ttyUSB0'
BAUDRATE = 2000000
PROTOCOL = 2.0

#from address table of XM-430 dynamixel
ADDR_MODE     = 11 #address of mode setting
ADDR_TORQUE   = 64 #address of torque enable
ADDR_GOAL_POS = 116 #address of position request
ADDR_GOAL_VEL = 104 #address of velocity request
ADDR_GOAL_CUR = 102 #address of current request
ADDR_POS      = 132 #address of position readout
ADDR_VEL      = 128 #address of velocity readout
ADDR_CUR      = 126 #address of current readout

#constants
POS_MAX = 4095 #previously initialized on motor in EEPROM
VEL_TO_RPM = 0.229 # unitless integer times this is RPM
CUR_TO_mA  = 2.69 # unitless integer times this is current in mA

#motor angle limits
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
            #print("bulk write worked")
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


#############camera initialization#############
#start camera
cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Camera not found!")
    exit()

# undistortion map
ret, frame = cap.read()
#h, w = frame.shape[:2]
#mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, mtx, (w, h), cv.CV_32FC1)

#color range
lower = np.array([40, 130, 60]) #for ball: lower = np.array([40, 150, 60])
upper = np.array([100, 230, 255]) #for ball: upper = np.array([100, 255, 255])
###############################################

##################start motor##################
mode(3)
Ainit = checkPosition(IDA)
Binit = checkPosition(IDB)
print(f"ainit == {Ainit} | binit == {Binit}")
print(f"pos A == {(checkPosition(IDA)-Ainit)}")
print(f"pos B == {(checkPosition(IDB)-Binit)}")
###############################################

###############get frame crop##################
XMIN = 250 #default
XMAX = 600
YMIN = 40
YMAX = 400


input("Center")

_, frame = cap.read()
#Undistort and convert to hsv
#frame = cv.remap(frame, mapx, mapy, cv.INTER_LINEAR)
hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
mask = cv.inRange(hsv, lower, upper)
contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
x, y = 0, 0
if contours:
    c = max(contours, key=cv.contourArea)
    m = cv.moments(c)
    if m["m00"] != 0:
        x = int(m["m10"] / m["m00"])
        y = int(m["m01"] / m["m00"])
        cv.circle(frame, (x, y), 5, (0, 0, 255), -1)
    XMIN = int(x - 416.75)
    XMAX = int(x + 416.75)

    YMIN = int(y - 416.75)
    YMAX = int(y + 416.75)
print(f"Xmin == {XMIN} Xmax == {XMAX}")
print(f"Ymin == {YMIN} Ymax == {YMAX}")
###############################################

################control loop###################

########PID VALUES#########
dt = 0.016
prevAerror = 0
prevBerror = 0
Aerror = 0
Berror = 0
IAerror = 0
IBerror = 0

#motor A
PA = 0.3
IA = 0.00
DA = 0.00
#motor B
PB = PA
IB = IA
DB = DA
try:
    while True:
        _, frame = cap.read()

        # Undistort and convert to hsv
        #frame = cv.remap(frame, mapx, mapy, cv.INTER_LINEAR)
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, lower, upper)

        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        x, y = 0, 0
        if contours:
            c = max(contours, key=cv.contourArea)
            m = cv.moments(c)
            if m["m00"] != 0:
                x = int(m["m10"] / m["m00"])
                y = int(m["m01"] / m["m00"])
                cv.circle(frame, (x, y), 5, (0, 0, 255), -1)
        
        x = np.clip(x, XMIN, XMAX) #clamp x and y values
        y = np.clip(y, YMIN, YMAX)
        #print(f"({x},{y})")
        goalA = int( ( (1 - ((y - YMIN) / (YMAX - YMIN))) * (AMAX - AMIN)) + AMIN )
        goalB = int( ( (1 - ((x - XMIN) / (XMAX - XMIN))) * (BMAX - BMIN)) + BMIN )
        print(f"rangex == {round((x - XMIN)/(XMAX - XMIN), 2)} rangey == {round((y - YMIN)/(YMAX - YMIN), 2)}")
        if (contours):
            #PID for motor A
            presentA = checkPosition(IDA)
            Aerror = (goalA - (presentA - Ainit))
            IAerror += (dt * Aerror)
            IAerror = np.clip(IAerror, -200, 200)
            DAerror = (Aerror - prevAerror) / dt
            prevAerror = Aerror
            PIDA = (Aerror * PA) + (IAerror * IA) + (DAerror * DA)

            #PID for motor B
            presentB = checkPosition(IDB)
            Berror = (goalB - (presentB - Binit))
            IBerror += (dt * Berror)
            IBerror = np.clip(IBerror, -200, 200)
            DBerror = (Berror - prevBerror) / dt
            prevBerror = Berror
            PIDB = (Berror * PB) + (IBerror * IB) + (DBerror * DB)

            adjustedA = (checkPosition(IDA) - Ainit) + int(PIDA)
            adjustedB = (presentB - Binit) + int(PIDB)

            if (( ((adjustedA) > AMIN) and ((adjustedA) < AMAX) ) and ( ((adjustedB) > BMIN) and ((adjustedB) < BMAX) )):
                setPosition((adjustedA + Ainit), (adjustedB + Binit))
            else:
                print(f"") #out of bounds request goal A: {goalA} goal B: {goalB}
        else:
            print("tracking lost")



        time.sleep(dt) ####################### refresh rate --> 50 hz
        #cv.imshow("frame", frame)
        #cv.imshow("mask", mask)
        #if cv.waitKey(1) == 27:  # ESC to quit
            #break
except KeyboardInterrupt:
    print("Keyboard interrupt detected")
finally:
    print("Terminating")
    # Finish program
    time.sleep(5)
    torque(False)
    port.closePort()

    cap.release()
    #cv.destroyAllWindows()

    print("\nDone!")