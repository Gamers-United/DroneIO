import DroneIO
import time
import sys
import zmq
# setup all variables, functions / one time setup:
stream = zmq.Context()
socket = stream.socket(zmq.REP)
socket.bind("tcp://*:5555")

#called when W key pressed on controlling computer
def WKey():

    return
#called when S key pressed on controlling computer
def SKey():

    return
#called when A key pressed on controlling computer
def AKey():

    return
#called when D key pressed on controlling computer
def DKey():

    return




#store starting time
starttime=time.time()
# begin function loop
while True:
    incoming = socket.recv()
    print("Received MSG: %s" % incoming)




    #wait 1/100th of a second between executions:
    time.sleep(0.01 - ((time.time() - starttime) % 60.0))
