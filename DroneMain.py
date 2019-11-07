#import libraries
import DroneIO
import time
import sys
import zmq
from twisted.internet import task, reactor
# setup all variables, functions / one time setup:
stream = zmq.Context()
socket = stream.socket(zmq.REP)
socket.bind("tcp://*:5555")
timeout = 0.01

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
def MainLoop():
    incoming = socket.recv()
    print("Received MSG: %s" % incoming)


#begin MainLoop cycle
taskloop = task.LoopingCall(MainLoop)
taskloop.start(timeout)

reactor.run()
