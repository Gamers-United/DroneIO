#import libraries
import DroneIO
import time
import sys
import socket
from twisted.internet import task, reactor
# setup all variables, functions / one time setup:
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = ('localhost', 5555)
sock.bind(server_address)
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
    incoming = sock.recv()
    print("Received MSG: %s" % incoming)


#begin MainLoop cycle
taskloop = task.LoopingCall(MainLoop)
taskloop.start(timeout)

reactor.run()
