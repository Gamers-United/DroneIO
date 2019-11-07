#import libraries
import DroneIO
import time
import sys
import socket
from twisted.internet import task, reactor
# setup all variables, functions / one time setup:
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = ('192.168.1.84', 10000)
print("Starting up on "+str(server_address))
sock.bind(server_address)
timeout = 0.01
sock.listen(1)
i=1000
while i>=0:
    print("Waiting For Connection")
    connection, client_address = sock.accept()
    i = i - 1

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
    incoming = sock.recv(2)
    print("Received MSG: %s" % incoming)


#begin MainLoop cycle
taskloop = task.LoopingCall(MainLoop)
taskloop.start(timeout)

reactor.run()
