#import libraries
import DroneIO
import time
import sys
import socket
import math
from twisted.internet import task, reactor
# setup all variables, functions / one time setup:
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = ('127.0.0.1', 10000)
print("Starting up on "+str(server_address))
sock.bind(server_address)
timeout = 0.01
sock.listen(2)
connection, client_address = sock.accept()
drone = DroneIO.DroneControl()


#called when W key pressed on controlling computer
def WKey(drone):

    return
#called when S key pressed on controlling computer
def SKey(drone):

    return
#called when A key pressed on controlling computer
def AKey(drone):

    return
#called when D key pressed on controlling computer
def DKey(drone):

    return
#called when Z key pressed on controlling computer
def ZKey(drone):

    return
#called when X key pressed on controlling computer
def XKey(drone):

    return
#For Processing Accelerometer data
def processAccel(drone):

    return
#For Processing Gyro data
def processGyro(drone):

    return
#For Processing Barometer data
def processBaro(drone):

    return

#store starting time
starttime=time.time()
# begin function loop
def MainLoop():
    incoming = connection.recv(1024)
    print("Received MSG: %s" % incoming.decode('utf-8'))
    message = incoming.decode('utf-8')
    #check all keys pressed
    if message == "w":
        WKey(drone)
    if message == "a":
        AKey(drone)
    if message == "s":
        SKey(drone)
    if message == "d":
        DKey(drone)
    if message == "x":
        XKey(drone)
    if message == "z":
        ZKey(drone)
    processGyro(drone)
    processAccel(drone)
    processBaro(drone)

#begin MainLoop cycle
taskloop = task.LoopingCall(MainLoop)
taskloop.start(timeout)

reactor.run()
