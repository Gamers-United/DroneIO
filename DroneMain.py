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
sock.listen(2)
connection, client_address = sock.accept()
drone = DroneIO.DroneControl()


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
#called when Z key pressed on controlling computer
def ZKey():

    return
#called when X key pressed on controlling computer
def XKey():

    return
#For Processing Accelerometer data
def processAccel():

    return
#For Processing Gyro data
def processGyro():

    return
#For Processing Barometer data
def processBaro():

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
        WKey()
    if message == "a":
        AKey()
    if message == "s":
        SKey()
    if message == "d":
        DKey()
    if message == "x":
        XKey()
    if message == "z":
        ZKey()
    processGyro(drone)
    processAccel(drone)
    processBaro(drone)

#begin MainLoop cycle
taskloop = task.LoopingCall(MainLoop)
taskloop.start(timeout)

reactor.run()
