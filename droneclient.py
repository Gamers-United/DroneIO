import socket
import keyboard
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = ('127.0.0.1', 10000)

sock.connect(server_address)

while True:
    if keyboard.is_pressed('w'):
        sock.send("w".encode('utf-8'))
        print("Button W Pressed")
    if keyboard.is_pressed('a'):
        print("Button A Pressed")
        sock.send("a".encode('utf-8'))
    if keyboard.is_pressed('s'):
        print("Button S Pressed")
        sock.send("s".encode('utf-8'))
    if keyboard.is_pressed('d'):
        print("Button D Pressed")
        sock.send("d".encode('utf-8'))
    if keyboard.is_pressed('z'):
        print("Button Z Pressed")
        sock.send("z".encode('utf-8'))
    if keyboard.is_pressed('x'):
        print("Button X Pressed")
        sock.send("x".encode('utf-8'))
    if keyboard.is_pressed('q'):
        sock.send("q".encode('utf-8'))
        print("Button Q Pressed")
    if keyboard.is_pressed('e'):
        print("Button E Pressed")
        sock.send("e".encode('utf-8'))
    if keyboard.is_pressed('left'):
        print("Button Left Arrow Pressed")
        sock.send("la".encode('utf-8'))
    if keyboard.is_pressed('right'):
        print("Button Right Arrow Pressed")
        sock.send("ra".encode('utf-8'))
    if keyboard.is_pressed('up'):
        print("Button Up Arrow Pressed")
        sock.send("ua".encode('utf-8'))
    if keyboard.is_pressed('down'):
        print("Button Down Arrow Pressed")
        sock.send("da".encode('utf-8'))
    time.sleep(0.1)
sock.close()

