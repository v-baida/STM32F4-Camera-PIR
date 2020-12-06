from PIL import Image
import numpy as np
import serial
import math
from datetime import datetime
import time
import socket
import sys

start_time = time.time()

w, h = 320, 240
l, c = 0, 0
raw_pixel_arr = []
data = np.zeros((h, w, 3), dtype=np.uint8)

#stmData = serial.Serial('com3', 115200)

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Connect the socket to the port where the server is listening
server_address = ('192.168.1.191', 7)
sock.connect(server_address)

message = 'I`m ready'
msg_bytes = bytes(message, 'utf-8')

while True:

    #raw_pixel_arr = stmData.read(w * h * 2)

    now = datetime.now()
    dt_string = now.strftime("%d%m%Y_%H-%M-%S")

    amount_received = 0
    amount_expected = 320 * 240 * 2
    raw_pixel_arr = bytearray()

    sock.sendall(msg_bytes)

    print("Receiving %s" % (time.time() - start_time))
    while amount_received < amount_expected:
        TCP_data = sock.recv(100000000)
        amount_received += len(TCP_data)
        raw_pixel_arr += bytearray(TCP_data)

    print("Received %s" % (time.time() - start_time))
    print("Amount received = " + str(amount_received))

    for i in range(0, (h * w) - 1):
        RGB = ((raw_pixel_arr[i * 2])) | (raw_pixel_arr[i * 2 + 1] << 8)
        R5 = (RGB & 0xF800) >> 11
        G6 = (RGB & 0x07E0) >> 5
        B5 = (RGB & 0x001F)
        R8 = round(R5 * 255 / 31)
        G8 = round(G6 * 255 / 63)
        B8 = round(B5 * 255 / 31)

        l = math.floor(i / w)
        c = i - w * l
        data[l][c] = [R8, G8, B8]

    print("Processed %s" % (time.time() - start_time))
    img = Image.fromarray(data, 'RGB')
    img.save('save/pic_' + dt_string + '.png')
    print("Saved %s" % (time.time() - start_time))
    print("--------------------------")









