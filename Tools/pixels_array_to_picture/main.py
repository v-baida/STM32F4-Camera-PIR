from PIL import Image
import numpy as np
import serial
import math
from datetime import datetime

w, h = 320, 240
l, c = 0, 0
raw_pixel_arr = []
data = np.zeros((h, w, 3), dtype=np.uint8)

stmData = serial.Serial('com3', 115200)
cnt = 1
while True:
    cnt_str = str(cnt)
    print(cnt_str + ") Waiting")
    cnt = cnt + 1

    raw_pixel_arr = stmData.read(w * h * 2)
    now = datetime.now()
    dt_string = now.strftime("%d%m%Y_%H-%M-%S")


    for i in range (0, (h * w) - 1):
        RGB = ((raw_pixel_arr[i*2])) | (raw_pixel_arr[i*2+1] << 8)
        R5 = (RGB & 0xF800) >> 11
        G6 = (RGB & 0x07E0) >> 5
        B5 = (RGB & 0x001F)
        R8 = round(R5 * 255/31)
        G8 = round(G6 * 255/63)
        B8 = round(B5 * 255/31)

        l = math.floor(i / w)
        c = i - w * l
        data[l][c] = [R8, G8, B8]


    img = Image.fromarray(data, 'RGB')
    img.save('save/pic_' + dt_string + '.png')
