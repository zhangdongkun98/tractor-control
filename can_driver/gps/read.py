

import serial

def parse_rtk(data):
    sp_data = data.split(',')
    # for i in range(len(sp_data)):
    #     print(i, ':', sp_data[i])
    info = {}
    info['heading'] = float(sp_data[3]) # 0 - 359.99
    info['pitch'] = float(sp_data[4]) # -90 - 90
    info['roll'] = float(sp_data[5]) # -180 - 180
    info['gyro_x'] = float(sp_data[6])
    info['gyro_y'] = float(sp_data[7])
    info['gyro_z'] = float(sp_data[8])
    info['acc_x'] = float(sp_data[9]) # g
    info['acc_y'] = float(sp_data[10]) # g
    info['acc_z'] = float(sp_data[11]) # g
    info['lattitude'] = float(sp_data[12])
    info['longitude '] = float(sp_data[13])
    info['altitude '] = float(sp_data[14])
    info['v_e'] = float(sp_data[15]) # m/s
    info['v_n'] = float(sp_data[16]) # m/s
    info['v_u'] = float(sp_data[17]) # m/s
    info['v'] = float(sp_data[18]) # m/s
    info['num_satellite_1'] = int(sp_data[19])
    info['num_satellite_2'] = int(sp_data[20])
    return info



# with open('gps.txt', 'r') as file:
#     lines = file.readlines()
#     for line in lines:
#         if line[:6] != '$GPCHC':
#             continue
#         else:
#             info = parse_rtk(line)
#             print(info)

ser = serial.Serial('COM3', 115200)
ser.readline()

while True:
    if line[:6] != '$GPCHC':
        continue
    else:
        info = parse_rtk(line)
        print(info)