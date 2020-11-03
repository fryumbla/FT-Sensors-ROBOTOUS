import can
import time
import numpy as np
import matplotlib.pyplot as plt
bus = can.interface.Bus(bustype = 'kvaser', channel = 0, bitrate = 1000000)

# Set filter
tx_message = can.Message(arbitration_id = 0x64, is_extended_id = False, data = [0x08, 0x01, 0x09, 0x01, 0x01, 0x01, 0x01, 0x01])
bus.send(tx_message, timeout = 0.5)
bus.recv()

tx_message = can.Message(arbitration_id = 0x64, is_extended_id = False, data = [0x11, 0x01, 0x09, 0x01, 0x01, 0x01, 0x01, 0x01])
bus.send(tx_message, timeout = 0.5)
bus.recv()

# plt.rcParams['animation.html'] = 'jshtml'
fig = plt.figure()
ax = fig.add_subplot(111)
fig.show()
# plt.show(block = True)

# plt.style.use('seaborn-whitegrid')
plt.axis([0, 300, -10, 10])
plt.ion()
i = 0
t, x, y, z = [], [], [], []
while i<300:
    t.append(i)
    # read once
    tx_message = can.Message(arbitration_id = 0x64, is_extended_id = False, data = [0x0A, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01])
    bus.send(tx_message, timeout = 0.5)

    rx_message_1 = bus.recv()
    rx_message_2 = bus.recv()

    fx = ((rx_message_1.data[1]<<8) + rx_message_1.data[2])
    signed_fx = (-(fx & 0x8000) | (fx&0x7fff))/50.0
    x.append(signed_fx)
    fy = ((rx_message_1.data[3]<<8) + rx_message_1.data[4])
    signed_fy = (-(fy & 0x8000) | (fy&0x7fff))/50.0
    y.append(signed_fy)
    fz = ((rx_message_1.data[5]<<8) + rx_message_1.data[6])
    signed_fz = (-(fz & 0x8000) | (fz&0x7fff))/50.0
    print(signed_fz)
    z.append(signed_fz)
    ax.plot(t, x, color = 'r')
    ax.plot(t, y, color = 'g')
    ax.plot(t, z, color = 'b')
    
    fig.canvas.draw()
    ax.set_xlim(left = max(0, i-25), right = i+25)
    time.sleep(0.01)
    i += 1
    # plt.plot(i, signed_fx, marker = 'o', markerfacecolor = 'red', markersize = 3)
    # plt.plot(i, signed_fy, marker = 'o', markerfacecolor = 'green', markersize = 3)
    # plt.plot(i, signed_fz, marker = 'o', markerfacecolor = 'blue', markersize = 3)
    # # print("Fx = " + str(signed_fx) + "  Fy = " + str(signed_fy) + "  Fz = " + str(signed_fz))
    # plt.legend()
    # plt.pause(0.05)

np.savetxt("/home/susung/Desktop/sujong/data/kvaser/kvaser_mini.txt", np.array([[t], [x], [y], [z]]), fmt="%s")
# while True:
#     plt.pause(0.05)
'''
print(bus.recv())
print(bus.recv())



lower_byte = 0x01 # b0000 0001 : << 1 : 0000 0010 = 2/ <<2 0000 0100 = 4
upper_byte = 0x02 # b0000 0010


[upper_byte][lower_byte] [0x02] [0x01] 0000 0010 0000 0001
'''
