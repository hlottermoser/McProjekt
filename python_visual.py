
import sys
import glob
import serial
import matplotlib.pyplot as plt
import struct

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation



def serial_ports():
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

ser = serial.Serial(serial_ports()[0], 115200, timeout=1)

# First set up the figure, the axis, and the plot element we want to animate
#fig = plt.figure()
#ax = plt.axes(xlim=(0, 8000), ylim=(0, 700))
#line, = ax.plot([], [], lw=2)
b1 = []
b2 = []
b3 = []
a = []

# create a figure with two subplots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1)

# intialize two line objects (one in each axes)
line1, = ax1.plot([], [], lw=2)
line2, = ax2.plot([], [], lw=2, color='r')
line3, = ax3.plot([], [], lw=2, color="b")
line = [line1, line2, line3]

for ax in [ax1, ax2, ax3]:
    ax.set_ylim(-3, 3)
    ax.set_xlim(0, 7000)
    ax.grid()


# initialization function: plot the background of each frame
def init():
    line[0].set_data([],[])
    line[1].set_data([],[])
    line[2].set_data([],[])
    return line

# animation function.  This is called sequentially
def animate(i,a,b1,b2,b3):
    #a = list(range(i+1))
    #print(a)
    data = ser.read(12)
    data1 = struct.unpack('<f', data[:4])
    data2 = struct.unpack('<f',  data[4:8])
    data3 = struct.unpack('<f', data[8:12])

    #data = data[:4]
    #print(data)
    #dataf = struct.unpack('<f', data)
    #print(dataf)
    #print(data1)
    #print(data2)
    #print(data3)
    #f = int(data[0])
    #fgh = int(data[1])
    #t = f | (fgh << 8)
    #b.append(t)
    b1.append(data1)
    b2.append(data2)
    b3.append(data3)

    a = np.array(list(range(len(b1))))
    b1 = np.array(b1)
    b2 = np.array(b2)
    b3 = np.array(b3)

    #print("len of a ",len(a))
    #print("len of b ",len(b))
    #print(type(y))
    #print(x)
    #print(y)
    line[0].set_data(a, b1)
    line[1].set_data(a, b2)
    line[2].set_data(a, b3)
    return line

# call the animator.  blit=True means only re-draw the parts that have changed.
anim = animation.FuncAnimation(fig, animate, init_func=init, fargs=(a,b1,b2,b3),
                               frames=2, interval=1, blit=True)

# save the animation as an mp4.  This requires ffmpeg or mencoder to be
# installed.  The extra_args ensure that the x264 codec is used, so that
# the video can be embedded in html5.  You may need to adjust this for
# your system: for more information, see
# http://matplotlib.sourceforge.net/api/animation_api.html
anim.save('basic_animation.mp4', fps=30, extra_args=['-vcodec', 'libx264'])

plt.show()
