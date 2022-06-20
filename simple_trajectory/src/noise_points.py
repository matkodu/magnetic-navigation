#!/usr/bin/env python
# license removed for brevity

import math 
import rospy
import numpy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64

############### line parameters ###################

# point 1
x_line_1 = -6
y_line_1 = 8
z_line_1 = 1.5

# point 2
x_line_2 = 3
y_line_2 = 2
z_line_2 = 1.5

############## noise parameters ##################

# mean
mean = 0

# standard deviation
std = 1 

# number of elements in array noise
num_samples = 1000

# array noise
noise = numpy.random.normal(mean, std, size=num_samples)

for i in range(1000):
    noise[i] = (noise[i] / 3) * 0.25
##################### Plot ##########################
# Plot noise
#plt.title('White Noise Plot', size=20)
#plt.plot(numpy.arange(len(noise)), noise);
#plt.show()

def get_noise(i):

    noise_point = float(noise[i])

    return(noise_point)



############## calculating first line point with nosie #######
def get_line_point_1(i):

    line_point_1 = Vector3()

    line_point_1.x = float(x_line_1 + noise[i])
    line_point_1.y = float(y_line_1 + noise[i])
    line_point_1.z = float(z_line_1 + noise[i])

    return(line_point_1)   


############## calculating second line point with nosie ######
def get_line_point_2(i):

    line_point_2 = Vector3()

    line_point_2.x = float(x_line_2 + noise[i])
    line_point_2.y = float(y_line_2 + noise[i])
    line_point_2.z = float(z_line_2 + noise[i])

    return(line_point_2)


def talker():

    pub_line_1 = rospy.Publisher('line_point_1', Vector3, queue_size=10)
    pub_line_2 = rospy.Publisher('line_point_2', Vector3, queue_size=10)
    pub_noise = rospy.Publisher('noise', Float64, queue_size=10)
    rospy.init_node('point_noise_publisher', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    i = 0
    while not rospy.is_shutdown():

        line_point_1 = get_line_point_1(i)
        pub_line_1.publish(line_point_1)

        line_point_2 = get_line_point_2(i)
        pub_line_2.publish(line_point_2)

        noise_point = get_noise(i)
        pub_noise.publish(noise_point)

        i = i + 1

        rate.sleep()
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

