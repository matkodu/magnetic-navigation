import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []

def callback(msg):
    callback = Vector3()
    callback.x = msg.x
    callback.y = msg.y
    callback.z = msg.z

# This function is called periodically from FuncAnimation
def animate(i, xs, ys):

    data = Vector3()
    data.x = callback.x
    data.y = callback.y
    data.z = callback.z

    # Add x and y to lists
    xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
    ys.append(data.x)

    # Limit x and y lists to 20 items
    xs = xs[-20:]
    ys = ys[-20:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('Powerline point in time')
    plt.ylabel('Powerline Point')



def talker():
    rospy.Subscriber('/line_point_1', Vector3, callback, queue_size=20)
    # Set up plot to call animate() function periodically
    ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=1000)
    plt.show()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
