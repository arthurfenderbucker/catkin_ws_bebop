import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import sys
import select
import termios
import tty


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == "__main__":

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('teleop_twist_keyboard')
    pub2 = rospy.Publisher('bebop/takeoff', Empty, queue_size=1)
    pub3 = rospy.Publisher('bebop/land', Empty, queue_size=1)
    empty_msg = Empty()

    pub2.publish(empty_msg)

    while(1):
        key = getKey()
        print(key)
        if key in moveBindings.keys():
            x = moveBindings[key][0]
            y = moveBindings[key][1]
            z = moveBindings[key][2]
            th = moveBindings[key][3]
        elif key in speedBindings.keys():
            speed = speed * speedBindings[key][0]
            turn = turn * speedBindings[key][1]

            print vels(speed, turn)
            if (status == 14):
                print msg
            status = (status + 1) % 15

        elif key == '1':
            pub2.publish(empty_msg)
        elif key == '2':
            pub3.publish(empty_msg)

        else:
            x = 0
            y = 0
            z = 0
            th = 0
            if (key == '\x03'):
                break

        twist = Twist()
    twist.linear.x = x * speed
    twist.linear.y = y * speed
    twist.linear.z = z * speed
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = th * turn
    pub.publish(twist)
