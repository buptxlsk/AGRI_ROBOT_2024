import rospy
from std_msgs.msg import UInt8MultiArray
import socket

def main():
    rospy.init_node('tcp_client_node')
    rate = rospy.Rate(10)
    host = '10.10.10.101'
    port = 25669
    pub = rospy.Publisher('tcp_data', UInt8MultiArray, queue_size=10)
    while not rospy.is_shutdown():
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((host, port))
            data = client_socket.recv(1024)
            if data:
                raw_data = [i for i in data]
                pub.publish(UInt8MultiArray(data=raw_data))
            client_socket.close()
        except socket.error:
            rospy.logwarn("无法连接到服务端，将重试...")
            rospy.sleep(1)
        rate.sleep()

if __name__ == '__main__':
    main()