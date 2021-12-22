import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np
import matplotlib.pyplot as plt
import array


class Mapfilter(Node):

    def __init__(self):
        super().__init__('map_filter')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map_filtered', 10)
        timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)

    def listener_callback(self, msg):
        #msg.data = cv2.medianBlur(msg.data, 3)
        #print(msg.data.typecode)
        data = np.asarray(msg.data, dtype=np.uint8)
        data = data.reshape(msg.info.height, msg.info.width)
        new_data = cv2.medianBlur(data, 3)
        new_array = array.array('b')
        new_array.frombytes(new_data.flatten().tobytes())
        msg.data = new_array
        #print("a")
        #plt.imshow(data, cmap='gray', vmin=0, vmax=255)
        #plt.show()
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = Mapfilter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
