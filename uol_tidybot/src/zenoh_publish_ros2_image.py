import rclpy
from sensor_msgs.msg import Image
import zenoh
import cv2
from cv_bridge import CvBridge
import base64

def image_callback(msg, session):
    try:
        # Convert ROS2 image message to OpenCV image
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Convert the OpenCV image to JPEG format
        _, jpeg_image = cv2.imencode('.jpg', cv_image)

        # Encode the JPEG image into base64 string
        image_data_base64 = base64.b64encode(jpeg_image)

        # Publish the image data over Zenoh
        topic = "vista/image_data"
        pub = session.declare_publisher(topic)
        pub.put(image_data_base64)
        #print("Published image on zenoh")
    except Exception as e:
        print(f"Error while processing and publishing image data: {e}")

def main(args=None):
    rclpy.init(args=args)

    # Initialize Zenoh session
    host_ip = "127.0.0.1"  # Change this to the host machine's IP address
    config = {
        "mode": "peer",
        "connect": {
            "endpoints": [f"tcp/{host_ip}:7447"]
        }
    }
    session = zenoh.open(config)

    node = rclpy.create_node('image_subscriber')

    # Subscribe to the ROS2 image topic
    image_subscription = node.create_subscription(
        Image,
        '/limo/depth_camera_link/image_raw',
        lambda msg: image_callback(msg, session),
        10
    )
    image_subscription  # prevent unused variable warning

    print("Subscribed to ROS2 topic '/limo/depth_camera_link/image_raw'")

    rclpy.spin(node)

    # Close Zenoh session
    session.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
