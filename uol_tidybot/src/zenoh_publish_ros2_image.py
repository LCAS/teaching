import rclpy
from sensor_msgs.msg import Image
import zenoh
import base64

def image_callback(msg, session):
    # Extract image data from ROS2 message
    image_data = msg.data

    # Convert image data to base64 string
    image_data_base64 = base64.b64encode(image_data).decode('utf-8')

    # Publish the image data over Zenoh
    topic = "image_data"
    try:
        pub = session.declare_publisher(topic)
        pub.put(image_data_base64.encode('utf-8'))
        #print("Published image on zenoh")
    except Exception as e:
        print(f"Error while publishing image data: {e}")

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
