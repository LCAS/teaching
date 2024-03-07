import zenoh
import time
from io import BytesIO

# Define the zenoh topic where the image data will be published
topic = "vista/image_data"

# Callback function to handle received image data
def callback(data):
    key_expr = data.key_expr
    image_data = data.payload

    # Print that a new image has been received
    print(f"Received a new image")

    # Print image data
    print("Image data:")
    print(image_data)

if __name__ == "__main__":
    session = zenoh.open()
    sub = session.declare_subscriber(topic, callback)
    print("Waiting for image data...")
    time.sleep(60)  # Keep the subscriber alive for 60 seconds
