import cv2
import numpy as np
import subprocess
from PIL import Image
import Xlib
from Xlib.display import Display, X
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import rclpy
import time as tm
import os, signal
# import time as tm


# Function to get the window ID of a specific window name
def start_screen_publisher():
    return subprocess.Popen(["scrcpy"])

def get_window_id(window_name):
    try:
        # Get the window ID using xdotool
        cmd = ['xdotool', 'search', '--name', window_name]
        result = subprocess.run(cmd, stdout=subprocess.PIPE, check=True)
        window_id = result.stdout.decode().strip()
        print("Window ID:", window_id)
        return window_id
    except subprocess.CalledProcessError:
        return None

def get_absolute_geometry(win,root):
    """
    Returns the (x, y, height, width) of a window relative to the top-left
    of the screen.
    """
    geom = win.get_geometry()
    (x, y) = (geom.x, geom.y)
    while True:
        parent = win.query_tree().parent
        pgeom = parent.get_geometry()
        x += pgeom.x
        y += pgeom.y
        if parent.id == root.id:
            break
        win = parent
    return x, y, geom.width, geom.height

def publish_video(topic_name,window_name):
    rclpy.init()
    node = rclpy.create_node('ultrasound_publisher')
    if(start_screen_publisher() is None):
        print("scrcpy not installed or device not connected")
        return
    else:
        print("scrcpy started")
        tm.sleep(2)
    # Get the window ID
    print(get_window_id(window_name))
    window_id = int(get_window_id(window_name).split()[0])
    display = Display()
    root = display.screen().root

    window = display.create_resource_object('window', window_id)
    # Get the window geometry
    _,_,window_width,window_height = get_absolute_geometry(window,root)
    # Create the publisher
    publisher = node.create_publisher(CompressedImage, topic_name, 10)
    bridge = CvBridge()
    rate = node.create_rate(0.03)
    print("Publishing screen")
    while rclpy.ok():
        # compute the raw image data
        x,y,window_width,window_height = get_absolute_geometry(window,root)

        try:
            raw_image = root.get_image(x, y, window_width,window_height, X.ZPixmap, 0xffffffff)
            if error_:
                error_ = False
                print("\n\nResuming recording")
        except:
            print("ERROR: Margins out of bounds, cannot capture screen")
            error_ = True
            continue
        # create an image from the raw data
        pil_image = Image.frombytes("RGB", (window_width, window_height), raw_image.data, "raw", "BGRX")
        
        #pil_image.show()
        # reopen img in opencv
        image = np.array(pil_image) 
        # downscale image by 50%
        # image = cv2.resize(image, (0,0), fx=0.5, fy=0.5)

        # Convert RGB to BGR
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)


        # Convert OpenCV image to ROS2 message
        # ros_msg = bridge.cv2_to_imgmsg(image, encoding='bgr8')

        msg = CompressedImage()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
        
        # Publish the ROS2 message
        publisher.publish(msg)


    print("Shutting down")
    node.destroy_node()
    rclpy.shutdown()
    os.killpg(os.getpgid(os.getpid()), signal.SIGTERM) 

# Example usage



def main(args=None):
    topic_name = '/screen'
    window_name = 'moto'
    publish_video(topic_name,window_name)
