import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import os
from datetime import datetime

class SyncNode(Node):
    def __init__(self):
        super().__init__('sync_node')
        self.odometry_sub = self.create_subscription(Odometry, '/Odometry/Camera', self.odometry_callback, 10)
        self.image_sub = self.create_subscription(CompressedImage, '/camera/camera/color/image_raw/compressed', self.image_callback, 30)

        self.latest_image = None
        self.latest_images = []
        self.latest_images_fm = []

        self.image_counter = 0
        self.output_dir = datetime.now().strftime('synced_data/synced_%Y%m%d%H%M')
        self.img_output_dir = f'{self.output_dir}/images'
        self.pose_output_dir = f'{self.output_dir}/sparse/0'
        os.makedirs(self.img_output_dir, exist_ok=True)
        os.makedirs(self.pose_output_dir, exist_ok=True)

        self.odometry_log_filename = os.path.join(self.pose_output_dir , 'keyframes.txt')
        with open(self.odometry_log_filename, 'w') as f:
            f.write('# Image list with two lines of data per image:\n')
            f.write('#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n')
            f.write('#   POINTS2D[] as (X, Y, POINT3D_ID)\n')
            f.write('# Number of images: 70, mean observations per image: 10117.249169435216\n')

        with open(os.path.join(self.pose_output_dir, 'cameras.txt'), 'w') as f:
            f.write('# Camera list with one line of data per camera:\n')
            f.write('#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n')
            f.write('# Number of cameras: 1\n')
            f.write('1 PINHOLE 1920 1080 1380.59 1381.47 977.437 540.622\n')

    def odometry_callback(self, msg):
        if self.latest_images is not None:
            # Save image
            image_filename = os.path.join(self.img_output_dir, f'frame_{self.image_counter:05d}.jpg')
            max_idx = np.array(self.latest_images_fm).argmax()
            cv2.imwrite(image_filename, self.latest_images[max_idx])
            self.latest_images = []
            self.latest_images_fm = []
            self.get_logger().info(f'Image saved as {image_filename}')

            # Extract odometry data and append to log file
            orientation = msg.pose.pose.orientation
            position = msg.pose.pose.position
            with open(self.odometry_log_filename, 'a') as f:
                f.write(f'{self.image_counter} {orientation.w} {orientation.x} {orientation.y} {orientation.z} ')
                f.write(f'{position.x} {position.y} {position.z} 1 frame_{self.image_counter:05d}.jpg\n')
                f.write('462.34395265098647 115.1042383155459 -1\n')
            self.get_logger().info(f'Odometry data appended to {self.odometry_log_filename}')
            # Increment counter
            self.image_counter += 1

    def image_callback(self, msg):
        # Convert compressed image to OpenCV format
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.latest_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.latest_images.append(self.latest_image)
        begin = datetime.now()
        fm = cv2.Laplacian(cv2.cvtColor(self.latest_image,cv2.COLOR_BGR2GRAY),cv2.CV_64F).var()
        end = datetime.now()
        self.get_logger().info(f'Elapsed time: {end - begin}')
        self.latest_images_fm.append(fm)
        if len(self.latest_images) > 20:
            self.latest_images.pop(0)
            self.latest_images_fm.pop(0)

def main(args=None):
    rclpy.init(args=args)
    sync_node = SyncNode()
    rclpy.spin(sync_node)
    sync_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()