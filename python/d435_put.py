import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import cv2
#from rclpy.qos import QoSProfile

class D435_Node(Node):
    def __init__(self):
        super().__init__("D435_put_Node")

        #qos = QoSProfile(depth=5)

        self.bgr_pub=self.create_publisher(Image,"/d435/color/image_raw",10)
        self.dep_pub=self.create_publisher(Image,"/d435/depth/image_raw",10)

        #self.d435_info=self.create_publisher(CameraInfo,"/d435/info",10)

        self.bridge=CvBridge()

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in self.device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile=self.pipeline.start(self.config)

        #self.intr=self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.align = rs.align(rs.stream.color)          # 深度,彩色对齐

        self.timer=self.create_timer(1/30.0,self.callback)
    """
    def d435_init(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in self.device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile=self.pipeline.start(self.config)

        #self.intr=self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.align = rs.align(rs.stream.color)          # 深度,彩色对齐

        self.timer=self.create_timer(1/30.0,self.callback)
    """

    def callback(self):
        self.frame=self.pipeline.wait_for_frames()
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        aligns=self.align.process(frames)
        depth_frame = aligns.get_depth_frame()
        color_frame = aligns.get_color_frame()
        #读内参
        #self.intrin = color_frame.profile.as_video_stream_profile().intrinsics
        #self.fx,self.fy=self.intrin.fx,self.intrin.fy
        #self.cx,self.cy=self.intrin.ppx,self.intrin.ppy
        if not depth_frame or not color_frame:
            return 

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        bgr_msg=self.bridge.cv2_to_imgmsg(color_image,encoding="bgr8")
        bgr_msg.header.stamp=self.get_clock().now().to_msg()
        bgr_msg.header.frame_id="d435_world"

        depth_msg=self.bridge.cv2_to_imgmsg(depth_image,encoding="16UC1")
        depth_msg.header=bgr_msg.header
        """
        Info= CameraInfo()
        Info.header=bgr_msg.header
        Info.width=640
        Info.height=480
        Info.k = [self.fx,0.0,self.cx,
                0.0,self.fy,self.cy,
                0.0,0.0,1.0]

        Info.p = [self.fx,0.0,self.cx,0.0,
                0.0,self.fy,self.cy,0.0,
                0.0,0.0,1.0,0.0]

        """

        self.bgr_pub.publish(bgr_msg)
        self.dep_pub.publish(depth_msg)
        #self.d435_info.publish(Info)
        #print(f"数据已发布")

def main():
    rclpy.init()
    node=D435_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =="__main__":
    main()

