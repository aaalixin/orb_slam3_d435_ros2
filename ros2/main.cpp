#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <System.h>

using std::placeholders::_1;
using std::placeholders::_2;

class ORB_SLAM3_Node : public rclcpp::Node
{
    public:
        ORB_SLAM3_Node(const std::string& vocab_file, const std::string& config_file): Node("d435_slam_node")
        {
            // 初始化ORB_SLAM3
            SLAM_ = std::make_shared<ORB_SLAM3::System>(vocab_file, config_file, ORB_SLAM3::System::RGBD, true);
    
            // 订阅彩色图和深度图
            rgb_sub_.subscribe(this, "/d435/color/image_raw");
            depth_sub_.subscribe(this, "/d435/depth/image_raw");
                
            sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), rgb_sub_, depth_sub_);sync_->registerCallback(&ORB_SLAM3_Node::callback, this);
            RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 ROS2 Node initialized - Waiting for RGB-D images...");
        }
    
        void initialize()
        {
            SLAM_->SetROSNode(shared_from_this());
            
            RCLCPP_INFO(this->get_logger(), "ROS publishers initialized!");
            RCLCPP_INFO(this->get_logger(), "Will publish topics:");
            RCLCPP_INFO(this->get_logger(), "  - /d435/camera_pose");
            RCLCPP_INFO(this->get_logger(), "  - /d435/trajectory");
            RCLCPP_INFO(this->get_logger(), "  - /d435/map_points");
            RCLCPP_INFO(this->get_logger(), "  - /d435/current_points");
        }
    
        ~ORB_SLAM3_Node()
        {
            if (SLAM_) 
            {
                SLAM_->Shutdown();
            }
        }
    
    private:
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,sensor_msgs::msg::Image> MySyncPolicy;
    
        void callback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
        {
            cv::Mat imRGB, imDepth;
            try
            {
                imRGB = cv_bridge::toCvCopy(rgb_msg, "bgr8")->image;
                imDepth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
                
                RCLCPP_INFO(this->get_logger(), "Received images - RGB: %dx%d, Depth: %dx%d", 
                           imRGB.cols, imRGB.rows, imDepth.cols, imDepth.rows);
            }
            catch(cv_bridge::Exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
    
            double timestamp = rgb_msg->header.stamp.sec + rgb_msg->header.stamp.nanosec * 1e-9;
    
            //从yaml获取内参
            SLAM_->TrackRGBD(imRGB, imDepth, timestamp);
            
            RCLCPP_DEBUG(this->get_logger(), "Processing frame at time: %f", timestamp);
        }

    std::shared_ptr<ORB_SLAM3::System> SLAM_;

    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;

    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
};

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        std::cerr << "ros2 run d435_slam_ros2 d435_slam_node Vocabulary/ORBvoc.txt config/d435.yaml" << std::endl;
        return -1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<ORB_SLAM3_Node>(argv[1], argv[2]);
    
    // 在节点完全构造后调用初始化
    node->initialize();
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
