#include "ros/ros.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

#include "geometry_msgs/Point.h" //
#include "geometry_msgs/Twist.h" //
#include "geometry_msgs/Pose.h"

#include "sensor_msgs/Image.h"

#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"


float bounding_start = 0;
float bounding_end = 0;

float midpoint_x = 0;
float midpoint_y = 0;

float pixel_count_x1 = 0;
float pixel_count_y1 = 0;
float pixel_count_x2 = 0;
float pixel_count_y2 = 0;
           
float coordinate_z = 0;                 //depth of object (z) [m]
float coordinate_x = 0;                 //coordinate of object (x) [m]
float coordinate_y = 0;                 //coordinate of object (y) [m]

int image_width = 640;
int image_height= 480;
bool read_depth_from_cam_flag = 0;
bool flag_bbox = false;

float PI = 3.141592653589793238;
using namespace cv;

class Coordinate{
private:
    // ros::Subscriber bounding_box_sub;
    // ros::Subscriber realsense_camera_depth_sub;
    // ros::Subscriber calculate_coordinate_flag_sub;
    ros::Subscriber bounding_box_sub;

    // ros::Publisher calculate_coordinate_flag_pub;
    // ros::Publisher bbox_pose_pub;
    // ros::Publisher readCamData_flag_pub;
    ros::Publisher max_hight_pub;

    ros::NodeHandle nh;
    
    // geometry_msgs::Pose bbox_pose;
    // std_msgs::Bool calculate_coordinate_flag_msg;
    // std_msgs::Bool readCamData_Flag;
    std_msgs::Float32 kick_max_hight;

public:
    Coordinate():nh("~"){
        bounding_box_sub = nh.subscribe("/darknet_ros/bounding_boxes", 1000, &Coordinate::boundingboxCallback, this);
        // realsense_camera_depth_sub = nh.subscribe("/camera/depth/image_rect_raw", 1000, &Coordinate::depthCallback, this);
        // calculate_coordinate_flag_sub = nh.subscribe("/calculate_coordinate_flag_msg", 1000, &Coordinate::calculate_coordinate_flag_Callback, this);
        // max_hight_sub = nh.subscribe("/max_hight", 1000, &Coordinate::CalculateHightCallback, this);

        // calculate_coordinate_flag_pub = nh.advertise<std_msgs::Bool>("/calculate_coordinate_flag_msg", 1000);
        // bbox_pose_pub = nh.advertise<geometry_msgs::Pose>("/bbox_pose",1000);
        // readCamData_flag_pub = nh.advertise<std_msgs::Bool>("/readCamData_Flag",1000);
        max_hight_pub = nh.advertise<std_msgs::Float32>("/max_hight",1000);
    }

    void boundingboxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
    {
        read_depth_from_cam_flag = 0;
        float max_hight = 0;
        int max_id = 30000;

        for (int i = 0; i < msg->bounding_boxes.size(); i++){
            if (msg->bounding_boxes[i].id == 0){
                if (max_id == 30000){
                    max_hight = (msg->bounding_boxes[i].ymax - msg->bounding_boxes[i].ymin);
                    max_id = i;
                }else{
                    if((msg->bounding_boxes[i].ymax - msg->bounding_boxes[i].ymin) > max_hight){
                        max_hight = (msg->bounding_boxes[i].ymax - msg->bounding_boxes[i].ymin);
                        max_id = i;
                    }
                }
            }
        }

        if (max_id != 30000){
            
            // pixel_count_x1 = msg->bounding_boxes[min_id].xmin;
            // pixel_count_y1 = msg->bounding_boxes[min_id].ymin;
            // pixel_count_x2 = msg->bounding_boxes[min_id].xmax;
            // pixel_count_y2 = msg->bounding_boxes[min_id].ymax;
            // bounding_start = 640 * pixel_count_y1 + pixel_count_x1;
            // bounding_end = 640 * pixel_count_y2 + pixel_count_x2;
            
            // midpoint_x = (pixel_count_x1 + pixel_count_x2) / 2;
            // midpoint_y = (pixel_count_y1 + pixel_count_y2) / 2;

            kick_max_hight.data = max_hight;
            max_hight_pub.publish(kick_max_hight);
        }else{
            // read_depth_from_cam_flag = 0;
        }
    }

// Horizental x vertical  = 69.4° × 42.5°  (D : 77°)
//     void calculate_coordinate_flag_Callback(const std_msgs::Bool::ConstPtr& msg)
//     {

//         std::cout<<"stop6"<<std::endl;

//         if (msg->data == true){
            
//             std::cout<<"stop7"<<std::endl;
            
//             coordinate_x = (midpoint_x - 320)/320 * (coordinate_z * tan(34.7*PI/180));
//             coordinate_y = (240 - midpoint_y)/240 * (coordinate_z * tan(21.25*PI/180));

//             std::cout<<"midpoint_x: "<<midpoint_x<<std::endl;
//             std::cout<<"midpoint_y: "<<midpoint_y<<std::endl;
//             std::cout<<"coordinate_x: "<<coordinate_x<<std::endl;
//             std::cout<<"coordinate_y: "<<coordinate_y<<std::endl;
//             std::cout<<"coordinate_z: "<<coordinate_z<<std::endl;

//             bbox_pose.position.x = coordinate_x;
//             bbox_pose.position.y = coordinate_y;
//             bbox_pose.position.z = coordinate_z;

//             readCamData_Flag.data = true;

//             bbox_pose_pub.publish(bbox_pose);
//             readCamData_flag_pub.publish(readCamData_Flag);
//             readCamData_Flag.data = false;

//             std::cout<<"stop8"<<std::endl;

//         }

//     }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "coordinate_calculation");
  ros::AsyncSpinner spinner(2);

  spinner.start();
  Coordinate coordinate;
  ros::waitForShutdown();

  return 0;
}