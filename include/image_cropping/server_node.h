#ifndef SERVER_NODE_H_
#define SERVER_NODE_H_

#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "image_cropping/image_crop_srv.h"

class server_node
{
private:
    int x_width, y_width, sequence;
    cv::Mat image;
    ros::NodeHandle* nh_;
    ros::NodeHandle* private_nh_;
    ros::ServiceServer server;
    image_transport::ImageTransport* it_;
    image_transport::Publisher image_pub;


public:
    server_node(ros::NodeHandle* nh, ros::NodeHandle* private_nh, image_transport::ImageTransport* it);
    bool image_cropping_server_cb(image_cropping::image_crop_srv::Request &req, image_cropping::image_crop_srv::Response &res);
    bool crop_image(int x_pix, int y_pix);
    void publish_image();
    void read_image(char* path);
    void initialize_params();
};

#endif