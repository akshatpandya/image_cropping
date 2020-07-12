#ifndef CLIENT_NODE_H_
#define CLIENT_NODE_H_

#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "image_cropping/image_crop_srv.h"

class client_node
{
private:
    ros::NodeHandle* nh_;
    ros::ServiceClient client;
    image_transport::ImageTransport* it_;
    image_transport::Subscriber image_sub;


public:
    client_node(ros::NodeHandle* nh, image_transport::ImageTransport* it);
    // ~client_node();
    void image_sub_cb(const sensor_msgs::ImageConstPtr& msg);
    void create_crop_service_req(int x_pix, int y_pix);
};

#endif