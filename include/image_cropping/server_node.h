#ifndef SERVER_NODE_H_
#define SERVER_NODE_H_

#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <actionlib/server/simple_action_server.h>
#include"image_cropping/image_cropAction.h"

typedef actionlib::SimpleActionServer<image_cropping::image_cropAction> Server;

class server_node
{
private:
    int x_width, y_width, x_pix, y_pix, sequence;
    cv::Mat image;
    ros::NodeHandle* nh_;
    actionlib::SimpleActionServer<image_cropping::image_cropAction> server_;
    image_cropping::image_cropFeedback feedback_;
    image_cropping::image_cropResult result_;

public:
    server_node(ros::NodeHandle* nh, std::string server_name);
    void crop_image(cv::Mat* cropped_image, int x1, int x2, int y1, int y2);
    void publish_image(cv::Mat* img, bool in_process);
    void read_image(char* path);
    void initialize_params(const image_cropping::image_cropGoalConstPtr& goal);
    void execute_cb(const image_cropping::image_cropGoalConstPtr& goal);
    void update_crop_width();
};

#endif