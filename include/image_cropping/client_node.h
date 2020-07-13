#ifndef CLIENT_NODE_H_
#define CLIENT_NODE_H_

#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <actionlib/client/simple_action_client.h>
#include "image_cropping/image_cropAction.h"

typedef actionlib::SimpleActionClient<image_cropping::image_cropAction> Client;
typedef boost::shared_ptr<sensor_msgs::Image const> ImageConstPtr;


class client_node
{
private:
    ros::NodeHandle* nh_;
    Client client_;

public:
    client_node(ros::NodeHandle* nh);
    void display_image(const sensor_msgs::ImageConstPtr& msg);
    void client_cb(int x_pix, int y_pix, int x_width, int y_width);
    void active_cb();
    void feedback_cb(const image_cropping::image_cropFeedbackConstPtr& feedback);
    void done_cb(const actionlib::SimpleClientGoalState& state, const image_cropping::image_cropResultConstPtr& result);
};

#endif