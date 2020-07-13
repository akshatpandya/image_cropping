#include "image_cropping/client_node.h"

#define OPENCV_SUB_WINDOW "Subsciber Image Window"

client_node::client_node(ros::NodeHandle* nh) : nh_(nh), client_("image_crop_action", true)
{
    ROS_INFO("Waiting for action server to start.");
    client_.waitForServer();
    ROS_INFO("Action server started, sending goal.");
    cv::namedWindow(OPENCV_SUB_WINDOW);
}


void client_node::display_image(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        if (sensor_msgs::image_encodings::isColor(msg->encoding))
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        else
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        
        cv::imshow(OPENCV_SUB_WINDOW, cv_ptr->image);
        cv::waitKey(1000);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void client_node::active_cb()
{
    ROS_INFO("Client request is active now\n");
}

void client_node::feedback_cb(const image_cropping::image_cropFeedbackConstPtr& feedback)
{
    ROS_INFO("Feedback\n");
    sensor_msgs::ImageConstPtr rosimg = boost::make_shared<sensor_msgs::Image const>(feedback->image);
    display_image(rosimg);
}

void client_node::done_cb(const actionlib::SimpleClientGoalState& state, const image_cropping::image_cropResultConstPtr& result)
{
    sensor_msgs::ImageConstPtr rosimg = boost::make_shared<sensor_msgs::Image const>(result->final_image);
    display_image(rosimg);
}

void client_node::client_cb(int x_pix, int y_pix, int x_width, int y_width)
{
    image_cropping::image_cropGoal goal;
    goal.x_pix = x_pix;
    goal.y_pix = y_pix;
    goal.x_width = x_width;
    goal.y_width = y_width;

    client_.sendGoal(goal, boost::bind(&client_node::done_cb, this, _1, _2), boost::bind(&client_node::active_cb, this), boost::bind(&client_node::feedback_cb, this, _1));
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "client_node");
    ros::NodeHandle* nh;
    nh = new ros::NodeHandle();
    int x_pix, y_pix, x_width, y_width;
    
    client_node obj(nh);
    x_pix = std::stoi(argv[1]);
    y_pix = std::stoi(argv[2]);
    x_width = std::stoi(argv[3]);
    y_width = std::stoi(argv[4]);
    
    obj.client_cb(x_pix, y_pix, x_width, y_width);
    ros::spin();
}