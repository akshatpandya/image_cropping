#include "image_cropping/client_node.h"

#define OPENCV_SUB_WINDOW "Subsciber Image Window"

client_node::client_node(ros::NodeHandle* nh, image_transport::ImageTransport* it)
{
    nh_ = nh;
    it_ = it;
    // it_ = new image_transport::ImageTransport();
    image_sub = it_->subscribe("/image", 1, &client_node::image_sub_cb, this);
    client = nh_->serviceClient<image_cropping::image_crop_srv>("/image_cropping_service");
    cv::namedWindow(OPENCV_SUB_WINDOW);
}

// client_node::~client_node()
//     cv::destroyWindow(OPENCV_SUB_WINDOW);

void client_node::image_sub_cb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
    if (sensor_msgs::image_encodings::isColor(msg->encoding))
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    else
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    
    cv::imshow(OPENCV_SUB_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void client_node::create_crop_service_req(int x_pix, int y_pix)
{
    image_cropping::image_crop_srv temp_srv;
    temp_srv.request.x_pix = x_pix;
    temp_srv.request.y_pix = y_pix;
    if (client.call(temp_srv))
        ROS_INFO("response: %d", temp_srv.response.status);
    else
        ROS_ERROR("Failed to call service");
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "client_node");
    ros::NodeHandle* nh;
    nh = new ros::NodeHandle();
    image_transport::ImageTransport it(*nh);
    int x_pix, y_pix;
    

    std::cout<<"here\n";
    client_node obj(nh, &it);
    x_pix = std::stoi(argv[1]);
    y_pix = std::stoi(argv[2]);

    std::cout<<x_pix<<" "<<y_pix;
    ros::Time init_time =  ros::Time::now();
    ros::Duration d(1);
    while(ros::Time::now() - init_time <= d)
        ros::spinOnce();
    
    obj.create_crop_service_req(x_pix, y_pix);

    while(ros::ok())
        ros::spinOnce();
}