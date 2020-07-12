#include "image_cropping/server_node.h"

server_node::server_node(ros::NodeHandle* nh, ros::NodeHandle* private_nh, image_transport::ImageTransport* it)
{
    nh_ = nh;
    private_nh_ = private_nh;
    it_ = it;
    // it_ = new image_transport::ImageTransport();
    image_pub = it_->advertise("/image", 1);
    server = nh_->advertiseService("/image_cropping_service", &server_node::image_cropping_server_cb, this);
    sequence = 0;
    initialize_params();
}

void server_node::initialize_params()
{
    int x,y;
    if (!private_nh_->hasParam("x_width"))
    {
        ROS_INFO("No param named 'my_param'");
    }
    private_nh_->getParam("x_width", x);
    private_nh_->getParam("y_width", y);
    x_width = x;
    y_width = y;
    std::cout<<x_width<<std::endl;
    std::cout<<y_width<<std::endl;
}

void server_node::publish_image()
{
    cv_bridge::CvImage temp_CvImage;
    temp_CvImage.header.seq = sequence;
    temp_CvImage.header.stamp = ros::Time::now();
    temp_CvImage.header.frame_id = "base_frame";
    temp_CvImage.encoding = sensor_msgs::image_encodings::BGR8;
    temp_CvImage.image = image;

    image_pub.publish(temp_CvImage.toImageMsg());
}

bool server_node::crop_image(int x_pix, int y_pix)
{
    int final_x_pix, final_y_pix;
    if((image.rows-1) < x_pix + x_width)
        final_x_pix = image.rows;
    else
        final_x_pix = x_pix + x_width;

    if((image.cols-1) < y_pix + y_width)
        final_y_pix = image.cols;
    else
        final_y_pix = y_pix + y_width;

    cv::Mat cropped_image(final_x_pix - x_pix, final_y_pix - y_pix, CV_8UC3);
    for(int i=0; i<final_x_pix - x_pix; i++)
    {
        for(int j=0; j<final_y_pix - y_pix; j++)
        {
            cropped_image.at<cv::Vec3b>(i, j)[0] = image.at<cv::Vec3b>(i+x_pix, j+y_pix)[0];
            cropped_image.at<cv::Vec3b>(i, j)[1] = image.at<cv::Vec3b>(i+x_pix, j+y_pix)[1];
            cropped_image.at<cv::Vec3b>(i, j)[2] = image.at<cv::Vec3b>(i+x_pix, j+y_pix)[2];
        }
    }

    image = cropped_image.clone();
    return true;
}

bool server_node::image_cropping_server_cb(image_cropping::image_crop_srv::Request &req, image_cropping::image_crop_srv::Response &res)
{
    int x_pix, y_pix;
    x_pix = req.x_pix;
    y_pix = req.y_pix;
    res.status = crop_image(x_pix, y_pix);
    return true;
}

void server_node::read_image(char* path)
{
    image = cv::imread(path);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "server_node");
    ros::NodeHandle* nh;
    ros::NodeHandle private_nh("~");

    nh = new ros::NodeHandle();
    // private_nh = new ros::NodeHandle();
    image_transport::ImageTransport it(*nh);

    // it = new image_transport::ImageTransport();

    server_node obj(nh, &private_nh, &it);
    obj.read_image(argv[1]);

    while(ros::ok())
    {
        obj.publish_image();
        ros::spinOnce();
    }
    return 0;
}