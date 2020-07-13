#include "image_cropping/server_node.h"

server_node::server_node(ros::NodeHandle* nh, std::string server_name) : nh_(nh), server_(*nh_, server_name, boost::bind(&server_node::execute_cb, this, _1), false)     
{
    sequence = 0;
    server_.start();
}

void server_node::execute_cb(const image_cropping::image_cropGoalConstPtr& goal)
{
    ROS_INFO("Received action client request.\n");

    //get the params from image_cropGoal message
    initialize_params(goal);

    //start cropping the image
    update_crop_width();

    //crop 50 pixels from each side
    int x1, y1, x2, y2;
    int x1_final, y1_final, x2_final, y2_final;
    x1_final = x_pix;
    x2_final = x_pix + x_width - 1;
    y1_final = y_pix;
    y2_final = y_pix + y_width - 1;

    x1 = y1 = 0;
    x2 = image.cols - 1;
    y2 = image.rows - 1;

    std::cout<<"Rows = "<<image.rows<<" Cols = "<<image.cols;

    cv::Mat final_image;
    int count = 0;
    while(!(x1==x1_final && y1==y1_final && x2==x2_final && y2==y2_final))
    {
        ROS_INFO("Processing request\n");
        if(x1_final - x1 + 1 > 50)
            x1 += 50;
        else
            x1 = x1_final; 
        
        if(x2 - x2_final + 1 > 50)
            x2 -= 50;
        else
            x2 = x2_final; 

        if(y1_final - y1 + 1 > 50)
            y1 += 50;
        else
            y1 = y1_final; 

        if(y2 - y2_final + 1 > 50)
            y2 -= 50;
        else
            y2 = y2_final; 

        count += 1;
        std::cout<<x1<<" "<<x2<<" "<<y1<<" "<<y2<<"\n";
        cv::Mat cropped_image(y2-y1+1, x2-x1+1, CV_8UC3);
        crop_image(&cropped_image, x1, x2, y1, y2);

        publish_image(&cropped_image, true);
        final_image = cropped_image.clone();
        ros::Duration(3).sleep();
    }
    ROS_INFO("%d\n", count);
    ROS_INFO("Completed\n");
    publish_image(&final_image, false);
}

void server_node::initialize_params(const image_cropping::image_cropGoalConstPtr& goal)
{
    x_pix = goal->x_pix;
    y_pix = goal->y_pix;
    x_width = goal->x_width;
    y_width = goal->y_width;
}

void server_node::update_crop_width()
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

    x_width = final_x_pix - x_pix;
    y_width = final_y_pix - y_pix;
}

void server_node::publish_image(cv::Mat* img, bool in_process)
{
    cv_bridge::CvImage temp_CvImage;
    temp_CvImage.header.seq = sequence;
    temp_CvImage.header.stamp = ros::Time::now();
    temp_CvImage.header.frame_id = "base_frame";
    temp_CvImage.encoding = sensor_msgs::image_encodings::BGR8;
    temp_CvImage.image = *img;

    if(in_process == true)
    {
        feedback_.image = *temp_CvImage.toImageMsg();
        server_.publishFeedback(feedback_);
    }
    else
    {
        result_.final_image = *temp_CvImage.toImageMsg();
        server_.setSucceeded(result_);
    }
    sequence += 1;
}

void server_node::crop_image(cv::Mat* cropped_image, int x1, int x2, int y1, int y2)
{
    for(int i=0; i<y2-y1+1; i++)
    {
        for(int j=0; j<x2-x1+1; j++)
        {
            cropped_image->at<cv::Vec3b>(i, j)[0] = image.at<cv::Vec3b>(i+x1, j+y1)[0];
            cropped_image->at<cv::Vec3b>(i, j)[1] = image.at<cv::Vec3b>(i+x1, j+y1)[1];
            cropped_image->at<cv::Vec3b>(i, j)[2] = image.at<cv::Vec3b>(i+x1, j+y1)[2];
        }
    }
    std::cout<<"Cropping\n";

}

void server_node::read_image(char* path)
{
    image = cv::imread(path);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "server_node");
    ros::NodeHandle* nh;
    nh = new ros::NodeHandle();

    server_node obj(nh, std::string("image_crop_action"));
    obj.read_image(argv[1]);

    ros::spin();
    return 0;
}