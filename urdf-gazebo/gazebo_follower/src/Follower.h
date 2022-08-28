#include <ros/ros.h>
#include <image_transport/image_transport.h>

class ImageConverter {
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport imageTransport;
    image_transport::Subscriber imageSubscriber;
    ros::Publisher cmdVelPublisher;

    double left_limit, right_limit;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

public:
    ImageConverter();
    virtual ~ImageConverter();
};
