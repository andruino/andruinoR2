#include <ros/ros.h>

#include "Follower.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>
#include <string>


using namespace std;

string publishedName;
char host[128];

static const std::string OPENCV_WINDOW = "Image window";

//Clase ImageConverter
ImageConverter::ImageConverter(): 
	left_limit(150),
	right_limit(490),
	imageTransport(nh) {
    		imageSubscriber = imageTransport.subscribe(publishedName + "/camera/image", 1,
            &ImageConverter::imageCallback, this);
    		cmdVelPublisher = nh.advertise <geometry_msgs::Twist> (publishedName + "/cmd_vel", 1);

    //Creación de la ventana en la que se visualizará la imagen
    cv::namedWindow(OPENCV_WINDOW);
}

//Cierre de la ventana de visualización
ImageConverter::~ImageConverter() {
    cv::destroyWindow(OPENCV_WINDOW);
}

//Función para el tratamiento de la imagen
void ImageConverter::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    //Convierte el mensaje imagen de ROS a CvImage
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //Convierte la imagen de entrada a HSV (Hue Saturation Value)
    cv::Mat image = cv_ptr->image;
    cv::Mat hsvImage;
    cv:cvtColor(image, hsvImage, CV_BGR2HSV);

    //Tratamiento de la imagen HSV para quedarnos sólo con los píxeles amarillos
    cv::Mat mask;
    cv::Scalar lower_yellow(20, 100, 100);
    cv::Scalar upper_yellow(30, 255, 255);
    cv::inRange(hsvImage, lower_yellow, upper_yellow, mask);

    int width = mask.cols;
    int height = mask.rows;
    ROS_INFO_THROTTLE(1, "Width=%i", width);
    ROS_INFO_THROTTLE(1, "Height=%i", height);
    int search_top = 3 * height / 4;
    int search_bottom = search_top + 20;

    //Damos valor 0 a los píxeles que quedan fuera de la región deseada
    for (int y = 0; y < height - 2; y++) {
        if (y < search_top || y > search_bottom) {
            for (int x = 0; x < width; x++) {
                mask.at<cv::Vec3b>(y, x)[0] = 0;
                mask.at<cv::Vec3b>(y, x)[1] = 0;
                mask.at<cv::Vec3b>(y, x)[2] = 0;
            }
        }
    }

    //Calculamos el centroide de la región deseada (imagen binaria) usando la función momento.
    cv::Moments M = cv::moments(mask);
    geometry_msgs::Twist cmd;
 
    if (M.m00 > 0) {
        int cx = int(M.m10 / M.m00);
        ROS_INFO_THROTTLE(1, "Cx=%i", cx);
        int cy = int(M.m01 / M.m00);
	ROS_INFO_THROTTLE(1, "Cy=%i", cy);
        cv::circle(image, cv::Point(cx, cy), 20, CV_RGB(255, 0, 0), -1);

    //Movemos el robot en proporción al error (Controlador P)
        int err = cx - width / 2;
	
	if (abs(cx) <= left_limit) { //Giro a la izquierda (el signo debería ser positivo, según la 					       regla de la mano derecha, pero será negativo debido a la elección 					       que hemos hecho de los sensores).
		cmd.angular.z = (float)err / 1000;
		ROS_INFO("Buscando camino a la izquierda");
	}
	else if (abs(cx) >= right_limit) { //Giro a la derecha (el signo debería ser negativo, según la 					     regla de la mano derecha, pero será positivo debido a la 						     elección que hemos hecho de los sensores).
		cmd.angular.z = (float)err / 1000;
		ROS_INFO("Buscando camino a la derecha");
	}
	else {
		cmd.linear.x = 0.2;
	}
        
        cmdVelPublisher.publish(cmd);
    }
    else if (M.m00 == 0){
	cout << "¡" << publishedName << " no ve el camino amarillo!" << endl;
	//ROS_INFO("No ve camino");
	cmd.angular.z = -0.2;

	cmdVelPublisher.publish(cmd);
    }

    //Actualización de la ventana de imagen (blanco y negro)
    //cv::imshow(OPENCV_WINDOW, mask);
    //Actualización de la ventana de imagen (muestra el centroide calculado, imagen a color)
    cv::imshow(OPENCV_WINDOW, image);
    cv::waitKey(3);
}

int main(int argc, char **argv) {

    gethostname(host, sizeof (host));
    string hostname(host);
    
    if (argc >= 2) {
        publishedName = argv[1];
        cout << "¡Robot " << publishedName << " cargado correctamente!" << endl;
    } else {
        publishedName = hostname;
        cout << "No name selected. Default is: " << publishedName << endl;
    }

    ros::init(argc, argv, (publishedName + "_FOLLOWER"));
    ImageConverter _FOLLOWER;
    ros::spin();
    return 0;
}
