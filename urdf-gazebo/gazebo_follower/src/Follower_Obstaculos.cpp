#include <ros/ros.h>

#include "Follower.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
#include <string>

using namespace std;

//Parámetros globales
double collisionDistance = 2.5; //Distancia (m) a la que los sensores detectarán obtáculos
string publishedName;
char host[128];
int obstacle_Mode = 0;
float heartbeat_publish_interval = 2;

//Publishers
ros::Publisher obstaclePublish;
ros::Publisher heartbeatPublisher;

//Timers
ros::Timer publish_heartbeat_timer;

//Funciones "Callback"
void sonarHandler(const sensor_msgs::Range::ConstPtr& sensorLeft, const sensor_msgs::Range::ConstPtr& sensorCenter, const sensor_msgs::Range::ConstPtr& sensorRight);
void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);

//Constantes
static const std::string OPENCV_WINDOW = "Image window";

//Clase ImageConverter
ImageConverter::ImageConverter(): 
	left_limit(100),
	right_limit(540),
	imageTransport(nh) {
    		imageSubscriber = imageTransport.subscribe(publishedName + "/camera/image", 1,
            &ImageConverter::imageCallback, this);
    		cmdVelPublisher = nh.advertise <geometry_msgs::Twist> (publishedName +"/cmd_vel", 1);

    //Creación de la ventana en la que se visualizará la imagen
    cv::namedWindow(OPENCV_WINDOW);
}

//Cierre de la ventana de visualización
ImageConverter::~ImageConverter() {
    cv::destroyWindow(OPENCV_WINDOW);
}

//Función para el tratamiento de la imagen y las acciones a realizar si se detectan obstáculos
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

    //Movemos el robot en proporción al error (PD)
    //Las velocidades izquierda y derecha no corresponden con sus signos según la regla de la mano derecha, es decir, la velocidad de giro a la derecha tendría que ser negativa si seguimos la regla pero en nuestro caso será positiva. La velocidad de giro a la izquierda tendría que ser positiva según la regla y en nuestro caso es negativa. Esto se debe a la elección que hemos hecho de los sensores.

        int err = cx - width / 2;
	
	if (obstacle_Mode==1){ //Detecta obstáculo a la derecha
		//if (err < 0){
		//	cmd.angular.z = (float)err / 1000;
		//}
		//else{
		//	cmd.angular.z = -(float)err / 1000;
		//}
		cmd.angular.z = -0.2;
		if (cx > 540){
			cmd.linear.x = 0.2;
		}
		ROS_INFO("Obstaculo a la derecha, giro a la izquierda");
	}
	else if (obstacle_Mode==2){ //Detecta obstáculo a la izquierda
		//if (err < 0){
		//	cmd.angular.z = -(float)err / 1000;
		//}
		//else{
		//	cmd.angular.z = (float)err / 1000;
		//}
		cmd.angular.z = 0.2;
		if (cx < 100){
			cmd.linear.x = 0.2;
		}
		ROS_INFO("Obstaculo a la izquierda, giro a la derecha");
	}
	else if ((obstacle_Mode==3) && (abs(cx) <= left_limit)){ //Obstáculo de frente y camino a la izquierda
		cmd.angular.z = (float)err / 1000;
		ROS_INFO("Obstaculo de frente y sigo por la izquierda");
	}
	else if ((obstacle_Mode==3) && (abs(cx) >= right_limit)){ //Obstáculo de frente y camino a la derecha
		cmd.angular.z = (float)err / 1000;
		ROS_INFO("Obstaculo de frente y sigo por la derecha");
	}
	else if (obstacle_Mode==3){ //Detecta obstáculo de frente
		//if (cx < 320){ //Si el camino está más hacia la izquierda
		//	cmd.angular.z = -0.2;
		//	ROS_INFO("Obstaculo de frente, giro a la izquierda");
		//	cmd.linear.x = 0.1;
		//}
		//else { //Si el camino está más hacia la derecha
		//	cmd.angular.z = 0.2;
		//	ROS_INFO("Obstaculo de frente, giro a la derecha");
		//	cmd.linear.x = 0.1;
		//}
		if (err < 0){ //Si el camino está más hacia la izquierda
			cmd.angular.z = -0.2;
			ROS_INFO("Obstaculo de frente, giro a la izquierda");
			cmd.linear.x = 0.1;
		}
		else { //Si el camino está más hacia la derecha
			cmd.angular.z = 0.2;
			ROS_INFO("Obstaculo de frente, giro a la derecha");
			cmd.linear.x = 0.1;
		}
	}
	//else if ((obstacle_Mode==1) && (abs(cx) >= right_limit)) { // Hay que girar a la derecha para buscar el circuito pero hay obstáculo a 									      la derecha así que giramos a la izquierda.
	//	cmd.angular.z = -(float)err / 1000;
	//	ROS_INFO("Obstaculo y limite derecha, giro a la izquierda");
	//	cmd.linear.x = 0.2;
	//}
	//else if ((obstacle_Mode==2) && (abs(cx) <= left_limit)){ // Hay que girar a la izquierda para buscar el circuito pero hay obstáculo a 									      la izquierda así que giramos a la derecha.
	//	cmd.angular.z = -(float)err / 1000;
	//	ROS_INFO("Obstaculo y limite izquierda, giro a la derecha");
	//	cmd.linear.x = 0.2;
	//}
	
	else if	(abs(cx) <= left_limit) { // Giro a la izquierda siguiendo el camino
		cmd.angular.z = (float)err / 1000;
		ROS_INFO("Buscando camino a la izquierda");
	}
	else if (abs(cx) >= right_limit) { // Giro a la derecha siguiendo el camino
		cmd.angular.z = (float)err / 1000;
		ROS_INFO("Buscando camino a la derecha");
	}	
	else if (obstacle_Mode==0){ //No hay obstáculos
		cmd.linear.x = 0.2;
		ROS_INFO("Continua recto");
	}
        
        cmdVelPublisher.publish(cmd);

    }
    else if (M.m00 == 0){ //El camino se ha quedado fuera del campo de visión
		if (obstacle_Mode==1){ 
			cmd.angular.z = -0.2;
			cout << "¡" << publishedName << " no ve el camino amarillo!" << endl;
			ROS_INFO("Obstaculo a la derecha, giro a la izquierda");
			cmd.linear.x = 0.2;
		}
		else if (obstacle_Mode==2){
			cmd.angular.z = 0.2;
			cout << "¡" << publishedName << " no ve el camino amarillo!" << endl;
			ROS_INFO("Obstaculo a la izquierda, giro a la derecha");
			cmd.linear.x = 0.2;
		}
		else if (obstacle_Mode==3){
			cmd.angular.z = 0.2;
			cout << "¡" << publishedName << " no ve el camino amarillo!" << endl;
			ROS_INFO("Obstaculo de frente, giro a la derecha");
		}
    		else{
			ROS_INFO("Buscando el circuito"); 
			cmd.angular.z = -0.2;
		}
	cmdVelPublisher.publish(cmd);
      }

    //Actualización de la ventana de imagen (blanco y negro)
    cv::imshow(OPENCV_WINDOW, mask);
    //Actualización de la ventana de imagen (muestra el centroide calculado, imagen a color)
    //cv::imshow(OPENCV_WINDOW, image);
    cv::waitKey(3);


    ros::spinOnce(); // ROS comprueba si hay mensajes
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

    ros::init(argc, argv, (publishedName + "_OBSTACLE"));

    ros::NodeHandle oNH;
    
    obstaclePublish = oNH.advertise<std_msgs::UInt8>((publishedName + "/obstacle"), 10);
    heartbeatPublisher = oNH.advertise<std_msgs::String>((publishedName + "/obstacle/heartbeat"), 1, true);
    
    message_filters::Subscriber<sensor_msgs::Range> sensorLeftSubscriber(oNH, (publishedName + "/hc_sr04_1"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sensorCenterSubscriber(oNH, (publishedName + "/hc_sr04_2"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sensorRightSubscriber(oNH, (publishedName + "/hc_sr04_3"), 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSyncPolicy;

    message_filters::Synchronizer<sonarSyncPolicy> sonarSync(sonarSyncPolicy(10), sensorLeftSubscriber, sensorCenterSubscriber, sensorRightSubscriber);
    sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));

    publish_heartbeat_timer = oNH.createTimer(ros::Duration(heartbeat_publish_interval), publishHeartBeatTimerEventHandler);

    ImageConverter _OBSTACLE;

    ros::spin();
    return 0;
}


void sonarHandler(const sensor_msgs::Range::ConstPtr& sensorLeft, const sensor_msgs::Range::ConstPtr& sensorCenter, const sensor_msgs::Range::ConstPtr& sensorRight) {
	std_msgs::UInt8 obstacleMode;
	
//La descripción de los sensores la hemos hecho mirando el robot de frente (hc_sr04_1 = izquierda, hc_sr04_2 =  centro y hc_sr04_3 = derecha). Para la detección de obstáculos hemos considerado el robot visto desde atrás por tanto, hemos cambiado el signo <> en los if.

	if (sensorLeft->range < collisionDistance) {
		obstacleMode.data = 1; //Obtáculo a la derecha
	}
	else if (sensorRight->range < collisionDistance) {
		obstacleMode.data = 2; //Obstáculo a la izquierda
	}
	else if (sensorCenter->range < collisionDistance) 
	{
		obstacleMode.data = 3;//Obstáculo de frente
	}
	else{ //((sensorLeft->range > collisionDistance) && (sensorCenter->range > collisionDistance) 
	      //&& (sensorRight->range > collisionDistance))
		obstacleMode.data = 0; //No hay obstáculos
	}
	
        obstaclePublish.publish(obstacleMode);
        ROS_INFO("Info Obstaclemode: [%d]", obstacleMode.data);
	obstacle_Mode = obstacleMode.data;
}

void publishHeartBeatTimerEventHandler(const ros::TimerEvent&) {
    std_msgs::String msg;
    msg.data = "";
    heartbeatPublisher.publish(msg);
     ROS_INFO("yes");
}
