#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
    
    int main(int argc, char** argv) {
        ros::init(argc, argv, "state_publisher");
        ros::NodeHandle n;
        ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
       tf::TransformBroadcaster broadcaster;
       ros::Rate loop_rate(30);
   
       const double degree = M_PI/180;
   
       // robot state
      // double inc=0.005, cen_izq_inc = 0.005, cen_drch_inc = 0.005, izq_arrib_inc = 0.005, izq_abaj_inc = 0.005, drch_arrib_inc = 0.005, drch_abaj_inc = 0.005; 
       //double angle = 0, cen_izq = 0, cen_drch = 0, izq_arrib = 0, izq_abaj = 0, drch_arrib = 0, drch_abaj = 0; 

	double inc=0.005, izqda_inc = 0.005, drcha_inc = 0.005, centro_inc = 0.005; 
       double angle = 0, centro = 0, drcha = 0, izqda = 0; 
   
       // message declarations
       geometry_msgs::TransformStamped odom_trans;
       sensor_msgs::JointState joint_state;
       odom_trans.header.frame_id = "odom";	
       odom_trans.child_frame_id = "base";
   
       while (ros::ok()) {
           //update joint_state
           joint_state.header.stamp = ros::Time::now();
           joint_state.name.resize(36);
           joint_state.position.resize(36);
	   joint_state.name[0] ="pata_centro_izqda";
           joint_state.position[0] = 0;
           joint_state.name[1] ="pata_centro_drcha";
           joint_state.position[1] = 0;
 	   joint_state.name[2] ="pata_centro_izqda_tras";
           joint_state.position[2] = 0;
	   joint_state.name[3] ="pata_centro_drcha_tras";
           joint_state.position[3] = 0;
	   joint_state.name[4] ="izqda_arriba";
           joint_state.position[4] = 0;
	   joint_state.name[5] ="larga_izqda_arriba";
           joint_state.position[5] = 0;
	   joint_state.name[6] ="izqda_abajo";
           joint_state.position[6] = 0;
	   joint_state.name[7] ="larga_izqda_abajo";
           joint_state.position[7] = 0;
	   joint_state.name[8] ="drcha_arriba";
           joint_state.position[8] = 0;
	   joint_state.name[9] ="larga_drcha_arriba";
           joint_state.position[9] = 0;
	   joint_state.name[10] ="drcha_abajo";
           joint_state.position[10] = 0;
	   joint_state.name[11] ="larga_drcha_abajo";
           joint_state.position[11] = 0;
	   joint_state.name[12] ="eje_pata_centro_izqda";
           joint_state.position[12] = 0;
	   joint_state.name[13] ="rueda_eje_centro_izqda";
           joint_state.position[13] = centro;
	   joint_state.name[14] ="eje_pata_centro_drcha";
           joint_state.position[14] = 0;
	   joint_state.name[15] ="rueda_eje_centro_drcha";
           joint_state.position[15] = centro;
	   joint_state.name[16] ="eje_pata_izqda_arriba";
           joint_state.position[16] = 0;
	   joint_state.name[17] ="rueda_eje_izqda_arriba";
           joint_state.position[17] = izqda;
	   joint_state.name[18] ="eje_pata_izqda_abajo";
           joint_state.position[18] = 0;
	   joint_state.name[19] ="rueda_eje_izqda_abajo";
           joint_state.position[19] = izqda;
	   joint_state.name[20] ="eje_pata_drcha_arriba";
           joint_state.position[20] = 0;
	   joint_state.name[21] ="rueda_eje_drcha_arriba";
           joint_state.position[21] = drcha;
	   joint_state.name[22] ="eje_pata_drcha_abajo";
           joint_state.position[22] = 0;
	   joint_state.name[23] ="rueda_eje_drcha_abajo";
           joint_state.position[23] = drcha;
	   joint_state.name[24] ="plataforma_base";
           joint_state.position[24] = 0;
	   joint_state.name[25] ="placa_arduino_plataforma";
           joint_state.position[25] = 0;
	   joint_state.name[26] ="control_motores_arduino";
           joint_state.position[26] = 0;
	   joint_state.name[27] ="engranaje_eje_izqda_arriba";
           joint_state.position[27] = 0;
	   joint_state.name[28] ="engranaje_eje_izqda_abajo";
           joint_state.position[28] = 0;
	   joint_state.name[29] ="engranaje_eje_drcha_arriba";
           joint_state.position[29] = 0;
	   joint_state.name[30] ="engranaje_eje_drcha_abajo";
           joint_state.position[30] = 0;
	   joint_state.name[31] ="sensor_front";
           joint_state.position[31] = 0;
	   joint_state.name[32] ="sensor_drcha";
           joint_state.position[32] = 0;
	   joint_state.name[33] ="sensor_izqda";
           joint_state.position[33] = 0;
	   joint_state.name[34] ="movil_plataforma";
           joint_state.position[34] = 0;
	   joint_state.name[35] ="camera_movil";
           joint_state.position[35] = 0;
	   
   
   
           // update transform
           // (moving in a circle with radius=2)
           odom_trans.header.stamp = ros::Time::now();
           odom_trans.transform.translation.x = cos(angle);
           odom_trans.transform.translation.y = sin(angle);
           odom_trans.transform.translation.z = 0.0;
           odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle);
   
           //send the joint state and transform
           joint_pub.publish(joint_state);
           broadcaster.sendTransform(odom_trans);
   
           // Create new robot state
           izqda += izqda_inc;
           if (izqda<-1.5 || izqda>1.5) izqda_inc *= -1;
           drcha += drcha_inc;
           if (drcha>1 || drcha<-1.0) drcha_inc *= -1;
	   centro += centro_inc;
           if (centro<0 || centro>1) centro_inc *= -1;
           
           angle += degree/4;
   
           // This will adjust as needed per iteration
           loop_rate.sleep();
       }
   
   
       return 0;
}
