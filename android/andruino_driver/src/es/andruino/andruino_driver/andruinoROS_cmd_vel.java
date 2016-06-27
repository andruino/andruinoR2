package es.andruino.andruino_driver;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;




public class andruinoROS_cmd_vel implements NodeMain {

	public static float vel_lin;
	public static float vel_ang;
	float vel_ang_R;
	float vel_ang_L;
	
	// distancia_rueba y radio_rueda deberían ser parámetros de ROS !!!!!!!!!!!!!!!!!!!!!!!!!
	// Para el "Six-leg walking type de CIC kit"
	final double distancia_rueda= 0.1490;
	final double radio_rueda= 0.01785;
	
	int PWM_R=0;
	int PWM_L=0;
	

	@Override
	public void onError(Node arg0, Throwable arg1) {
		// TODO Auto-generated method stub

	}

	@Override
	public void onShutdown(Node arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub

	}


	@Override
	public void onStart(ConnectedNode connectedNode) {
		
		Subscriber<geometry_msgs.Twist> subscriber = connectedNode.newSubscriber("andruino/cmd_vel", geometry_msgs.Twist._TYPE);
		subscriber.addMessageListener(new MessageListener<geometry_msgs.Twist>() {
			@Override
			public void onNewMessage(geometry_msgs.Twist message) {
				//log.info("I heard: \"" + message.getData() + "\"");
				
				vel_lin=(float)message.getLinear().getX();
				vel_ang=(float)message.getAngular().getZ();
				
				//Modelo en Arduino
				// Mando la orden al arduino
				String wbuf = "iiikkk"+Float.toString(vel_lin)+"ww"+Float.toString(vel_ang) + "ww###";
				andruino_driver.mSerial.write(wbuf.getBytes());

				/*//Modelo en Android
				  
				
				vel_ang_R=(vel_lin+(vel_ang*distancia_rueda/2))/radio_rueda;
				vel_ang_L=(vel_lin-(vel_ang*distancia_rueda/2))/radio_rueda;
				
				//radio_rueda y distancia_rueda deben ser ajustadas en el proceso de calibracion !!!!!
				
				//Aplico modelo lineal obtendo entre PWM y velocidad angular en calibracion test 4
				// Resultados en hoja de cálculo
				PWM_R=(int)(101.49*vel_ang_R+103.41);
				PWM_L=(int) (111.47*vel_ang_L+132.47);
				
				// Limito a valores posibles
				if (PWM_R > 255){
					PWM_R=255;}
				else if (PWM_R<155){
					PWM_R=0;
				}
				
				// Limito a valores posibles
				if (PWM_L > 255){
					PWM_L=255;}
				else if (PWM_L<155){
					PWM_L=0;
				}
				
				
				// Mando la orden al arduino
				String wbuf = "iiimmm"+Integer.toString(PWM_R)+"ww"+Integer.toString(PWM_L)+"ww###";
				andruino_driver.mSerial.write(wbuf.getBytes());
				
				// Fin de modelo en Android
				*/
				
			}
		}); // ,50); //AQUI
	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("andruino/cmd_vel");
	}

}