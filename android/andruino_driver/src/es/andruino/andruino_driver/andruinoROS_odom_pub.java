package es.andruino.andruino_driver;

import geometry_msgs.TransformStamped;
import nav_msgs.Odometry;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.message.MessageFactory;

import tf2_msgs.TFMessage;
import org.ros.message.MessageFactory;
import android.os.SystemClock;
import java.util.List;

//import sensor_msgs.Range;

public class andruinoROS_odom_pub implements NodeMain {

	String base_frame_id = "base_link";
	String odom_frame_id = "odom";
	/*
	private Publisher<Odometry> odometryPublisher;
	private Publisher<TFMessage> tfPublisher;
	private TransformStamped odomToBaseLink;
	private TransformStamped baseToLaser;
*/
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
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		// return null;
		return GraphName.of("andruino_driver/odom");
	}

	@Override
	public void onStart(final ConnectedNode node) {

		
		//final Publisher<Odometry> publisher = node.newPublisher("andruino/odom", "nav_msgs/Odometry");
		//final Publisher<TransformStamped> broadcaster = node.newPublisher("andruino/tf", "geometry_msgs/TransformStamped");
		final Publisher<Odometry> publisher = node.newPublisher("/odom", "nav_msgs/Odometry");
		//final Publisher<TransformStamped> broadcaster = node.newPublisher("/tf", "geometry_msgs/TransformStamped");
		//final Publisher<TFMessage> broadcaster = node.newPublisher("tf", TFMessage._TYPE); //!
		final Publisher<TransformStamped> broadcaster = node.newPublisher("/tf", TransformStamped._TYPE);
		
	/*
		odometryPublisher = node.newPublisher("/odom", "nav_msgs/Odometry");
		tfPublisher = node.newPublisher("/tf", TFMessage._TYPE);
		//odomToBaseLink = mMessageFactory.newFromType(TransformStamped._TYPE);
		odomToBaseLink= tfPublisher.newMessage();
		*/
		node.executeCancellableLoop(new CancellableLoop() {

			@Override
			protected void setup() {

			}

			protected void loop() throws InterruptedException {
				
				
				
				
				
				long time_delta_millis = System.currentTimeMillis(); // - SystemClock.uptimeMillis();
				// publish the odom information
				TransformStamped transform = broadcaster.newMessage();
				//TransformStamped transform = node.newPublisher("/tf", "geometry_msgs/TransformStamped").newMessage();
				transform.getTransform().getRotation().setX(0);
				transform.getTransform().getRotation().setY(0);
				transform.getTransform().getRotation().setZ(Math.sin(andruino_driver.gAzimut / 2));
				transform.getTransform().getRotation().setW(Math.cos(andruino_driver.gAzimut / 2));
				transform.getTransform().getTranslation().setX(andruino_driver.x);
				transform.getTransform().getTranslation().setY(andruino_driver.y);
				transform.getTransform().getTranslation().setZ(0);
				transform.getHeader().setStamp(Time.fromMillis(time_delta_millis));
				transform.setChildFrameId(odom_frame_id);
				transform.getHeader().setFrameId(base_frame_id);
				//transform.setChildFrameId(base_frame_id);
				//transform.getHeader().setFrameId(odom_frame_id);
				broadcaster.publish(transform);
				
				//TFMessage tfm=broadcaster2.newMessage(); //!
				//List<TransformStamped> tf1 = tfm.getTransforms();
				//tf1.add(transform);
			    //broadcaster2.publish(tfm);
				
				Odometry odom = publisher.newMessage();
				odom.getHeader().setStamp(Time.fromMillis(time_delta_millis)); //setStamp(new Time());
				odom.getHeader().setFrameId(odom_frame_id);
				odom.getPose().getPose().getPosition().setX(andruino_driver.x);
				odom.getPose().getPose().getPosition().setY(andruino_driver.y);
				odom.getPose().getPose().getPosition().setZ(0);
				
				
				odom.getPose().getPose()
						.setOrientation(transform.getTransform().getRotation());
				// Lo que está comentado se saca del sensor Android
				odom.setChildFrameId(base_frame_id);
				odom.getTwist().getTwist().getLinear().setX(andruinoROS_cmd_vel.vel_lin); //CAMBIAR!!! LA VELOCIDD LA COGE DEL COMANDO Y NO DEL ARDUINO. DEBE DARLA EL ARDUINO (CAMBIAR!!!). VERIFICAR!!!!!
				odom.getTwist().getTwist().getLinear().setY(0);
				odom.getTwist().getTwist().getAngular().setZ(andruino_driver.gOmega); //VERIFICaR !!!!
				publisher.publish(odom);

			

			}

		});

	}
}