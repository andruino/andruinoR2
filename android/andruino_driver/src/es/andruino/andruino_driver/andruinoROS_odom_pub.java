package es.andruino.andruino_driver;

import geometry_msgs.TransformStamped;

import java.util.List;

import nav_msgs.Odometry;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import tf.tfMessage;
//import tf2_msgs.TFMessage;
//import tf2_msgs.TFMessage; //tf2

public class andruinoROS_odom_pub implements NodeMain {

	String base_frame_id = "base_link";
	String odom_frame_id = "odom";

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

		final Publisher<Odometry> publisher = node.newPublisher("odom",
				"nav_msgs/Odometry");
		final Publisher<tfMessage> publisher2 = node.newPublisher("/tf",
				tfMessage._TYPE);
		final MessageFactory mMessageFactory = node.getTopicMessageFactory();

		node.executeCancellableLoop(new CancellableLoop() {

			@Override
			protected void setup() {

			}

			protected void loop() throws InterruptedException {

				geometry_msgs.Quaternion cuaternio;

				long time_delta_millis = System.currentTimeMillis();

				// ODOM
				Odometry odom = publisher.newMessage();
				odom.getHeader().setStamp(Time.fromMillis(time_delta_millis));
				odom.getHeader().setFrameId("odom");

				odom.setChildFrameId("base_link");

				odom.getPose().getPose().getPosition()
						.setX((double) andruino_driver.gx);
				odom.getPose().getPose().getPosition()
						.setY((double) andruino_driver.gy);
				odom.getPose().getPose().getPosition().setZ(0);

				cuaternio = node.getTopicMessageFactory().newFromType(
						geometry_msgs.Quaternion._TYPE);
				cuaternio.setX(0.0);
				cuaternio.setY(0.0);
				cuaternio.setZ(Math.sin(andruino_driver.gAzimut_odom / 2.0));
				cuaternio.setW(Math.cos(andruino_driver.gAzimut_odom / 2.0));
				odom.getPose().getPose().setOrientation(cuaternio);

				double[] tmpCov = { 1e-2, 0, 0, 0, 0, 0, 0, 1e-2, 0, 0, 0, 0,
						0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0,
						1e6, 0, 0, 0, 0, 0, 0, 1e3 };

				odom.getPose().setCovariance(tmpCov);

				publisher.publish(odom);

				// TF de base_link a odom
				TransformStamped odom2base_link = mMessageFactory
						.newFromType(TransformStamped._TYPE);
				odom2base_link.getHeader().setStamp(
						Time.fromMillis(time_delta_millis));
				odom2base_link.getHeader().setFrameId("odom");
				odom2base_link.setChildFrameId("base_link");

				odom2base_link.getTransform().getTranslation()
						.setX((double) andruino_driver.gx);
				odom2base_link.getTransform().getTranslation()
						.setY((double) andruino_driver.gy);
				odom2base_link.getTransform().getTranslation().setZ(0.0);

				odom2base_link.getTransform().getRotation().setX(0.0);
				odom2base_link.getTransform().getRotation().setY(0.0);
				odom2base_link.getTransform().getRotation()
						.setZ(Math.sin(andruino_driver.gAzimut_odom / 2.0));
				odom2base_link.getTransform().getRotation()
						.setW(Math.cos(andruino_driver.gAzimut_odom / 2.0));

				// Publicacion TF
				tfMessage tfm = publisher2.newMessage();
				List<TransformStamped> tfl = tfm.getTransforms();
				tfl.add(odom2base_link);
				publisher2.publish(tfm);

				Thread.sleep(100);

			}

		});

	}
}