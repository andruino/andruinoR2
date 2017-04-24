package es.andruino.andruino_driver;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import android.content.Context;

public class andruinoROS_sensor_Wifi implements NodeMain {

	public sensorWifi miSensorWifi;
	boolean origen;

	public andruinoROS_sensor_Wifi(Context c) {
		miSensorWifi = new sensorWifi(c);

	}

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
	public void onStart(final ConnectedNode node) {
		final Publisher<std_msgs.String> publisher = node.newPublisher(
				"beacons", "std_msgs/String");

		node.executeCancellableLoop(new CancellableLoop() {

			@Override
			protected void setup() {

			}

			@Override
			protected void loop() throws InterruptedException {

				StringBuffer srbuf;

				// Publica la correlación más todos los puntos obtenidos

				std_msgs.String str = publisher.newMessage();

				str.setData(miSensorWifi.getBeacons());

				publisher.publish(str);

				Thread.sleep(3333);
			}

		});

	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("andruino_driver/beacons");
	}

}