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
		//origen=false;
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
				"andruino/beacons", "std_msgs/String");

		node.executeCancellableLoop(new CancellableLoop() {
			

			@Override
			protected void setup() {

			}

			@Override
			protected void loop() throws InterruptedException {

			
				StringBuffer mensaje = new StringBuffer();

				StringBuffer srbuf;

				srbuf = new StringBuffer(miSensorWifi.getBeacons());

				mensaje.append(srbuf);
				
				String wbuf = "iiibbb" + String.valueOf(miSensorWifi.corr) + "ww" + "###";
				
				andruino_driver.mSerial.write(wbuf.getBytes());

				std_msgs.String str = publisher.newMessage();

				str.setData(" Puntos de acceso próximos: " + miSensorWifi.getBeacons() + "\n");
				
				publisher.publish(str);
				
				Thread.sleep(5000);
			}

		});

	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("andruino_driver/beacons");
	}

}