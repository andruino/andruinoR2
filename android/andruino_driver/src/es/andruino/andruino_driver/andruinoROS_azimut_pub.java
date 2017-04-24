package es.andruino.andruino_driver;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import android.content.Context;
import android.speech.tts.TextToSpeech;

public class andruinoROS_azimut_pub implements NodeMain {

	private OrientationListener oListener;

	public andruinoROS_azimut_pub(Context c) {
		oListener = new OrientationListener(c);
	}

	@Override
	public void onError(Node arg0, Throwable arg1) {
		// TODO Auto-generated method stub

	}

	@Override
	public void onShutdown(Node arg0) {
		// TODO Auto-generated method stub
		oListener.onPause();

	}

	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		// return null;
		return GraphName.of("andruino_driver/azimut");
	}

	@Override
	public void onStart(final ConnectedNode node) {
		final Publisher<std_msgs.String> publisher = node.newPublisher(
				"azimut", "std_msgs/String");

		oListener.onResume();

		node.executeCancellableLoop(new CancellableLoop() {

			@Override
			protected void setup() {

			}

			@Override
			protected void loop() throws InterruptedException {
				if (andruino_driver.gEstado == 0) {

					// Variables globales
					andruino_driver.gAzimut = oListener.azimut;
					andruino_driver.gOmega = oListener.omega;

					// Envio de mensaje por el puerto serie
					// Formato Mensaje: iiiaaaXXXww###, donde XXX es el azimut
					String wbuf = "iiiaaa"
							+ String.valueOf((oListener.azimut * 10000.0))
							+ "ww"
							+ String.valueOf((oListener.omega * 10000.0))
							+ "ww###";
					andruino_driver.mSerial.write(wbuf.getBytes());

					std_msgs.String str = publisher.newMessage();
					str.setData(wbuf);
					publisher.publish(str);

					Thread.sleep(100); // 150); //Sólo publica valores cada 100
										// ms

				}
			}
		});

	}
}
