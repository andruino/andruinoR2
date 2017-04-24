package es.andruino.andruino_driver;

import java.util.Map;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterListener;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Subscriber;

import android.speech.tts.TextToSpeech;

public class andruinoROS_command_sub implements NodeMain {

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

		final ParameterTree params = connectedNode.getParameterTree();

		Subscriber<std_msgs.String> subscriber = connectedNode.newSubscriber(
				"/cmd", std_msgs.String._TYPE);
		subscriber.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String message) {
				;

				String wbuf = message.getData();

				andruino_driver.mSerial.write(wbuf.getBytes());

				if (!wbuf.contains("iiiqqq")) {
					andruino_driver.t1.speak("Command received, sir",
							TextToSpeech.QUEUE_FLUSH, null);
				}

				// Cambia el estado
				if (wbuf.contains("iiittt") == true
						&& wbuf.contains("ww###") == true) {
					String[] arrayMsg = wbuf.split("ww");

					if (arrayMsg[0].contains("16")) {
						andruino_driver.gEstado = 16;
						// Debería hacerlo para todos los estados
					} else {
						andruino_driver.gEstado = 0;
					}

					if (arrayMsg[0].contains("17")) {

						andruino_driver.t1.speak(arrayMsg[1],
								TextToSpeech.QUEUE_FLUSH, null);

					}

				}

			}
		});

		// Definición de los parámetros(deben ser los mismos valores que en
		// Arduino)

		params.set(andruino_driver.prefijoNS + "/Kvel", 1.1);
		params.set(andruino_driver.prefijoNS + "/Komega", 1.0);

		params.set(andruino_driver.prefijoNS + "/Kssr", 8.34);
		params.set(andruino_driver.prefijoNS + "/Kssb", 1.65);
		params.set(andruino_driver.prefijoNS + "/KssIccr", 1.0);

		params.set(andruino_driver.prefijoNS + "/SlopeLeft", 111.5);
		params.set(andruino_driver.prefijoNS + "/OffsetLeft", 112.41);
		params.set(andruino_driver.prefijoNS + "/SlopeRight", 101.5);
		params.set(andruino_driver.prefijoNS + "/OffsetRight", 103.41);

		params.set(andruino_driver.prefijoNS + "/KpLine", 100.00);
		params.set(andruino_driver.prefijoNS + "/KiLine", 3.0);
		params.set(andruino_driver.prefijoNS + "/KdLine", 0.0);

		params.set(andruino_driver.prefijoNS + "/KpSpin", 30.0);
		params.set(andruino_driver.prefijoNS + "/KiSpin", 0.0);
		params.set(andruino_driver.prefijoNS + "/KdSpin", 0.0);

		params.set(andruino_driver.prefijoNS + "/KpArc", 103.41);
		params.set(andruino_driver.prefijoNS + "/KiArc", 0.0);
		params.set(andruino_driver.prefijoNS + "/KdArc", 0.0);

		params.set(andruino_driver.prefijoNS + "/PWMMAX", 255.0);
		params.set(andruino_driver.prefijoNS + "/PWMMIN", 155.0);

		Map<String, Double> subtree = (Map<String, Double>) params
				.getMap(andruino_driver.prefijoNS);

		for (final Map.Entry<String, Double> entry : subtree.entrySet()) {

			params.addParameterListener(
					andruino_driver.prefijoNS + "/" + entry.getKey(),

					new ParameterListener() {
						@Override
						public void onNewValue(Object value) {
							float fpar = (float) params
									.getDouble(andruino_driver.prefijoNS + "/"
											+ entry.getKey());

							String wbuf = "iiippp" + entry.getKey() + "ww"
									+ Float.toString(fpar) + "ww###";// entrySet().iterator()

							if ("Kvel".equals(entry.getKey())) {
								wbuf = "iiippp" + "000" + "ww"
										+ Float.toString(fpar) + "ww###";// entrySet().iterator()
							}

							if ("Komega".equals(entry.getKey())) {
								wbuf = "iiippp" + "001" + "ww"
										+ Float.toString(fpar) + "ww###";// entrySet().iterator()
							}

							if ("Kssr".equals(entry.getKey())) {
								wbuf = "iiippp" + "002" + "ww"
										+ Float.toString(fpar) + "ww###";// entrySet().iterator()
							}

							if ("Kssb".equals(entry.getKey())) {
								wbuf = "iiippp" + "003" + "ww"
										+ Float.toString(fpar) + "ww###";// entrySet().iterator()
							}

							if ("KssIccr".equals(entry.getKey())) {
								wbuf = "iiippp" + "004" + "ww"
										+ Float.toString(fpar) + "ww###";// entrySet().iterator()
							}

							if ("SlopeLeft".equals(entry.getKey())) {
								wbuf = "iiippp" + "005" + "ww"
										+ Float.toString(fpar) + "ww###";// entrySet().iterator()
							}

							if ("OffsetLeft".equals(entry.getKey())) {
								wbuf = "iiippp" + "006" + "ww"
										+ Float.toString(fpar) + "ww###";// entrySet().iterator()
							}

							if ("SlopeRight".equals(entry.getKey())) {
								wbuf = "iiippp" + "007" + "ww"
										+ Float.toString(fpar) + "ww###";// entrySet().iterator()
							}

							if ("OffsetRight".equals(entry.getKey())) {
								wbuf = "iiippp" + "008" + "ww"
										+ Float.toString(fpar) + "ww###";// entrySet().iterator()
							}

							if ("KpLine".equals(entry.getKey())) {
								wbuf = "iiippp" + "009" + "ww"
										+ Float.toString(fpar) + "ww###";// entrySet().iterator()
							}

							if ("KiLine".equals(entry.getKey())) {
								wbuf = "iiippp" + "010" + "ww"
										+ Float.toString(fpar) + "ww###";// entrySet().iterator()
							}

							if ("KdLine".equals(entry.getKey())) {
								wbuf = "iiippp" + "011" + "ww"
										+ Float.toString(fpar) + "ww###";// entrySet().iterator()
							}

							if ("KpSpin".equals(entry.getKey())) {
								wbuf = "iiippp" + "012" + "ww"
										+ Float.toString(fpar) + "ww###";// entrySet().iterator()
							}
							if ("KiSpin".equals(entry.getKey())) {
								wbuf = "iiippp" + "013" + "ww"
										+ Float.toString(fpar) + "ww###";// entrySet().iterator()
							}
							if ("KdSpin".equals(entry.getKey())) {
								wbuf = "iiippp" + "014" + "ww"
										+ Float.toString(fpar) + "ww###";// entrySet().iterator()
							}

							if ("KpArc".equals(entry.getKey())) {
								wbuf = "iiippp" + "015" + "ww"
										+ Float.toString(fpar) + "ww###";// entrySet().iterator()
							}
							if ("KiArc".equals(entry.getKey())) {
								wbuf = "iiippp" + "016" + "ww"
										+ Float.toString(fpar) + "ww###";// entrySet().iterator()
							}
							if ("KdArc".equals(entry.getKey())) {
								wbuf = "iiippp" + "017" + "ww"
										+ Float.toString(fpar) + "ww###";// entrySet().iterator()
							}

							if ("PWMMAX".equals(entry.getKey())) {
								wbuf = "iiippp" + "018" + "ww"
										+ Float.toString(fpar) + "ww###";// entrySet().iterator()
							}
							if ("PWMMIN".equals(entry.getKey())) {
								wbuf = "iiippp" + "019" + "ww"
										+ Float.toString(fpar) + "ww###";// entrySet().iterator()
							}
							andruino_driver.mSerial.write(wbuf.getBytes());
							andruino_driver.t1.speak(
									"Parameter modified, sir. With value "
											+ wbuf, TextToSpeech.QUEUE_FLUSH,
									null);

						}
					});

		}

	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("andruino/cmd");
	}

}
