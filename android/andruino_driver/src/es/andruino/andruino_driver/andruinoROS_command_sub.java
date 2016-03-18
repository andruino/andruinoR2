package es.andruino.andruino_driver;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;




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
		
		Subscriber<std_msgs.String> subscriber = connectedNode.newSubscriber(
				"andruino/cmd", std_msgs.String._TYPE);
		subscriber.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String message) {
				//log.info("I heard: \"" + message.getData() + "\"");
				
				String wbuf = message.getData();
				andruino_driver.mSerial.write(wbuf.getBytes());
				
			}
		}); // ,50); //AQUI
	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("andruino/cmd");
	}

}
