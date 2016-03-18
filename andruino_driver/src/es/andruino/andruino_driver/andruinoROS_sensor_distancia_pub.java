package es.andruino.andruino_driver;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
//import sensor_msgs.Range;

public class andruinoROS_sensor_distancia_pub implements NodeMain {


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
		return GraphName.of("andruino_driver/distance");
	}

	@Override
	public void onStart(final ConnectedNode node) {
		final Publisher<std_msgs.String> publisher = node.newPublisher("andruino/distance", "std_msgs/String");
	////!!!!! CAMBIO a RANGEfinal Publisher<sensor_msgs.Range> publisher = node.newPublisher("andruino/distance", "sensor_msgs/Range");
		
		
		node.executeCancellableLoop(new CancellableLoop() {
	
			private String mensajeSerieRecibido;

			@Override
			protected void setup() {
				
				mensajeSerieRecibido="Ningun mensaje recibido";
			}

		
			protected void loop() throws InterruptedException {
			
				int i,len=0;
				String aux;
				StringBuffer mensaje=new StringBuffer();
				
		        // [FTDriver] Crea el buffer del puerto serie, con 4096 bytes
		        byte[] rbuf = new byte[4096]; 

		        // [FTDriver] Lee del puerto Serie
		        len = andruino_driver.mSerial.read(rbuf);
		        
		        
		        for(i=0; i<len; i++) {
		            mensaje.append((char) rbuf[i]);	
		        }
		        
		        aux=mensaje.toString();
		        
		        if (aux.endsWith("##")){
		        // OJO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		        //Ahora en un solo buffer recoge varios datos, entonces deberia hacer un bucle para analizar todos los mensajes del buffer
		        
		        std_msgs.String str = publisher.newMessage();
				
		        mensajeSerieRecibido=mensajeSerieRecibido+mensaje.toString();
		        str.setData(" Sensor Distancia: " + mensajeSerieRecibido);
		        
		        
				publisher.publish(str);
				
				//Thread.sleep(2000);
				mensajeSerieRecibido="";
		        
			}
		        else{
		        	mensajeSerieRecibido=mensajeSerieRecibido+mensaje.toString();
		        }
		        
			}
			
		});

		

	}

}
