package es.andruino.andruino_driver;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import android.os.SystemClock;
import sensor_msgs.Range;

public class andruinoROS_sensor_distancia_pub implements NodeMain {

	String base_frame_id = "base_link";
	String odom_frame_id = "odom";
	
	public int String2Int(String cadena){
		try
	    {
	      // the String to int conversion happens here
	      return Integer.parseInt(cadena.trim());

	    }
	    catch (NumberFormatException nfe)
	    {
	      //System.out.println("NumberFormatException: " + nfe.getMessage());
	    	return -9999;
	    }
		
	}
	
	public float String2Float(String cadena){
		try
	    {
	      // the String to int conversion happens here
	      return Float.parseFloat(cadena.trim());

	    }
	    catch (NumberFormatException nfe)
	    {
	      //System.out.println("NumberFormatException: " + nfe.getMessage());
	    	return -9999;
	    }
		
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
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		// return null;
		return GraphName.of("andruino_driver/distance");
	}

	@Override
	public void onStart(final ConnectedNode node) {
		final Publisher<std_msgs.String> publisher = node.newPublisher("andruino/distance", "std_msgs/String");
		
		/// RANGE o LASER SCAN???????????????? (PROBRAR!!!!)
		final Publisher<sensor_msgs.Range> publisherUS1 = node.newPublisher("hc_sr04_1", "sensor_msgs/Range");
		final Publisher<sensor_msgs.Range> publisherUS2 = node.newPublisher("hc_sr04_2", "sensor_msgs/Range");
		final Publisher<sensor_msgs.Range> publisherUS3 = node.newPublisher("hc_sr04_3", "sensor_msgs/Range");
		
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
		        
		        //if (aux.endsWith("###")){
		        if (aux.contains("###")){
		        
		        std_msgs.String str = publisher.newMessage();
		        
		        sensor_msgs.Range rangeUS1= publisherUS1.newMessage();
				
		        mensajeSerieRecibido=mensajeSerieRecibido+mensaje.toString();
		        
		        //Análisis del mensajer recibido y almacena datos en las variables globales
		        
				if (mensajeSerieRecibido.contains("ooosss")  && (mensajeSerieRecibido.indexOf("ooosss") < mensajeSerieRecibido.indexOf("###"))){
					long time_delta_millis = System.currentTimeMillis() - SystemClock.uptimeMillis(); //Quizas habria que restar los milisegundos del Loop!!!
					mensajeSerieRecibido= mensajeSerieRecibido.substring(mensajeSerieRecibido.indexOf("ooosss"),mensajeSerieRecibido.indexOf("###"));
		        	mensajeSerieRecibido=mensajeSerieRecibido.replace("ooosss",""); // Quito ### del final
			        mensajeSerieRecibido=mensajeSerieRecibido.replace("ww###","ww"); // Quito ### del final
					mensajeSerieRecibido=mensajeSerieRecibido.trim();
					mensajeSerieRecibido= mensajeSerieRecibido.replaceAll("[\n\r\\s]", "");
					mensajeSerieRecibido=mensajeSerieRecibido.replace("ww",";");
					String[] datosArduino = mensajeSerieRecibido.split("[;]");	
					
					//Variables globales (CAMBIAR!!!!!!!!!!!!!!!!!)
					andruino_driver.HCSR04_1=String2Int(datosArduino[0]);
					andruino_driver.HCSR04_2=String2Int(datosArduino[1]);
					andruino_driver.HCSR04_3=String2Int(datosArduino[2]);
					andruino_driver.LDR1=String2Int(datosArduino[3]);
					andruino_driver.LDR2=String2Int(datosArduino[4]);
					andruino_driver.LDR3=String2Int(datosArduino[5]);
					andruino_driver.dtLoop=String2Int(datosArduino[6]);
					andruino_driver.distancia=String2Float(datosArduino[7]);
					andruino_driver.x=String2Float(datosArduino[8]);
					andruino_driver.y=String2Float(datosArduino[9]);
					
					str.setData(mensajeSerieRecibido);
			        publisher.publish(str);
			        
					//Como el mensaje fue correctarremente recibido borro todo, para recibir el próximo mensaje.
					mensajeSerieRecibido="";
				
					//Publica los sensorRange
					//rangeUS1.getHeader().setStamp(new Time());
					rangeUS1.getHeader().setStamp(Time.fromMillis(time_delta_millis));
					rangeUS1.getHeader().setFrameId(base_frame_id);
					rangeUS1.setRadiationType((byte) 0); //Define el tipo como Sensor de Ultrasonido
					rangeUS1.setFieldOfView((float) 0.3);
					rangeUS1.setMinRange((float)0.1);
					rangeUS1.setMaxRange((float)2.0);
					rangeUS1.setRange(String2Float(datosArduino[0])/1000); //Lo paso a metros VERIFICAR!!!!!!!!!!!!!!!!!!!!!!!!!!!
					
					publisherUS1.publish(rangeUS1);
					
					
					
		        } else {
		        	
		        	if (mensajeSerieRecibido.length()>=4 && mensajeSerieRecibido.length()<200){
		        		
		        	mensajeSerieRecibido=mensajeSerieRecibido.substring(mensajeSerieRecibido.indexOf("###")+3,mensajeSerieRecibido.length());
		        
		        	} else {
		        		mensajeSerieRecibido="";
		        	}
		        }
		        
				//Fin de Análisis del mensaje
				
				//mensajeSerieRecibido="";
				//Thread.sleep(2000); //Limita la cantidad de mensaje recibidos
		        
		        }
		        else{
		        	mensajeSerieRecibido=mensajeSerieRecibido+mensaje.toString();
		        }
		        
			}
			
		});

		

	}

}
