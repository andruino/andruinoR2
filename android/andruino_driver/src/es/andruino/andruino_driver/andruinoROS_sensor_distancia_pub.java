package es.andruino.andruino_driver;

import geometry_msgs.TransformStamped;
import java.util.List;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import tf.tfMessage;

public class andruinoROS_sensor_distancia_pub implements NodeMain {

	String base_frame_id = "base_link";
	String odom_frame_id = "odom";
	String us1_frame_id = "hc_sr04_1_link";
	String us2_frame_id = "hc_sr04_2_link";
	String us3_frame_id = "hc_sr04_3_link";

	public int String2Int(String cadena) {
		try {
			// the String to int conversion happens here
			return Integer.parseInt(cadena.trim());

		} catch (NumberFormatException nfe) {
			System.out.println("NumberFormatException: " + nfe.getMessage());
			return -9999;
		}

	}

	public float String2Float(String cadena) {
		try {
			// the String to int conversion happens here
			return Float.parseFloat(cadena.trim());

		} catch (NumberFormatException nfe) {
			System.out.println("NumberFormatException: " + nfe.getMessage());
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
		final Publisher<std_msgs.String> publisher = node.newPublisher("distance", "std_msgs/String");
		final Publisher<sensor_msgs.Range> publisherUS1 = node.newPublisher("hc_sr04_1", "sensor_msgs/Range");
		final Publisher<sensor_msgs.Range> publisherUS2 = node.newPublisher("hc_sr04_2", "sensor_msgs/Range");
		final Publisher<sensor_msgs.Range> publisherUS3 = node.newPublisher("hc_sr04_3", "sensor_msgs/Range");
		final Publisher<sensor_msgs.LaserScan> publisherLS = node.newPublisher("scan", "sensor_msgs/LaserScan");
		
		
		
		final Publisher<tfMessage> publisherUS = node.newPublisher("/tf", tfMessage._TYPE);
		
		final MessageFactory mf2 = node.getTopicMessageFactory();
		
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
				
				float valorUS1=0F;
				float valorUS2=0F;
				float valorUS3=0F;
				
				long time_delta_millis = System.currentTimeMillis(); 
				byte[] rbuf = new byte[8192]; 
				
		        len = andruino_driver.mSerial.read(rbuf);
		        
		        
		        for(i=0; i<len; i++) {
		            mensaje.append((char) rbuf[i]);	
		        }
		        
		        aux=mensaje.toString();
		        if (aux.contains("###")){
		        
		       
		        std_msgs.String str = publisher.newMessage();
		        
		        sensor_msgs.Range rangeUS1= publisherUS1.newMessage();
		        sensor_msgs.Range rangeUS2= publisherUS2.newMessage();
		        sensor_msgs.Range rangeUS3= publisherUS3.newMessage();
		        
		        sensor_msgs.LaserScan laserScan= publisherLS.newMessage();
				
		        mensajeSerieRecibido=mensajeSerieRecibido+aux;
		        
		        
		        //Análisis del mensajer recibido y almacena datos en las variables globales
		         
				if (mensajeSerieRecibido.contains("ooosss")  && (mensajeSerieRecibido.indexOf("ooosss") < mensajeSerieRecibido.indexOf("###"))){
					
				
					
					
					mensajeSerieRecibido= mensajeSerieRecibido.substring(mensajeSerieRecibido.indexOf("ooosss"),mensajeSerieRecibido.indexOf("###"));
		        	mensajeSerieRecibido=mensajeSerieRecibido.replace("ooosss",""); // Quito ### del final
			        mensajeSerieRecibido=mensajeSerieRecibido.replace("ww###",""); // Quito ### del final
					mensajeSerieRecibido=mensajeSerieRecibido.trim();
					mensajeSerieRecibido= mensajeSerieRecibido.replaceAll("[\n\r\\s]", "");
					mensajeSerieRecibido=mensajeSerieRecibido.replace("ww",";");
					String[] datosArduino = mensajeSerieRecibido.split("[;]");	
					
					//Se deben recibir 12 valores que es la cadena que pasa el Arduino al Android
				    if (datosArduino.length==12){
					
					
					if (datosArduino[8].length() > 0) {
					try
				    {
					andruino_driver.gx=(Float.parseFloat(datosArduino[8].trim())/1000F);

				    }
				    catch (NumberFormatException nfe)
				    {
				      System.out.println("NumberFormatException: " + nfe.getMessage());
				 
				    }
					}
					
					if (datosArduino[9].length() > 0) {
						try
					    {
							andruino_driver.gy=(Float.parseFloat(datosArduino[9].trim())/1000F);

					    }
					    catch (NumberFormatException nfe)
					    {
					      System.out.println("NumberFormatException: " + nfe.getMessage());
					 
					    }
						}
					if (datosArduino[10].length() > 0) {
						try
					    {
								andruino_driver.gAzimut_odom=(Float.parseFloat(datosArduino[10].trim())/1000F);

					    }
					    catch (NumberFormatException nfe)
					    {
					      System.out.println("NumberFormatException: " + nfe.getMessage());
					 
					    }
						}
					
					str.setData(mensajeSerieRecibido);
			        publisher.publish(str);
			        
					//Como el mensaje fue correctarremente recibido borro todo, para recibir el próximo mensaje.
					mensajeSerieRecibido="";
				
					
					if (datosArduino[0].length() > 0) {
						
							
							//Publica los sensorRange
							rangeUS1.getHeader().setStamp(Time.fromMillis(time_delta_millis));
							rangeUS1.getHeader().setFrameId(us1_frame_id);
							rangeUS1.setRadiationType((byte) 0); //Define el tipo como Sensor de Ultrasonido
							rangeUS1.setFieldOfView((float) 0.3);
							rangeUS1.setMinRange((float)0.1);
							rangeUS1.setMaxRange((float)2.0);
							
							try
						    {
							
								valorUS1=Float.parseFloat(datosArduino[0].trim())/1000F;
							

						    }
								catch (NumberFormatException nfe)
								{
									System.out.println("NumberFormatException: " + nfe.getMessage());
									valorUS1=-1;
								}
							rangeUS1.setRange(valorUS1); 
							publisherUS1.publish(rangeUS1);
						}
					
					
				
				
					if (datosArduino[1].length() > 0) {
						
					
					rangeUS2.getHeader().setStamp(Time.fromMillis(time_delta_millis));
					
					rangeUS2.getHeader().setFrameId(us2_frame_id);
					rangeUS2.setRadiationType((byte) 0); //Define el tipo como Sensor de Ultrasonido
					rangeUS2.setFieldOfView((float) 0.3);
					rangeUS2.setMinRange((float)0.1);
					rangeUS2.setMaxRange((float)2.0);
					
					try
				    {
						valorUS2=Float.parseFloat(datosArduino[1].trim())/1000F;
				
					

				    }
						catch (NumberFormatException nfe)
					{
							System.out.println("NumberFormatException: " + nfe.getMessage());
							valorUS2=-1;	
					}
					rangeUS2.setRange(valorUS2); 
					
					publisherUS2.publish(rangeUS2);
					
					
					}
					
					
					
					if (datosArduino[2].length() > 0) {
				
					rangeUS3.getHeader().setStamp(Time.fromMillis(time_delta_millis));
					rangeUS3.getHeader().setFrameId(us3_frame_id);
					rangeUS3.setRadiationType((byte) 0); //Define el tipo como Sensor de Ultrasonido
					rangeUS3.setFieldOfView((float) 0.3);
					rangeUS3.setMinRange((float)0.1);
					rangeUS3.setMaxRange((float)2.0);
					
					try
				    {
						valorUS3=Float.parseFloat(datosArduino[2].trim())/1000F;
					
					

				    }
						catch (NumberFormatException nfe)
						{
							System.out.println("NumberFormatException: " + nfe.getMessage());
			 
						}
					
						rangeUS3.setRange(valorUS3); 
					
					publisherUS3.publish(rangeUS3);
					
					}

					//TF de base_link a odom
					//US1
					TransformStamped base_link_US1 = mf2.newFromType(TransformStamped._TYPE);
					
					base_link_US1.getHeader().setStamp(Time.fromMillis(time_delta_millis));
					base_link_US1.getHeader().setFrameId("base_link");
					base_link_US1.setChildFrameId(us1_frame_id);
					
					
					base_link_US1.getTransform().getTranslation().setX(0.0);
					base_link_US1.getTransform().getTranslation().setY(-0.03);
					base_link_US1.getTransform().getTranslation().setZ(0.02);
					
					base_link_US1.getTransform().getRotation().setX(0.0);
					base_link_US1.getTransform().getRotation().setY(0.0);
					base_link_US1.getTransform().getRotation().setZ(-0.38267504);
					base_link_US1.getTransform().getRotation().setW(0.923879);
					
					//US2
					TransformStamped base_link_US2 = mf2.newFromType(TransformStamped._TYPE);
					
					base_link_US2.getHeader().setStamp(Time.fromMillis(time_delta_millis));
					base_link_US2.getHeader().setFrameId("base_link");
					base_link_US2.setChildFrameId(us2_frame_id);
					
					
					base_link_US2.getTransform().getTranslation().setX(0.02);
					base_link_US2.getTransform().getTranslation().setY(0);
					base_link_US2.getTransform().getTranslation().setZ(0.02);
					
					base_link_US2.getTransform().getRotation().setX(0.0);
					base_link_US2.getTransform().getRotation().setY(0.0);
					base_link_US2.getTransform().getRotation().setZ(0);
					base_link_US2.getTransform().getRotation().setW(1);
					
					//US3
					TransformStamped base_link_US3 = mf2.newFromType(TransformStamped._TYPE);
					
					base_link_US3.getHeader().setStamp(Time.fromMillis(time_delta_millis));
					base_link_US3.getHeader().setFrameId("base_link");
					base_link_US3.setChildFrameId(us3_frame_id);
					
					
					base_link_US3.getTransform().getTranslation().setX(0.0);
					base_link_US3.getTransform().getTranslation().setY(0.03);
					base_link_US3.getTransform().getTranslation().setZ(0.02);
					
					base_link_US3.getTransform().getRotation().setX(0.0);
					base_link_US3.getTransform().getRotation().setY(0.0);
					base_link_US3.getTransform().getRotation().setZ(0.38267504);
					base_link_US3.getTransform().getRotation().setW(0.923879);
					
					//Publicacion TF
					tfMessage tfmUS = publisherUS.newMessage();
					List<TransformStamped> tflUS = tfmUS.getTransforms();
					tflUS.add(base_link_US1);
					tflUS.add(base_link_US2);
					tflUS.add(base_link_US3);
					publisherUS.publish(tfmUS);
					
					
					//LaserScan Fake basado en US
					
					float[] rangosSL ={0, 0, 0 , valorUS1, valorUS1, valorUS1, 0, valorUS2,valorUS2,valorUS2,0 , valorUS3, valorUS3, valorUS3,0,0};
					
					laserScan.getHeader().setStamp(Time.fromMillis(time_delta_millis));
					laserScan.getHeader().setFrameId("base_link");
					laserScan.setAngleMax((float)(-Math.PI));
					laserScan.setAngleMin((float)(-Math.PI/2));
					laserScan.setAngleIncrement((float)(Math.PI/16));
					laserScan.setTimeIncrement(0F); //Parámetro a verificar
					laserScan.setScanTime(0F); //Parámetro a verificar
					laserScan.setRangeMin((float)0.1);
					laserScan.setRangeMax((float)0.5); //Prueba para evitar echos!!
					laserScan.setRanges(rangosSL);
					
					publisherLS.publish(laserScan);
				
				    } 
					
		        } else  {
		        	
		        	// Si recibe un mensaje de calibración para pintar la curva PWM / omega
		        	if (mensajeSerieRecibido.contains("oooccc") && (mensajeSerieRecibido.indexOf("oooccc") < mensajeSerieRecibido.indexOf("###"))){
		        		mensajeSerieRecibido= mensajeSerieRecibido.substring(mensajeSerieRecibido.indexOf("oooccc"),mensajeSerieRecibido.indexOf("###"));
			        	mensajeSerieRecibido=mensajeSerieRecibido.replace("oooccc",""); // Quito ### del final
				        mensajeSerieRecibido=mensajeSerieRecibido.replace("ww###","ww"); // Quito ### del final
						mensajeSerieRecibido=mensajeSerieRecibido.trim();
						mensajeSerieRecibido= mensajeSerieRecibido.replaceAll("[\n\r\\s]", "");
						mensajeSerieRecibido=mensajeSerieRecibido.replace("ww",";");	
						str.setData(mensajeSerieRecibido);
				        publisher.publish(str);
		        	}
		        	
		        	if (mensajeSerieRecibido.length()>=4 && mensajeSerieRecibido.length()<200){
		        	
		        	
		        			mensajeSerieRecibido=mensajeSerieRecibido.substring(mensajeSerieRecibido.indexOf("###")+3,mensajeSerieRecibido.length());
		        	
		        	} else { 
		        		mensajeSerieRecibido="";
		        	}
		        }
		    
		        
		        }
		        else{
		        	if (aux.length()>200)
		        		mensajeSerieRecibido="";
		        	if (aux.length()>=1)
		        		mensajeSerieRecibido=mensajeSerieRecibido+aux;	
		        }
		      
			}
			
			
		});

		

	}
}
