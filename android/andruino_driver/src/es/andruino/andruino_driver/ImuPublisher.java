// Modified by Paco Lopez (@andruinos) from ImuPlisher.java (Chad Rockey)

package es.andruino.andruino_driver;


import java.util.List;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Looper;


import org.ros.node.ConnectedNode;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import sensor_msgs.Imu;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;


public class ImuPublisher implements NodeMain
{

  private ImuThread imuThread;
  private SensorListener sensorListener;
  private SensorManager sensorManager;
  private Publisher<Imu> publisher;
  
  private class ImuThread extends Thread
  {
	  private final SensorManager sensorManager;
	  private SensorListener sensorListener;
	  private Looper threadLooper;
	  
	  private final Sensor accelSensor;
	  private final Sensor gyroSensor;
	  private final Sensor quatSensor;
	  
	  private ImuThread(SensorManager sensorManager, SensorListener sensorListener)
	  {
		  this.sensorManager = sensorManager;
		  this.sensorListener = sensorListener;
		  this.accelSensor = this.sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
		  this.gyroSensor = this.sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
		  this.quatSensor = this.sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
	  }
	  
	    
	  public void run()
	  {
			Looper.prepare();
			this.threadLooper = Looper.myLooper();
			this.sensorManager.registerListener(this.sensorListener, this.accelSensor, SensorManager.SENSOR_DELAY_FASTEST);
			this.sensorManager.registerListener(this.sensorListener, this.gyroSensor, SensorManager.SENSOR_DELAY_FASTEST);
			this.sensorManager.registerListener(this.sensorListener, this.quatSensor, SensorManager.SENSOR_DELAY_FASTEST);
			Looper.loop();
			
	  }
	    
	    
	  public void shutdown()
	  {
	    	this.sensorManager.unregisterListener(this.sensorListener);
	    	if(this.threadLooper != null)
	    	{
	            this.threadLooper.quit();
	    	}
	  }
	}
  
  private class SensorListener implements SensorEventListener
  {

    private Publisher<Imu> publisher;
    
    private boolean hasAccel;
    private boolean hasGyro;
    private boolean hasQuat;
    
    private long accelTime;
    private long gyroTime;
    private long quatTime;
    
    private Imu imu;

    private SensorListener(Publisher<Imu> publisher, boolean hasAccel, boolean hasGyro, boolean hasQuat)
    {
      this.publisher = publisher;
      this.hasAccel = hasAccel;
      this.hasGyro = hasGyro;
      this.hasQuat = hasQuat;
      this.accelTime = 0;
      this.gyroTime = 0;
      this.quatTime = 0;
      this.imu = this.publisher.newMessage();
    }

//	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy)
	{
	}

//	@Override
	public void onSensorChanged(SensorEvent event)
	{
		if(event.sensor.getType() == Sensor.TYPE_ACCELEROMETER)
		{
			
			this.imu.getLinearAcceleration().setX(-1*event.values[2]);
			this.imu.getLinearAcceleration().setY(event.values[1]);
			this.imu.getLinearAcceleration().setZ(0.0);
			
			
			double[] tmpCov = {0.01,0,0, 0,0.01,0, 0,0,0.01};// TODO Make Parameter
			this.imu.setLinearAccelerationCovariance(tmpCov);
			this.accelTime = event.timestamp;
		}
		else if(event.sensor.getType() == Sensor.TYPE_GYROSCOPE)
		{
			
			this.imu.getAngularVelocity().setX(0.0);
			this.imu.getAngularVelocity().setY(0.0);
			this.imu.getAngularVelocity().setZ(event.values[0]);
			
			double[] tmpCov = {0.0025,0,0, 0,0.0025,0, 0,0,0.0025};// TODO Make Parameter
			this.imu.setAngularVelocityCovariance(tmpCov);
	        this.gyroTime = event.timestamp;
		}
		else if(event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR)
		
		{
	        float[] quaternion = new float[4];
	        
	        float[] rMat = new float[9]; //
	        float[] orientation = new float[3]; //
	        
	     
	        SensorManager.getRotationMatrixFromVector(rMat, event.values);      
	        this.imu.getOrientation().setX(0.0);
	        this.imu.getOrientation().setY(0.0);
	        
	       this.imu.getOrientation().setZ(Math.sin(andruino_driver.gAzimut / 2.0));
	        this.imu.getOrientation().setW(Math.cos(andruino_driver.gAzimut / 2.0));
	        
			double[] tmpCov = {0.001,0,0, 0,0.001,0, 0,0,0.001};// TODO Make Parameter
			this.imu.setOrientationCovariance(tmpCov);
	       	this.quatTime = event.timestamp;
		}
		
		if((this.accelTime != 0 || !this.hasAccel) && (this.gyroTime != 0 || !this.hasGyro) && (this.quatTime != 0 || !this.hasQuat))
		{
			
			long time_delta_millis = System.currentTimeMillis();
			this.imu.getHeader().setStamp(Time.fromMillis(time_delta_millis));
				
			this.imu.getHeader().setFrameId("base_footprint"); //ASí funciona robot_pose_ekf
			
			publisher.publish(this.imu);

			this.imu = this.publisher.newMessage();
			
			// Reset times
			this.accelTime = 0;
			this.gyroTime = 0;
			this.quatTime = 0;
			
		}
	}
  }

  
  public ImuPublisher(SensorManager manager)
  {
	  this.sensorManager = manager;
  }

  public GraphName getDefaultNodeName()
  {
	  return GraphName.of("andruino_driver/imuPublisher");
  }
  
  public void onError(Node node, Throwable throwable)
  {
  }

  public void onStart(ConnectedNode node)
  {
	  try
	  {
		  this.publisher = node.newPublisher("imu", "sensor_msgs/Imu");	
		  
		  	boolean hasAccel = false;
			boolean hasGyro = false;
			boolean hasQuat = false;
	
			List<Sensor> accelList = this.sensorManager.getSensorList(Sensor.TYPE_ACCELEROMETER);
			
			if(accelList.size() > 0)
			{
				hasAccel = true;
			}
			
			List<Sensor> gyroList = this.sensorManager.getSensorList(Sensor.TYPE_GYROSCOPE);
			if(gyroList.size() > 0)
			{
				hasGyro = true;
			}
			
			List<Sensor> quatList = this.sensorManager.getSensorList(Sensor.TYPE_ROTATION_VECTOR);
			if(quatList.size() > 0)
			{
				hasQuat = true;
			}
			
			this.sensorListener = new SensorListener(publisher, hasAccel, hasGyro, hasQuat);
			this.imuThread = new ImuThread(this.sensorManager, sensorListener);
			this.imuThread.start();		
	  }
	  catch (Exception e)
	  {
		  if (node != null)
		  {
			  node.getLog().fatal(e);
		  }
		  else
		  {
			  e.printStackTrace();
		  }
	  }
  }

//@Override
  public void onShutdown(Node arg0)
  {
	  this.imuThread.shutdown();
	
	  try
	  {
		  this.imuThread.join();
	  }
	  catch (InterruptedException e)
	  {
		  e.printStackTrace();
	  }
  }

//@Override
  public void onShutdownComplete(Node arg0)
  {
  }

}

