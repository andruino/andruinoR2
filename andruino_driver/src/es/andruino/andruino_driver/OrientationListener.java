package es.andruino.andruino_driver;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

public class OrientationListener implements SensorEventListener {

	private final static float PI = (float) Math.PI;
	private final static float TWO_PI = PI*2;


	private Context context;

	static final float ALPHA = 0.5f;
	
	float azimut;
	

	float R[] = new float[9];
	float I[] = new float[9];
	
	
	float[] orientation = new float[3];
	float[] rMat = new float[9];
	float[] rMat2 = new float[9];
	
	public void onAccuracyChanged( Sensor sensor, int accuracy ) {}

	
	@Override
	public void onSensorChanged( SensorEvent event ) {
		if( event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR ){
		
			SensorManager.getRotationMatrixFromVector( rMat, event.values );
			
			azimut = (float) mod( (SensorManager.getOrientation( rMat, orientation )[0]) + TWO_PI, TWO_PI)- PI;//important
      	
		}
	}

	private double mod(double a, double b){
        return a % b;
    }
	

	
	public float[] getSensorValue() {
		return orientation;
	}

	public float getAzimut() {
		return azimut;
	}

	// Constructor
	public OrientationListener(Context c) {
		this.context = c;
		
	}


	protected void onResume() {

		
		SensorManager sm = (SensorManager) context
				.getSystemService(Context.SENSOR_SERVICE);

		sm.registerListener(this,
				sm.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR),
				SensorManager.SENSOR_DELAY_NORMAL);
				

	}

	protected void onPause() {
		SensorManager sm = (SensorManager) context
				.getSystemService(Context.SENSOR_SERVICE);
		sm.unregisterListener(this);
	}
	
	
}