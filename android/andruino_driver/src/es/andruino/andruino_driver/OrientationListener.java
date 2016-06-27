package es.andruino.andruino_driver;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.SystemClock;

public class OrientationListener implements SensorEventListener {

	private final static float PI = (float) Math.PI;
	private final static float TWO_PI = PI * 2;

	private Context context;

	float azimut;
	float omega;
	float omegaAnterior;
	long tiempo;

	float R[] = new float[9];
	float I[] = new float[9];

	float[] orientation = new float[3];
	float[] rMat = new float[9];
	float[] rMat2 = new float[9];

	float accelZ = 0;
	float velZ =0;
	float distZ=0;
	float prevVelZ =0;
	float prevAccelZ=0;
	float accelKFactor=0.1f;
	
	private static final float NS2S = 1.0f / 1000000000.0f;
	private float timestamp = 0;

	public void onAccuracyChanged(Sensor sensor, int accuracy) {
	}

	@Override
	public void onSensorChanged(SensorEvent event) {
		// if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
		// El sensor ge magnético introduce errores (muy cerca de motores y
		// campos mágneticos....??!!??))
		if (event.sensor.getType() == Sensor.TYPE_GAME_ROTATION_VECTOR) {

			SensorManager.getRotationMatrixFromVector(rMat, event.values);
			tiempo = (event.timestamp / 1000000);
			azimut = (SensorManager.getOrientation(rMat, orientation)[0]);
			/*
			 * azimut = (float) mod( (SensorManager.getOrientation(rMat,
			 * orientation)[0]) + TWO_PI, TWO_PI) - PI;
			 */
		}
		if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
			omega = event.values[0];
			tiempo = (event.timestamp / 1000000);
		}
		/*
		if (event.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) {
			// From
			// http://stackoverflow.com/questions/30443727/integrating-android-accelerometer-to-find-velocity-and-position
			// accelX = event.values[0];
			// accelY = event.values[1];
			accelZ = event.values[2];
			
			//accelX = (accelX * accelKFactor) + prevAccelX * (1 - accelKFactor);
			//accelY = (accelY * accelKFactor) + prevAccelY * (1 - accelKFactor);
			accelZ = (accelZ * accelKFactor) + prevAccelZ * (1 - accelKFactor);
		
				float interval = (event.timestamp - timestamp) * NS2S;
				timestamp=event.timestamp;
				
				//velX += accelX * interval;
				//velY += accelY * interval;
				velZ += accelZ * interval;

				//distX += prevVelX + velX * interval;
				//distY += prevVelY + velY * interval;
				distZ += prevVelZ + velZ * interval;

				//prevAccelX = accelX;
				//prevAccelY = accelY;
				prevAccelZ = accelZ;

				//prevVelX = velX;
				//prevVelY = velY;
				prevVelZ = velZ;
					
			
		}*/

	}

	private double mod(double a, double b) {
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
				// sm.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR),
				sm.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR),
				SensorManager.SENSOR_DELAY_FASTEST);
		sm.registerListener(this, sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
				SensorManager.SENSOR_DELAY_FASTEST);
		/*sm.registerListener(this,
				sm.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION),
				SensorManager.SENSOR_DELAY_NORMAL);
				*/

	}

	protected void onPause() {
		SensorManager sm = (SensorManager) context
				.getSystemService(Context.SENSOR_SERVICE);
		sm.unregisterListener(this);
	}

}