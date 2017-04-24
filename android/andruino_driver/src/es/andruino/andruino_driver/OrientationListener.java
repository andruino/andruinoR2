package es.andruino.andruino_driver;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

public class OrientationListener implements SensorEventListener {

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
	float velZ = 0;
	float distZ = 0;
	float prevVelZ = 0;
	float prevAccelZ = 0;
	float accelKFactor = 0.1f;

	public void onAccuracyChanged(Sensor sensor, int accuracy) {
	}

	@Override
	public void onSensorChanged(SensorEvent event) {

		if (event.sensor.getType() == Sensor.TYPE_GAME_ROTATION_VECTOR) {

			SensorManager.getRotationMatrixFromVector(rMat, event.values);
			tiempo = (event.timestamp / 1000000);
			azimut = (SensorManager.getOrientation(rMat, orientation)[0]);

		}
		if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
			omega = event.values[0];
			tiempo = (event.timestamp / 1000000);
		}

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

		sm.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR),
				SensorManager.SENSOR_DELAY_FASTEST);
		sm.registerListener(this, sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
				SensorManager.SENSOR_DELAY_FASTEST);

	}

	protected void onPause() {
		SensorManager sm = (SensorManager) context
				.getSystemService(Context.SENSOR_SERVICE);
		sm.unregisterListener(this);
	}

}