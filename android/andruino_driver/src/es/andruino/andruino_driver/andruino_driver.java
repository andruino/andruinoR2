package es.andruino.andruino_driver;

import jp.ksksue.driver.serial.FTDriver;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.android.view.camera.RosCameraPreviewView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import android.annotation.SuppressLint;
import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.hardware.Camera;
import android.hardware.SensorManager;
import android.hardware.usb.UsbManager;
//import android.location.LocationManager;
import android.os.Bundle;
import android.view.MotionEvent;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Toast;

public class andruino_driver extends RosActivity {

	private static SensorManager mSensorManager;
	//private static LocationManager mLocationManager;

	// Camara
	private int cameraId;
	private RosCameraPreviewView rosCameraPreviewView;

	// USB Serial. [FTDriver] Object
	static FTDriver mSerial;

	// [FTDriver] Permission String
	private static final String ACTION_USB_PERMISSION = "jp.ksksue.tutorial.USB_PERMISSION";

	// Nodos
	private andruinoROS_azimut_pub my_andruinoROS_azimut_pub;
	private andruinoROS_command_sub my_andruinoROS_command_sub;
	private andruinoROS_sensor_distancia_pub my_andruinoROS_sensor_distancia_pub;
	private andruinoROS_sensor_Wifi my_andruinoROS_sensor_wifi_pub;

	// Nodos de android_sensor
	//private NavSatFixPublisher fix_pub;
	private ImuPublisher imu_pub;

	public andruino_driver() {
		super("andruino_driver", "andruino_driver");
	}

	public void onCreate(Bundle savedInstanceState) {

		super.onCreate(savedInstanceState);

		//mLocationManager = (LocationManager) this.getSystemService(Context.LOCATION_SERVICE);
		mSensorManager = (SensorManager) this.getSystemService(SENSOR_SERVICE);

		// Quita el título de la aplicación y máximiza la pantalla
		requestWindowFeature(Window.FEATURE_NO_TITLE);
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);

		// Carga el xml
		setContentView(R.layout.main);

		// La pantalla de Android muestra la cámara
		rosCameraPreviewView = (RosCameraPreviewView) findViewById(R.id.ros_camera_preview_view);

		// USB Serial. FTDriver] Crea Instancia
		mSerial = new FTDriver(
				(UsbManager) getSystemService(Context.USB_SERVICE));

		// [FTDriver] setPermissionIntent() before begin()
		PendingIntent permissionIntent = PendingIntent.getBroadcast(this, 0,
				new Intent(ACTION_USB_PERMISSION), 0);
		mSerial.setPermissionIntent(permissionIntent);

		// Abre el puerto Serie a 9600. SUBIR !!!!!!!
		if (mSerial.begin(FTDriver.BAUD9600)) {
			Toast.makeText(this, "Arduino connected", Toast.LENGTH_SHORT)
					.show();
		} else {
			Toast.makeText(this,
					"Andruino error: Arduino not connect to Android",
					Toast.LENGTH_SHORT).show();
			onDestroy();
		}

	}

	@Override
	public void onStop() {
		super.onStop();

	}

	@Override
	public void onDestroy() {

		// [FTDriver] Close USB Serial
		mSerial.end();

		super.onDestroy();
		finishAffinity();
		android.os.Process.killProcess(android.os.Process.myPid());
		System.exit(0);

	}

	// Camaras
	@SuppressLint("ShowToast")
	@Override
	public boolean onTouchEvent(MotionEvent event) {
		if (event.getAction() == MotionEvent.ACTION_UP) {
			int numberOfCameras = Camera.getNumberOfCameras();
			final Toast toast;
			if (numberOfCameras > 1) {
				cameraId = (cameraId + 1) % numberOfCameras;
				rosCameraPreviewView.releaseCamera();
				rosCameraPreviewView.setCamera(Camera.open(cameraId));
				toast = Toast.makeText(this, "Switching cameras.",
						Toast.LENGTH_SHORT);
			} else {
				toast = Toast.makeText(this,
						"No alternative cameras to switch to.",
						Toast.LENGTH_SHORT);
			}
			runOnUiThread(new Runnable() {
				@Override
				public void run() {
					toast.show();
				}
			});
		}
		return true;
	}

	@Override
	protected void init(NodeMainExecutor nodeMainExecutor) {

		// Crea el Listener
		my_andruinoROS_command_sub = new andruinoROS_command_sub();
		// Crea el talker
		my_andruinoROS_azimut_pub = new andruinoROS_azimut_pub(this);
		// Crea el talker
		my_andruinoROS_sensor_distancia_pub = new andruinoROS_sensor_distancia_pub();

		// CAMERA
		cameraId = 0;
		rosCameraPreviewView.setCamera(Camera.open(cameraId));
		NodeConfiguration nodeConfiguration5 = NodeConfiguration
				.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
		nodeConfiguration5.setMasterUri(getMasterUri());
		nodeConfiguration5.setNodeName("andruino_driver_camera");
		nodeMainExecutor.execute(rosCameraPreviewView, nodeConfiguration5);

		NodeConfiguration nodeConfiguration = NodeConfiguration
				.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
		nodeConfiguration.setMasterUri(getMasterUri());

		// Azimut
		nodeConfiguration.setNodeName("andruino_driver_azimut");
		nodeMainExecutor.execute(my_andruinoROS_azimut_pub, nodeConfiguration);

		// Comandos
		nodeConfiguration.setNodeName("andruino_driver_command");
		nodeMainExecutor.execute(my_andruinoROS_command_sub, nodeConfiguration);

		// Distancia y edidas del arduino
		nodeConfiguration.setNodeName("andruino_driver_distance");
		nodeMainExecutor.execute(my_andruinoROS_sensor_distancia_pub,
				nodeConfiguration);

		// GPS from android sensor
		/*
		NodeConfiguration nodeConfiguration2 = NodeConfiguration
				.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
		nodeConfiguration2.setMasterUri(getMasterUri());
		nodeConfiguration2.setNodeName("android_driver_nav_sat_fix");
		this.fix_pub = new NavSatFixPublisher(mLocationManager);
		nodeMainExecutor.execute(this.fix_pub, nodeConfiguration2);
		*/

		// IMU from android sensor
		NodeConfiguration nodeConfiguration3 = NodeConfiguration
				.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
		nodeConfiguration3.setMasterUri(getMasterUri());

		nodeConfiguration3.setNodeName("andruino_driver_imu");
		this.imu_pub = new ImuPublisher(mSensorManager);
		nodeMainExecutor.execute(this.imu_pub, nodeConfiguration3);

		// Wifi beacon sensor
		NodeConfiguration nodeConfiguration4 = NodeConfiguration
				.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
		nodeConfiguration4.setMasterUri(getMasterUri());
		nodeConfiguration4.setNodeName("andruino_driver_levels");
		my_andruinoROS_sensor_wifi_pub = new andruinoROS_sensor_Wifi(this);
		nodeMainExecutor.execute(my_andruinoROS_sensor_wifi_pub,
				nodeConfiguration4);

	}

}
