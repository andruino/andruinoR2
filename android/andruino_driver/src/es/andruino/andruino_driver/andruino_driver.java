package es.andruino.andruino_driver;

import java.io.IOException;

import jp.ksksue.driver.serial.FTDriver;

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
import android.os.Bundle;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.Toast;

//import android.location.LocationManager;

public class andruino_driver extends RosActivity {

	private static SensorManager mSensorManager;
	// private static LocationManager mLocationManager;

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
	// private andruinoROS_sensor_Wifi my_andruinoROS_sensor_wifi_pub;
	private andruinoROS_cmd_vel my_andruinoROS_cmd_vel;
	private andruinoROS_odom_pub my_andruinoROS_odom_pub;

	// Nodos de android_driver
	// private NavSatFixPublisher fix_pub;
	private ImuPublisher imu_pub;
	
	//Variables "globales"
	static int estado;
	public static float gAzimut; //Azimut Yaw
	public static float gOmega; //Velocidad angular
	public static int HCSR04_1;
	public static int HCSR04_2;
	public static int HCSR04_3;
	public static int LDR1;
	public static int LDR2;
	public static int LDR3;
	public static int dtLoop;
	public static float distancia;
	public static float x;
	public static float y;
	
	

	public andruino_driver() {
		super("andruino_driver", "andruino_driver");
	}

	public void onCreate(Bundle savedInstanceState) {

		super.onCreate(savedInstanceState);

		// mLocationManager = (LocationManager)
		// this.getSystemService(Context.LOCATION_SERVICE);
		mSensorManager = (SensorManager) this.getSystemService(SENSOR_SERVICE);

		// Quita el título de la aplicación y máximiza la pantalla
		// requestWindowFeature(Window.FEATURE_NO_TITLE);
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);

		// Carga el xml
		setContentView(R.layout.main);

		final Button buttonapagar = (Button) findViewById(R.id.buttonapagar);
		buttonapagar.setOnClickListener(new View.OnClickListener() {
			public void onClick(View v) {
				finishAffinity();
				android.os.Process.killProcess(android.os.Process.myPid());
				System.exit(0);
			}
		});
		// La pantalla de Android muestra la cámara
		rosCameraPreviewView = (RosCameraPreviewView) findViewById(R.id.ros_camera_preview_view);
		// Para que no sea visible en la pantalla, tamaño 1 pixel x 1 pixel
		//&rosCameraPreviewView.getLayoutParams().width = 1;
		//&rosCameraPreviewView.getLayoutParams().height = 1; //

		// USB Serial. FTDriver] Crea Instancia
		mSerial = new FTDriver(
				(UsbManager) getSystemService(Context.USB_SERVICE));

		// [FTDriver] setPermissionIntent() before begin()
		PendingIntent permissionIntent = PendingIntent.getBroadcast(this, 0,
				new Intent(ACTION_USB_PERMISSION), 0);
		mSerial.setPermissionIntent(permissionIntent);

		// Abre el puerto Serie a 11520. SUBIR !!!!!!! //

		if (mSerial.begin(FTDriver.BAUD115200)) {
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

		// Desregistar todos los sensores FALTA!!!!!!!!!!!!!!!!!!!

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
		// Creal el Listener
		my_andruinoROS_cmd_vel= new andruinoROS_cmd_vel();
		// Crea el talker
		my_andruinoROS_odom_pub=new andruinoROS_odom_pub();
		
		
		//Inicia variables globales
		estado=0;
		gAzimut=0; //Azimut Yaw
		gOmega=0; //Velocidad angular
		HCSR04_1=0;
		HCSR04_2=0;
		HCSR04_3=0	;
		LDR1=0;
		LDR2=0;
		LDR3=0;
		dtLoop=0;
		distancia=0;
		x=0;
		y=0;
		//

		// CAMERA

		try {
			java.net.Socket socket = new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
            java.net.InetAddress local_network_address = socket.getLocalAddress();
            socket.close();
            NodeConfiguration nodeConfiguration5 =
                    NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
            // NodeConfiguration nodeConfiguration5 =NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
    		// https://github.com/rosjava/android_core/issues/147 !!!!
            // https://github.com/rosjava/android_apps/blob/indigo/teleop/src/main/java/com/github/rosjava/android_apps/teleop/MainActivity.java#L76-L80
            
		cameraId = 0;
		rosCameraPreviewView.setCamera(Camera.open(cameraId));
		


			//NodeConfiguration nodeConfiguration5 = NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
			
			nodeConfiguration5.setMasterUri(getMasterUri());
			nodeConfiguration5.setNodeName("andruino_driver_camera");
			nodeMainExecutor.execute(rosCameraPreviewView, nodeConfiguration5);
			
		
			//NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
			NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
			nodeConfiguration.setMasterUri(getMasterUri());

			// Azimut
		
			nodeConfiguration.setNodeName("andruino_driver_azimut");
			nodeMainExecutor.execute(my_andruinoROS_azimut_pub,
					nodeConfiguration);
		
			//NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
			NodeConfiguration nodeConfiguration10 = NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
			nodeConfiguration10.setMasterUri(getMasterUri());

			// Comandos
			nodeConfiguration10.setNodeName("andruino_driver_command");
			nodeMainExecutor.execute(my_andruinoROS_command_sub,
					nodeConfiguration10);
			
			//NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
			NodeConfiguration nodeConfiguration11 = NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
			nodeConfiguration11.setMasterUri(getMasterUri());

			// Distancia y edidas del arduino
			nodeConfiguration11.setNodeName("andruino_driver_distance");
			nodeMainExecutor.execute(my_andruinoROS_sensor_distancia_pub,
					nodeConfiguration11);

			// GPS from android sensor
			/*
			 * //NodeConfiguration nodeConfiguration2 = NodeConfiguration
			 * .newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
			 * 	NodeConfiguration nodeConfiguration2 = NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
			 * nodeConfiguration2.setMasterUri(getMasterUri());
			 * nodeConfiguration2.setNodeName("android_driver_nav_sat_fix");
			 * this.fix_pub = new NavSatFixPublisher(mLocationManager);
			 * nodeMainExecutor.execute(this.fix_pub, nodeConfiguration2);
			 */

			// IMU from android sensor
			
			 //NodeConfiguration nodeConfiguration3 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
			 NodeConfiguration nodeConfiguration3 = NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
			 nodeConfiguration3.setMasterUri(getMasterUri());
			  
			 nodeConfiguration3.setNodeName("andruino_driver_imu");
			 this.imu_pub = new ImuPublisher(mSensorManager);
			 nodeMainExecutor.execute(this.imu_pub, nodeConfiguration3);
			 

			// Wifi beacon sensor
			/*
			 * NodeConfiguration nodeConfiguration4 = NodeConfiguration
			 * .newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
			 * nodeConfiguration4.setMasterUri(getMasterUri());
			 * nodeConfiguration4.setNodeName("andruino_driver_levels");
			 * my_andruinoROS_sensor_wifi_pub = new
			 * andruinoROS_sensor_Wifi(this);
			 * nodeMainExecutor.execute(my_andruinoROS_sensor_wifi_pub,
			 * nodeConfiguration4);
			 */
			
			//NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
			NodeConfiguration nodeConfiguration12 = NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
			nodeConfiguration12.setMasterUri(getMasterUri());

			// Comandos
			nodeConfiguration12.setNodeName("andruino_driver_cmd_vel");
			nodeMainExecutor.execute(my_andruinoROS_cmd_vel,nodeConfiguration12);

			//Odom
			NodeConfiguration nodeConfiguration13 = NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
			nodeConfiguration13.setMasterUri(getMasterUri());

			nodeConfiguration13.setNodeName("andruino_driver_odom");
			nodeMainExecutor.execute(my_andruinoROS_odom_pub,nodeConfiguration13);

			
		} catch (IOException e) {
            // Socket problem
        }

	}

	public static int getEstado() {
		return estado;
	}

	public static void setEstado(int estado) {
		andruino_driver.estado = estado;
	}

	public static float getgAzimut() {
		return gAzimut;
	}

	public static void setgAzimut(float gAzimut) {
		andruino_driver.gAzimut = gAzimut;
	}

	public static float getgOmega() {
		return gOmega;
	}

	public static void setgOmega(float gOmega) {
		andruino_driver.gOmega = gOmega;
	}

	public static int getHCSR04_1() {
		return HCSR04_1;
	}

	public static void setHCSR04_1(int hCSR04_1) {
		HCSR04_1 = hCSR04_1;
	}

	public static int getHCSR04_2() {
		return HCSR04_2;
	}

	public static void setHCSR04_2(int hCSR04_2) {
		HCSR04_2 = hCSR04_2;
	}

	public static int getHCSR04_3() {
		return HCSR04_3;
	}

	public static void setHCSR04_3(int hCSR04_3) {
		HCSR04_3 = hCSR04_3;
	}

	public static int getLDR1() {
		return LDR1;
	}

	public static void setLDR1(int lDR1) {
		LDR1 = lDR1;
	}

	public static int getLDR2() {
		return LDR2;
	}

	public static void setLDR2(int lDR2) {
		LDR2 = lDR2;
	}

	public static int getLDR3() {
		return LDR3;
	}

	public static void setLDR3(int lDR3) {
		LDR3 = lDR3;
	}

	public static int getDtLoop() {
		return dtLoop;
	}

	public static void setDtLoop(int dtLoop) {
		andruino_driver.dtLoop = dtLoop;
	}

	public static float getDistancia() {
		return distancia;
	}

	public static void setDistancia(float distancia) {
		andruino_driver.distancia = distancia;
	}

	public static float getX() {
		return x;
	}

	public static void setX(float x) {
		andruino_driver.x = x;
	}

	public static float getY() {
		return y;
	}

	public static void setY(float y) {
		andruino_driver.y = y;
	}

}
