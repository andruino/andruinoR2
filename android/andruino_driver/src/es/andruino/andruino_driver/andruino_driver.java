/*
Andruino R2. A DIY open ROS robot, iot compatible
Andruino comes with ABSOLUTELY NO WARRANTY, to the extent permitted by applicable law
Versión: 99 Date: 2017/04/19
Author: @andruinos
Based on previous works of:  
Damon Kohler (@damonkohler), Keisuke SUZUKI (@ksksue), Chad Rockey (@chadrockey), Jonathan Cohen (fritzing),
Brian Gerkey and others (if i forget someone, please tell me) and ROS community.
CC License Attribution-NonCommercial-ShareAlike 3.0 
https://creativecommons.org/licenses/by-nc-sa/3.0/
 */

package es.andruino.andruino_driver;

import java.io.IOException;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.util.Enumeration;
import java.util.Locale;

import jp.ksksue.driver.serial.FTDriver;

import org.apache.http.conn.util.InetAddressUtils;
import org.ros.android.RosActivity;
import org.ros.android.view.camera.RosCameraPreviewView;
import org.ros.namespace.NameResolver;
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
import android.speech.tts.TextToSpeech;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.Toast;

public class andruino_driver extends RosActivity {

	private static SensorManager mSensorManager;

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
	private andruinoROS_cmd_vel my_andruinoROS_cmd_vel;
	private andruinoROS_odom_pub my_andruinoROS_odom_pub;

	// Nodos de android_driver
	// private NavSatFixPublisher fix_pub;
	private ImuPublisher imu_pub;

	// TTS
	public static TextToSpeech t1;

	// Variables "globales"
	public static float gAzimut; // Azimut Yaw
	public static float gOmega; // Velocidad angular
	public static float gx;
	public static float gy;
	public static float gAzimut_odom;
	public static int gEstado;

	public static String prefijoNS = "";
	public static int contador = 0;

	public andruino_driver() {
		super("andruino_driver", "andruino_driver");
	}

	public void onCreate(Bundle savedInstanceState) {

		super.onCreate(savedInstanceState);

		// mLocationManager = (LocationManager)
		// this.getSystemService(Context.LOCATION_SERVICE);
		mSensorManager = (SensorManager) this.getSystemService(SENSOR_SERVICE);

		// Quita el título de la aplicación y maximize la pantalla
		requestWindowFeature(Window.FEATURE_NO_TITLE);
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

		// TTS
		t1 = new TextToSpeech(getApplicationContext(),
				new TextToSpeech.OnInitListener() {
					@Override
					public void onInit(int status) {
						if (status != TextToSpeech.ERROR) {
							t1.setLanguage(Locale.US);
							t1.speak(
									"Greetings professor Lopez. Andruino is ready.",
									TextToSpeech.QUEUE_FLUSH, null);
						} else {
							Toast.makeText(getApplicationContext(),
									"Andruino error: No voice",
									Toast.LENGTH_SHORT).show();
						}
					}
				});
		//

		// La pantalla de Android muestra la cámara
		rosCameraPreviewView = (RosCameraPreviewView) findViewById(R.id.ros_camera_preview_view);

		// Para que no sea visible en la pantalla, tamaño 1 pixel x 1 pixel
		// &rosCameraPreviewView.getLayoutParams().width = 1;
		// &rosCameraPreviewView.getLayoutParams().height = 1;

		// USB Serial. FTDriver] Crea Instancia
		mSerial = new FTDriver(
				(UsbManager) getSystemService(Context.USB_SERVICE));

		// FTDriver
		PendingIntent permissionIntent = PendingIntent.getBroadcast(this, 0,
				new Intent(ACTION_USB_PERMISSION), 0);
		mSerial.setPermissionIntent(permissionIntent);

		// Abre el puerto Serie a 11520
		if (mSerial.begin(FTDriver.BAUD115200)) {
			Toast.makeText(this, "Arduino connected", Toast.LENGTH_SHORT)
					.show();

		} else {
			Toast.makeText(this,
					"Andruino error: Arduino not connect to Android",
					Toast.LENGTH_SHORT).show();
			try {
				Thread.sleep(5000);
			} catch (InterruptedException e) {

			}
			onDestroy();
		}

		// Inicia las variables globlales
		gx = 0f;
		gy = 0f;
		gAzimut = 0f;
		gOmega = 0f;
		gEstado = 0;
		gAzimut_odom = 0f;

	}

	@Override
	public void onStop() {
		super.onStop();

	}

	@Override
	public void onDestroy() {

		// [FTDriver] Close USB Serial
		mSerial.end();

		// Falta Desregistar todos los sensores

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
		my_andruinoROS_cmd_vel = new andruinoROS_cmd_vel();
		// Crea el talker
		my_andruinoROS_odom_pub = new andruinoROS_odom_pub();

		String acadenaAux = "";

		// CAMERA

		try {
			java.net.Socket socket = new java.net.Socket(getMasterUri()
					.getHost(), getMasterUri().getPort());
			java.net.InetAddress local_network_address = socket
					.getLocalAddress();

			acadenaAux = getLocalIpAddress();

			String[] trozos = acadenaAux.split("\\.");
			if (trozos[3].length() > 0)
				prefijoNS = "/andruino" + trozos[3].trim();
			else
				prefijoNS = "/andruino";

			socket.close();

			NodeConfiguration nodeConfiguration5 = NodeConfiguration.newPublic(
					local_network_address.getHostAddress(), getMasterUri());

			NameResolver res = NameResolver.newFromNamespace(prefijoNS);
			nodeConfiguration5.setParentResolver(res);

			cameraId = 0;
			rosCameraPreviewView.setCamera(Camera.open(cameraId));

			nodeConfiguration5.setMasterUri(getMasterUri());
			nodeConfiguration5.setNodeName("andruino_driver_camera");
			nodeMainExecutor.execute(rosCameraPreviewView, nodeConfiguration5);

			NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(
					local_network_address.getHostAddress(), getMasterUri());
			nodeConfiguration.setParentResolver(res);
			nodeConfiguration.setMasterUri(getMasterUri());

			// Azimut

			nodeConfiguration.setNodeName("andruino_driver_azimut");
			nodeMainExecutor.execute(my_andruinoROS_azimut_pub,
					nodeConfiguration);

			NodeConfiguration nodeConfiguration10 = NodeConfiguration
					.newPublic(local_network_address.getHostAddress(),
							getMasterUri());
			nodeConfiguration10.setMasterUri(getMasterUri());
			nodeConfiguration10.setParentResolver(res);

			// Comandos
			nodeConfiguration10.setNodeName("andruino_driver_command");
			nodeMainExecutor.execute(my_andruinoROS_command_sub,
					nodeConfiguration10);

			NodeConfiguration nodeConfiguration11 = NodeConfiguration
					.newPublic(local_network_address.getHostAddress(),
							getMasterUri());
			nodeConfiguration11.setParentResolver(res);
			nodeConfiguration11.setMasterUri(getMasterUri());

			// Distancia y edidas del arduino
			nodeConfiguration11.setNodeName("andruino_driver_distance");
			nodeMainExecutor.execute(my_andruinoROS_sensor_distancia_pub,
					nodeConfiguration11);

			// GPS from android sensor
			/*
			 * //NodeConfiguration nodeConfiguration2 = NodeConfiguration
			 * .newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
			 * NodeConfiguration nodeConfiguration2 =
			 * NodeConfiguration.newPublic
			 * (local_network_address.getHostAddress(), getMasterUri());
			 * nodeConfiguration2.setMasterUri(getMasterUri());
			 * nodeConfiguration2.setNodeName("andruino_driver_nav_sat_fix");
			 * this.fix_pub = new NavSatFixPublisher(mLocationManager);
			 * nodeMainExecutor.execute(this.fix_pub, nodeConfiguration2);
			 */

			// IMU from android sensor

			NodeConfiguration nodeConfiguration3 = NodeConfiguration.newPublic(
					local_network_address.getHostAddress(), getMasterUri());
			nodeConfiguration3.setParentResolver(res);
			nodeConfiguration3.setMasterUri(getMasterUri());
			nodeConfiguration3.setNodeName("andruino_driver_imu");
			this.imu_pub = new ImuPublisher(mSensorManager);
			nodeMainExecutor.execute(this.imu_pub, nodeConfiguration3);

			// Wifi beacon sensor

			NodeConfiguration nodeConfiguration4 = NodeConfiguration.newPublic(
					local_network_address.getHostAddress(), getMasterUri());
			nodeConfiguration4.setParentResolver(res);
			nodeConfiguration4.setMasterUri(getMasterUri());
			nodeConfiguration4.setNodeName("andruino_driver_levels");
			my_andruinoROS_sensor_wifi_pub = new andruinoROS_sensor_Wifi(this);
			nodeMainExecutor.execute(my_andruinoROS_sensor_wifi_pub,
					nodeConfiguration4);

			NodeConfiguration nodeConfiguration12 = NodeConfiguration
					.newPublic(local_network_address.getHostAddress(),
							getMasterUri());
			nodeConfiguration12.setParentResolver(res);
			nodeConfiguration12.setMasterUri(getMasterUri());

			// Comandos
			nodeConfiguration12.setNodeName("andruino_driver_cmd_vel");
			nodeMainExecutor.execute(my_andruinoROS_cmd_vel,
					nodeConfiguration12);

			// Odom
			NodeConfiguration nodeConfiguration13 = NodeConfiguration
					.newPublic(local_network_address.getHostAddress(),
							getMasterUri());
			nodeConfiguration13.setParentResolver(res);
			nodeConfiguration13.setMasterUri(getMasterUri());
			nodeConfiguration13.setNodeName("andruino_driver_odom");
			nodeMainExecutor.execute(my_andruinoROS_odom_pub,
					nodeConfiguration13);

		} catch (IOException e) {
			// Socket problem
		}

	}

	public float getgAzimut() {
		return gAzimut;
	}

	public void setgAzimut(float gAzimut) {
		this.gAzimut = gAzimut;
	}

	public float getgOmega() {
		return gOmega;
	}

	public void setgOmega(float gOmega) {
		this.gOmega = gOmega;
	}

	public float getGx() {
		return gx;
	}

	public static void setGx(float gx) {
		andruino_driver.gx = gx;
	}

	public float getGy() {
		return gy;
	}

	public void setGy(float gy) {
		this.gy = gy;
	}

	public int getgEstado() {
		return gEstado;
	}

	public void setgEstado(int gEstado) {
		this.gEstado = gEstado;
	}

	public String getLocalIpAddress() {
		// From http://stackoverflow.com/questions/11015912/how-do-i-get-ip-address-in-ipv4-format
		try {
			for (Enumeration<NetworkInterface> en = NetworkInterface
					.getNetworkInterfaces(); en.hasMoreElements();) {
				NetworkInterface intf = en.nextElement();
				for (Enumeration<InetAddress> enumIpAddr = intf
						.getInetAddresses(); enumIpAddr.hasMoreElements();) {
					InetAddress inetAddress = enumIpAddr.nextElement();

					String ipv4;
					if (!inetAddress.isLoopbackAddress()
							&& InetAddressUtils
									.isIPv4Address(ipv4 = inetAddress
											.getHostAddress())) {

						String ip = inetAddress.getHostAddress().toString();
						return ip;
					}
				}
			}
		} catch (Exception ex) {
			Log.e("IP Address", ex.toString());
		}
		return null;
	}

}
