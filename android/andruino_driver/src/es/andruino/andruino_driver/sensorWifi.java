package es.andruino.andruino_driver;

import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

import org.apache.commons.math3.stat.correlation.PearsonsCorrelation;
import org.ros.message.Time;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.net.wifi.ScanResult;
import android.net.wifi.WifiManager;

class sensorWifi extends BroadcastReceiver {

	WifiManager Wifimgr;
	List<ScanResult> wifiList;

	StringBuilder sb = new StringBuilder();
	Context contexto;
	double corr;

	int contador = 0;

	HashMap<String, Integer> AP2Power = new HashMap<String, Integer>(); // Punto
																		// Actual
	HashMap<String, Integer> AP2PowerOrigen = new HashMap<String, Integer>(); // Punto//
																				// Origen

	int MinPower = 0;

	boolean guardaOrigen; // flag que indica si debe guardar el origen.

	public sensorWifi(Context contexto) {
		Wifimgr = (WifiManager) contexto.getSystemService(Context.WIFI_SERVICE);
		contexto.registerReceiver(this, new IntentFilter(
				WifiManager.SCAN_RESULTS_AVAILABLE_ACTION));
		Wifimgr.startScan();
		// guardaOrigen = true;
		AP2Power.clear();
		AP2PowerOrigen.clear();
	}

	// Si se invoca esta funcion se redefine el origen
	public void defineOrigin() {

		// AP2PowerOrigen = (HashMap<String, Integer>) AP2Power.clone();
		String SSID;
		MinPower = 0;
		AP2PowerOrigen.clear();
		Iterator<String> ListaSSID = AP2Power.keySet().iterator();
		while (ListaSSID.hasNext()) {
			SSID = ListaSSID.next();
			AP2PowerOrigen.put(SSID, AP2Power.get(SSID));
		}
	}

	@Override
	public void onReceive(Context c, Intent intent) {

		sb = new StringBuilder();
		wifiList = Wifimgr.getScanResults();

		// Borrar el hashMap
		AP2Power.clear();

		contador++;
		
		int coindicencia_ap=0;

		for (int i = 0; i < wifiList.size(); i++) {

			// En lugar de llevar todos los datos del wifiList sólo llevo BSSID
			// y los dBm
			AP2Power.put((wifiList.get(i)).BSSID, (wifiList.get(i)).level);
			/*
			 * sb.append((wifiList.get(i)).BSSID); sb.append(";");
			 * sb.append(AP2Power.get((wifiList.get(i)).BSSID));
			 * sb.append("\n");
			 */

		}

		// Depuración si no tiene origen definido lo coge
		// if (AP2PowerOrigen.size() < 5) {
		if (contador < 10) {
			defineOrigin();
		
			
			// AP2PowerOrigen = (HashMap<String, Integer>) AP2Power.clone();

		} else {

			String SSID;
			Iterator<String> ListaSSID = AP2PowerOrigen.keySet().iterator();
			while (ListaSSID.hasNext()) {
				SSID = ListaSSID.next();

				if (MinPower > AP2PowerOrigen.get(SSID))
					MinPower = AP2PowerOrigen.get(SSID);
				/*
				 * sb.append(SSID + ";" +
				 * Integer.toString(AP2PowerOrigen.get(SSID)) + "\n" + "TAMAÑO"
				 * + Integer.toString(AP2PowerOrigen.size()) + "/n");
				 */
			}

			// Bulcle para calcular "distancia" al origen

			int diffPower = 0;

			double x[] = new double[AP2PowerOrigen.size()];
			double y[] = new double[AP2PowerOrigen.size()];
			double z[] = new double[AP2PowerOrigen.size()];

			for (int i = 0; i < AP2PowerOrigen.size(); i++) {
				x[i] = 0.0;
				y[i] = 0.0;
				z[i] = 0.0;
			}
			int i = 0;
			
			coindicencia_ap=0;
			
			for (String SSIDOrigen : AP2PowerOrigen.keySet()) {

				int diffPowerParcial = 0;

				x[i] = (double) AP2PowerOrigen.get(SSIDOrigen);
				z[i] = (double) MinPower - (Math.random());

				if (AP2Power.containsKey(SSIDOrigen)) {
						diffPowerParcial = Math.abs(AP2PowerOrigen
								.get(SSIDOrigen) - AP2Power.get(SSIDOrigen));
						y[i] = (double) AP2Power.get(SSIDOrigen);
						coindicencia_ap++;
					
				} else {
					diffPowerParcial = Math.abs(MinPower)
							- Math.abs(AP2PowerOrigen.get(SSIDOrigen));

					y[i] = (double) MinPower - (Math.random()); // -100.0;
					
				}

				i++;

				if (diffPowerParcial > Math.abs(AP2PowerOrigen.get(SSIDOrigen))/10)//5) // considero 10dBm como variaciones
											// leves.
					diffPower = diffPower + diffPowerParcial;

				/*
				 * sb.append(SSIDOrigen + ";" +
				 * Integer.toString(AP2PowerOrigen.get(SSIDOrigen)) + ";" +
				 * Integer.toString(diffPowerParcial) + " : " +
				 * Integer.toString(diffPower) + "\n");// + ";" + //
				 * Integer.toString(AP2Power.get(SSIDOrigen)) // + " : " );
				 */

			}

			corr = new PearsonsCorrelation().correlation(x, y);

			// Quizas debería medir la ORIGINALIDAD DEL PUNTO,
			// ES DECIR VALORAR LOS NUEVOS PUNTOS QUE NO ESTABAN EN EL ORIGEN
			// PUES PUEDEN DAR MUCHA INFO (O NO......)
			// Seria
			// for (String SSIDOrigen : AP2Power.keySet()) {
			// y viendo los que no están en el origen.....

			// double corrbase = new PearsonsCorrelation().correlation(x, z);

			/*
			 * for (int j = 0; j < AP2PowerOrigen.size(); j++) { sb.append("\n"
			 * + Integer.toString(j) + " " + Double.toString(x[j]) + " " +
			 * Double.toString(y[j]) + " " + Double.toString(z[j]) + " "); }
			 */
			long time_delta_millis = System.currentTimeMillis();

			for (int l = 0; l < wifiList.size(); l++) {
				sb.append(time_delta_millis);
				sb.append(","); // Originalemente eran sb.append(",");
				sb.append((wifiList.get(l)).BSSID);
				sb.append(",");
				sb.append(AP2Power.get((wifiList.get(l)).BSSID));
				sb.append(",");
				sb.append(Double.toString(corr));
				int correlacion = (int) (1000 - (corr * 1000));
				sb.append(",");
				sb.append(Integer.toString(correlacion));
				sb.append(",");
				sb.append(diffPower);
				sb.append(",");
				sb.append(coindicencia_ap);
				sb.append(",");
				sb.append(wifiList.size()-coindicencia_ap);
				sb.append("\n");
			}

		}

		Wifimgr.startScan();

	}

	public String getBeacons() {

		return (sb.toString());
	}

	public int getBeaconsCorr() {
		int correlacion;
		correlacion = (int) (1000 - (corr * 1000));
		return correlacion;
	}

	public void desregistrasensorWifi(Context contexto) {
		contexto.unregisterReceiver(this);
	}

}