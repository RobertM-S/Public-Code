import java.io.BufferedInputStream;
import java.io.BufferedWriter;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketAddress;

import lejos.hardware.lcd.LCD;

public class AndroidInterface {
	private static String IPaddress = "10.0.1.5";
	private static int port = 1234;
	public static Socket connection = new Socket();
	public static DataInputStream dis;
	public static DataOutputStream dos;
	private static int MAX_READ = 50;
	private static BufferedInputStream in = null;
	private static BufferedWriter out = null;

	public AndroidInterface() throws IOException {
		SocketAddress sa = new InetSocketAddress(IPaddress, port);
		try {
			connection.connect(sa, 1500); 
		} catch (Exception ex) {
			LCD.drawString(ex.getMessage(), 0,6);
			connection = null;
		}
		if (connection != null) {
			in = new BufferedInputStream( connection.getInputStream());
			out = new BufferedWriter(new OutputStreamWriter(connection.getOutputStream()));
		}
	}
	public String getCoords() throws IOException, InterruptedException {
		if (connection != null) {
			byte[] buffer = new byte[MAX_READ];
			String Data = "";
			out.write("Request");
			out.flush();
			Thread.sleep(1000);
			if (in.available() > 0) {
				int read = in.read(buffer, 0, MAX_READ);
				for (int index= 0 ; index < read ; index++) {						
					Data += (char)buffer[index];
				}
			}
			return Data;	
		}
		else {
			return null;
		}
		
	}
	public String getDirectionG() throws IOException, InterruptedException {
		String input = getCoords();
		if(input != null) {
			int colonPos = input.indexOf(";");
			String data = input.substring(3,colonPos);
			
			if(data.equals("Null")) {
				return "NV";
			}
			else {
				String[] split = data.split(",");
				float xCord = Float.parseFloat(split[1]);
				System.out.println("X: "+xCord);
				if(xCord > 420) {
					return "right";
				}
				else if(xCord < 280) {
					return "left";
				}
				else {
					return null;
				}
			}
			
		}
		else {
			return "err";
		}
	}
	public String getDirectionR() {
		try {
		String input = getCoords();
		System.out.println("\n\n");
		if(input != null) {
			int colonPos = input.indexOf(";");
			String data = input.substring(colonPos+1);
			colonPos = data.indexOf(";");
			data = data.substring(2,colonPos);
			
			if(data.equals("Null")) {
				return "NV";
			}
			else {
				String[] split = data.split(",");
				float xCord = Float.parseFloat(split[1]);
				System.out.println("X: "+xCord);
				if(xCord > 420) {
					return "right";
				}
				else if(xCord < 280) {
					return "left";
				}
				else {
					return null;
				}
			}
			
		}
		else {
			return "Null-Err";
		}
		} catch (Exception e) {
			return("err");
		}
	}
	
}
	



