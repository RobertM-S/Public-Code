import java.io.BufferedInputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketAddress;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;

public class MainClass {
	private static String IPaddress = "10.0.1.5";
	private static int port = 1234;
	public static Socket connection = new Socket();
	public static DataInputStream dis;
	public static DataOutputStream dos;
	private static int MAX_READ = 50;
	private static BufferedInputStream in = null;
	private static OutputStream out = null;

	public static void main(String[] args) throws IOException {
		byte[] buffer = new byte[MAX_READ];
		//(new TunePlayer()).start();
		
		LCD.drawString("Waiting  ", 0, 0);
		SocketAddress sa = new InetSocketAddress(IPaddress, port);
		try {
			connection.connect(sa, 1500); // Timeout possible
		} catch (Exception ex) {
			// This connection fail is just ignored - we were probably not trying to connect because there was no
			// Android device
			// Could be Timeout or just a normal IO exception
			LCD.drawString(ex.getMessage(), 0,6);
			connection = null;
		}
		if (connection != null) {
			in = new BufferedInputStream( connection.getInputStream());
			out = connection.getOutputStream();
			LCD.drawString("Connected", 0, 0);
		}

		LCD.drawString("Waiting on ", 0, 1);
		LCD.drawString(IPaddress,1,3);
		while (!Button.ESCAPE.isDown()) {
			if (connection != null) {
				if (in.available() > 0) {
					String Data = "";
					LCD.clear();
					LCD.drawString("Reading Data",2,2);
					int read = in.read(buffer, 0, MAX_READ);
					for (int index= 0 ; index < read ; index++) {						
						Data += (char)buffer[index];
					}
					System.out.println(Data);
					out.write("Reply:".getBytes(), 0, 6);
					out.write(buffer, 0, read);
				}
			}
		}
	}
}
