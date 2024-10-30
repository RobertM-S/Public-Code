import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;

public class Behaviour {
	
	public static void main(String[] args) {
		
		
		 Motors motors = new Motors();
		 MovePilot plt = motors.getPilot();
		 // import from the motor and sensor class here
		 
		 
	     Behavior Trundle = new Trundle(plt);
	     Behavior Backup = new Backup(plt);
	     // Behaviour example = new MethodName(pass to the constructor here);
	     
	     
	     Behavior [] bArray = {Trundle, Backup};
	     // Add behaviours to the array
	     // They are ordered in increasing priority
	     // E.g. b2 has a higher priority than b1
	     
	     
	     Arbitrator arby = new Arbitrator(bArray);
		 Button.LEDPattern(4);
		 LCD.clear();
		 LCD.drawString("Ready",  2 , 2);
		 Button.ENTER.waitForPressAndRelease();
		 LCD.clear();
	     arby.go();
	}
}
