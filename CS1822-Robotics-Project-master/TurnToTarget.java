import lejos.robotics.navigation.MovePilot;
import lejos.robotics.subsumption.*;
//import Motors;

public class TurnToTarget implements Behavior {
	private AndroidInterface AI;
	private MovePilot pilot;
	private String direction;
	public TurnToTarget(MovePilot p, AndroidInterface colour){
		this.pilot = p;
		this.AI = colour;

	}
	
public boolean takeControl() {
	System.out.println(direction);
	direction = AI.getDirectionR();
	return (direction != null);
   }

   public void suppress() {
	   pilot.stop();
   }

   public void action() {
	   if (direction.equals("left")) {
		   pilot.rotate(5);
		   try{Thread.sleep(1000);}catch(Exception e) {}
	   }
	   else if (direction.equals("NV")) {
		   pilot.rotate(5);
		   try{Thread.sleep(1000);}catch(Exception e) {}
	   }
	   else if (direction.equals("err")) {
		   System.out.println("error");
	   }
	   else {
		   pilot.rotate(-5);
		   try{Thread.sleep(1000);}catch(Exception e) {}  
	   }
	   
   }
}