import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.subsumption.*;
import lejos.utility.Delay;
//import Motors;

public class Pickup implements Behavior {
	private AndroidInterface AI;
	private MovePilot pilot;
	private boolean gotBlock = false;
	private String direction;
	public Pickup(MovePilot p,  AndroidInterface colour){
		this.pilot = p;
		this.AI = colour;
	}
	
public boolean takeControl() {
	direction = AI.getDirectionG();
	return (direction != null);
   }

   public void suppress() {}

   public void action() {
	   
	   //	=	code added
	   float[] = colourCode = new float[1];//
	   code.fetchSample(colourCode, 0);//
	   
	   if (direction.equals("left")) {
		   pilot.rotate(5);
		   try{Thread.sleep(1000);}catch(Exception e) {}
	   }
	   else if (direction.equals("right")) {
		   pilot.rotate(-5);
		   try{Thread.sleep(1000);}catch(Exception e) {}
	   }
	   else if (direction.equals("err")) {
		   System.out.println("error");
	   }
	   
	   else {
		   if (colourCode[0] == 1) {//
			   mLeft.startSynchronization();//
			   mLeft.forward();//
			   mRight.forward();//
			   Delay.msDelay(10);//
			   mLeft.stopSynchronization();//
			   
			   //Code for closing the claw
			   claw.setSpeed(300);
			   claw.backward();
			   Delay.msDelay(3000);
			   claw.stop();
			   
			   //Code for opening the claw
			   //claw.setSpeed(300);
			   //claw.forward();
			   //Delay.msDelay(3000);
			   //claw.stop();
			   
		   }
			   
	   }
		   
	   try{Thread.sleep(1000);}catch(Exception e) {}
   }
}
