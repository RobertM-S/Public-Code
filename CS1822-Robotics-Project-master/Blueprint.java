import lejos.robotics.navigation.MovePilot;
import lejos.robotics.subsumption.*;
//import Motors;

public class Blueprint implements Behavior {
	
	private MovePilot example;
	
	public Blueprint(/*MovePilot example*/){
	//Constructor used to import. for example from the motors and sensor class
	//this.example = example
	}
	
	public boolean takeControl() {
	// what is required for this method to take control
	// moving forward is something the robot will do regardless so it will always return true
	// return (about to hit wall);
      return true;
    }

    public void suppress() {}
    // this method is required if the method must be stopped to allow another to take over
    // this method is not need unless the action method takes a long time to finish
    // e.g. the method that turns the robot to look for a colour may take a long time
    // in that case set a suppressed variable as true in this method
    // and false in the action method which will cause the method to cede control 
    // if suppress is ever called

    public void action() {
	// this is the method where the robot executes orders
	// e.g. example.forward() might make the robot go forward until something
	// else takes control
   }
}