import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.Navigator;

public class Motors {
	//Pilot required values
	final static float WHEEL_DIAMETER = 56;
    final static float AXEL_LENGTH = 170;
    final static float ANGULAR_SPEED = 50;
    final static float LINEAR_SPEED = 200;
	
    private BaseRegulatedMotor mLeft;
    private BaseRegulatedMotor mRight;
    private MovePilot plt;
    private Navigator nav;
    
    
    
    
    public Motors() {
    mLeft = new EV3LargeRegulatedMotor(MotorPort.A);
    mRight = new EV3LargeRegulatedMotor(MotorPort.B);
    mLeft.setSpeed(720); // 2 Revolutions Per Second ( RPS )
    mRight.setSpeed(720);
    
    Wheel wLeft = WheeledChassis.modelWheel(mLeft, WHEEL_DIAMETER).offset(-AXEL_LENGTH / 2);
    Wheel wRight = WheeledChassis.modelWheel(mRight, WHEEL_DIAMETER).offset(AXEL_LENGTH / 2);
    Chassis chassis = new WheeledChassis(new Wheel[] {wRight, wLeft}, WheeledChassis.TYPE_DIFFERENTIAL);
    
    plt = new MovePilot(chassis);
    plt.setLinearSpeed(LINEAR_SPEED);
    PoseProvider poseProvider = new OdometryPoseProvider(plt);

    nav = new Navigator ( plt , poseProvider );
    
    }
    
    public void setSpeed(int speed) {
    	mLeft.setSpeed(speed);
    	mRight.setSpeed(speed);
    }
    
    
   public int getSpeed() { return mLeft.getSpeed();}
   public int getTacho() {return mLeft.getTachoCount();}
   
   public BaseRegulatedMotor getLeftMotor() {return mLeft;};
   public BaseRegulatedMotor getRightMotor() {return mRight;};
   
   
   public MovePilot getPilot() {return plt;}
   public Navigator getNavigator() {return nav;}
   
   //Note : general use, call a new "Motor" class as a Motor.
   //		use getters to return motors and pilot+navigator
   //		optional to get speed and tacho
   //		only setter is for speed of both motors

}