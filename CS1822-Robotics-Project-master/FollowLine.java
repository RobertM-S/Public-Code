package worksheets;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.HiTechnicColorSensor;
import lejos.robotics.SampleProvider;

public class FollowLine {
	final static int Two_Revolutions_Per_Second = 180;

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		
		
		float[] colourCode = new float[1];
		Button.LEDPattern(4);
		LCD.clear();
		LCD.drawString("Ready",  2 , 2);
		Button.ENTER.waitForPressAndRelease();
		LCD.clear();
		BaseRegulatedMotor mLeft = new EV3LargeRegulatedMotor(MotorPort.A);
		BaseRegulatedMotor mRight = new EV3LargeRegulatedMotor(MotorPort.B);
		
		HiTechnicColorSensor colorSensor = new HiTechnicColorSensor(SensorPort.S1);
		SampleProvider code = colorSensor.getColorIDMode();
		
		mLeft.setSpeed(Two_Revolutions_Per_Second); // 2 Revolutions Per Second ( RPS )
		mRight.setSpeed(Two_Revolutions_Per_Second);
		mLeft.synchronizeWith ( new BaseRegulatedMotor [] { mRight });
		while(true) {
			code.fetchSample(colourCode, 0);
			if(Button.ENTER.isDown()) {
				mLeft.startSynchronization();
				mLeft.stop();
				mRight.stop();
				mLeft.endSynchronization();
		
				mLeft.close();
				mRight.close();
				colorSensor.close();
				break;
			}
			while (colourCode[0] == 7)
			{
				mLeft.startSynchronization();
				mLeft.backward();
				mRight.stop();
				mLeft.endSynchronization();
				code.fetchSample(colourCode, 0);
			}
			
			while (colourCode[0] != 7)
			{
				mLeft.startSynchronization();
				mLeft.stop();
				mRight.backward();
				mLeft.endSynchronization();
				code.fetchSample(colourCode, 0);
			}	
		}
	}

}
