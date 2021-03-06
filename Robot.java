package org.usfirst.frc.team217.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.*;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import java.lang.Math;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	//Joysticks
	Joystick oper, driver;
	Preferences prefs;
	
	//Joystick Variables
	final int buttonSquare = 1;
	final int buttonX = 2;
	final int buttonCircle = 3;
	final int buttonTriangle = 4;
	final int leftBumper = 5;
	final int rightBumper = 6;
	final int leftTrigger = 7;
	final int rightTrigger = 8;
	final int buttonShare = 9;
	final int buttonOption = 10;
	final int leftAnalog = 11;
	final int rightAnalog = 12;
	final int buttonPS = 13;
	final int touchpad = 14;
	
	
	double wheelRPM, hoodValue, flyP,flyI,flyD,flyF,autoForwardLeft,autoForwardRight;
	
	//Drive Motors
	CANTalon rightMaster, rightSlave, leftMaster, leftSlave;
	
	//Shooting Motors
	CANTalon flyWheel, flyWheel2, hood, turret, lifter, kicker, intakeArm, wheelOfDoom;
	
	//Climbing Motors
	CANTalon climber, climber2;
	
	//Solenoids
	Solenoid backSolenoid, frontSolenoid, armSolenoid, gearSolenoid;
	
	//Gyro
	ADXRS450_Gyro horzGyro;
	
	//Compressor
	Compressor compressor;
	
	Preferences pref;
	
	AnalogInput hoodEnc;
	
	
	NetworkTable table;
	
	double intakeSpeed,robotSpeed,visionKP,autoP;
	boolean camNum = false;
	
	enum BallsOut{
		reset,
		forward,
		turn,
		tacticalReload,
	};
	BallsOut ballsOutAuton;
	

	@Override
	public void robotInit() {
		
		pref = Preferences.getInstance();
		
		hoodEnc = new AnalogInput(0);
		
		//Joysticks
		driver = new Joystick(0);
		oper = new Joystick(1);
		
		//Drive Motors
		rightMaster = new CANTalon(15);
		rightMaster.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		
		rightSlave = new CANTalon(14);
		rightSlave.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		rightSlave.changeControlMode(TalonControlMode.Follower);
		rightSlave.set(15);
		
		leftMaster = new CANTalon(0);
		leftMaster.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		
		leftSlave = new CANTalon(1);
		leftSlave.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		leftSlave.changeControlMode(TalonControlMode.Follower);
		leftSlave.set(0);
		
		//Shooting Motors
		flyWheel = new CANTalon(10);
		flyWheel.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		flyWheel.reverseSensor(false);
		flyWheel2 = new CANTalon(11);
		flyWheel2.changeControlMode(TalonControlMode.Follower);
		flyWheel2.set(10);
		
		hood = new CANTalon(9);
		turret= new CANTalon(6);
		lifter = new CANTalon(5);
		kicker = new CANTalon(4);
		intakeArm =  new CANTalon(13);
		wheelOfDoom = new CANTalon(12);
		
		//flyWheel Settings
		flyWheel.configNominalOutputVoltage(+0.0f,  -0.0f);
		flyWheel.configPeakOutputVoltage(+12.0f, -12.0f);
		
		flyWheel.setProfile(0);  
		flyWheel.setP(flyP); //0.1
		flyWheel.setI(flyI); //0.0004895
		flyWheel.setD(flyD); //0.5
		flyWheel.setF(0);
		
		//Climbing Motors
		climber = new CANTalon(2);
		climber2 = new CANTalon(3);
		climber2.changeControlMode(TalonControlMode.Follower);
		climber2.set(2);
		
		
		//Solenoids
		frontSolenoid = new Solenoid(2);
		backSolenoid = new Solenoid(1);
		armSolenoid = new Solenoid(3);
		gearSolenoid = new Solenoid(0);
		
		//Gyro
	    horzGyro = new ADXRS450_Gyro();
	    
	    table = NetworkTable.getTable("SmartDashboard"); 
		table.putBoolean("Camera_Type", camNum);
	   
	    CameraServer.getInstance().startAutomaticCapture();
		
		smartDash();
	}

	@Override
	public void autonomousInit() {
		flyWheel.setEncPosition(0);
		//armSolenoid.set(true); //This deploys the arm at the start of the match.
		frontSolenoid.set(true);//This deploys omni if not already deployed.
		backSolenoid.set(false); // ^
		ballsOutAuton = BallsOut.reset;
	}

	@Override
	public void autonomousPeriodic() {
		/*
		
		switch(ballsOutAuton){
			case reset:
				rightMaster.setEncPosition(0);
				rightSlave.setEncPosition(0);
				leftMaster.setEncPosition(0);
				leftSlave.setEncPosition(0);
				horzGyro.reset();
				if(leftMaster.getEncPosition() == 0){
					ballsOutAuton = BallsOut.forward;
				}
				break;
			case forward:
				leftMaster.set(-normPID(2561,leftMaster.getEncPosition(),0.00217,0));
				rightMaster.set(-normPID(-2654,rightMaster.getEncPosition(),0.00217,0));
				System.out.println("Front Left: " + leftMaster.getEncPosition() + " Front Right: " + rightMaster.getEncPosition() + " Back Left: " + leftSlave.getEncPosition() + " Back Right: " + rightSlave.getEncPosition() + " Gyro: "+ horzGyro.getAngle());

				if(leftMaster.getEncPosition() >= 2555 && rightMaster.getEncPosition() <= -2650){
					leftMaster.set(0);
					rightMaster.set(0);
					ballsOutAuton = BallsOut.turn;
				}

				break;
			case turn:
					leftMaster.set(-normPID(-90,horzGyro.getAngle(),autoP,0));
					rightMaster.set(-normPID(-90,horzGyro.getAngle(),autoP,0));
					System.out.println("WE MADE IT TO TURN" + "Gyro: " + horzGyro.getAngle());

					
					if(horzGyro.getAngle() <= -86){
						leftMaster.set(0);
						rightMaster.set(0);
						ballsOutAuton = BallsOut.tacticalReload;
					}

				 break;
			case tacticalReload:
				System.out.println("TACTICAL RELOAD");
				leftMaster.set(-normPID(2561,leftMaster.getEncPosition(),0.00217,0));
				rightMaster.set(-normPID(-2654,rightMaster.getEncPosition(),0.00217,0));
				
				System.out.println("Front Left: " + leftMaster.getEncPosition() + " Front Right: " + rightMaster.getEncPosition() + " Back Left: " + leftSlave.getEncPosition() + " Back Right: " + rightSlave.getEncPosition() + " Gyro: "+ horzGyro.getAngle());

				if(leftMaster.getEncPosition() >= 2555 && rightMaster.getEncPosition() <= -2650){
					leftMaster.set(0);
					rightMaster.set(0);
				}
				break;
		}
		*/
		
	}

	@Override
	public void teleopInit() {
		flyWheel.setEncPosition(0);
		rightMaster.setEncPosition(0);
		rightSlave.setEncPosition(0);
		leftMaster.setEncPosition(0);
		leftSlave.setEncPosition(0);
		//frontSolenoid.set(true); //This deploys omni if not already deployed.
		//backSolenoid.set(true);	 // ^
		//armSolenoid.set(true);   //Deploys arm if not already deployed.
	}
	
	@Override
	public void teleopPeriodic() {
		//System.out.println("rightMaster:  " + rightMaster.getEncPosition() + "leftMaster:  " + leftMaster.getEncPosition() + "rightSlave:  " + rightSlave.getEncPosition() + "leftSlave:  " + leftSlave.getEncPosition());
		if(driver.getRawButton(buttonCircle)){
			climber.set(1);
		}else{
			if(driver.getRawButton(buttonX))
			climber.set(-1);
			else{
				climber.set(0);
			}
		}
		gearManipulator();
		shooter();
		hood();
		drivebase();
		smartDash();
		System.out.println("Front Left: " + leftMaster.getEncPosition() + " Front Right: " + rightMaster.getEncPosition() + " Back Left: " + leftSlave.getEncPosition() + " Back Right: " + rightSlave.getEncPosition() + " Gyro: "+ horzGyro.getAngle());
		//System.out.println("P: " + flyP + " I: " + flyI + " D: " + flyD+ " F: " +flyWheel.getF());
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
	
	
	void drivebase(){
		double speed = deadBand(-driver.getY());
		double turn = deadBand(-driver.getZ());
		
		leftMaster.set(robotSpeed*(-speed + turn));
		rightMaster.set(robotSpeed*(speed + turn));
		
		//Solenoids
		if(driver.getRawButton(leftBumper)){
			frontSolenoid.set(true); //omni
		}
		if(driver.getRawButton(leftTrigger)){
			backSolenoid.set(true);  //omni
		}
		if(driver.getRawButton(rightBumper)){
			frontSolenoid.set(false);//traction
		}
		if(driver.getRawButton(rightTrigger)){
			backSolenoid.set(false);//traction
		}
		
		if(oper.getRawButton(leftBumper)){
			armSolenoid.set(false);
		}
		if(oper.getRawButton(leftTrigger)){
			armSolenoid.set(true);
		}
		
		if(driver.getRawButton(1)){
			rightMaster.setEncPosition(0);
			rightSlave.setEncPosition(0);
			leftMaster.setEncPosition(0);
			leftSlave.setEncPosition(0);
		}
	
		
	}
	
	
	void hood(){
		
		if(oper.getPOV() == 0){
			hood.set(-hoodPID(680,hoodEnc.getValue()));
		}
		if(oper.getRawButton(rightBumper)){
			hood.set(.95);
		}
		else{
			if(oper.getRawButton(rightTrigger)){
				hood.set(-.95);
			}
			else{
				hood.set(0);
			}
		}
	
	}
	
	
	void shooter(){
		turret.set(-(deadBand(oper.getZ())));
		smartDash();
		//shooting
		double flyWheelRPM = wheelRPM;
		//System.out.println(flyWheel.getSpeed());
		flyWheel.setF(fGain(flyWheelRPM)/2);
		if(oper.getRawButton(buttonTriangle)){
			// I 7.5E-6
			// P .15
			// D .5
			flyWheel.changeControlMode(TalonControlMode.Speed);
			flyWheel.set(flyWheelRPM);

			lifter.set(-1);
			kicker.set(-1);
			smartDash();
			
		}else{
			flyWheel.changeControlMode(TalonControlMode.PercentVbus);
			flyWheel.set(0);
			lifter.set(0);
			kicker.set(0);
		}
		if(oper.getRawButton(buttonCircle)){
			wheelOfDoom.set(1);													
		}else{
			if(oper.getRawButton(buttonOption)){
				wheelOfDoom.set(-1);
			}else{
				wheelOfDoom.set(0);
			}
		}
		
		double intakeRollers;
		smartDash();
		intakeRollers = deadBand(oper.getY());
		intakeArm.set(intakeSpeed*intakeRollers);
		
		if(oper.getRawButton(touchpad)){
			turret.set(visionPID(96,table.getNumber("COG_Y",0),visionKP));
		}
			
		}	
	
	void gearManipulator(){
		if(oper.getRawButton(buttonSquare)){
			gearSolenoid.set(false);
		}
		if(oper.getRawButton(buttonX)){
			gearSolenoid.set(true);
		}
	}
	double visionPID(double target,double position,double kP){
		double error = target - position;
		double speed = error *kP;
		return speed;
		
	}
	
	
	void smartDash(){
		//wheelRPM = prefs.getDouble("RPM", 0.0);
		//SmartDashboard.putString("DB/String 0", "RPM: ");
		//wheelRPM = Double.parseDouble(SmartDashboard.getString("DB/String 0", "0").substring(5));
		//System.out.println(wheelRPM);
		SmartDashboard.putNumber("Speed", flyWheel.getSpeed());
		SmartDashboard.putNumber("COG_X", table.getNumber("COG_X",217));
		SmartDashboard.putNumber("COG_Y", table.getNumber("COG_Y",217));

		
		
		flyP = pref.getDouble("P",0.035);
		flyI = pref.getDouble("I",0.0000376);
		flyD = pref.getDouble("D",0.35);
		flyF = pref.getDouble("F",0.0299071);
		
		intakeSpeed = pref.getDouble("Intake", 1);
		robotSpeed = pref.getDouble("Speed",1);
		wheelRPM = pref.getDouble("RPM", 3240);
		hoodValue = pref.getDouble("Hood", 650);
		visionKP = pref.getDouble("VisionP", 0.00217);
		autoP = pref.getDouble("AutonGyroP", 0.00217);
		
		autoForwardLeft = pref.getDouble("ForwardLeft", 27);
		autoForwardRight = pref.getDouble("ForwardRight", 27);
		
		
		flyWheel.setP(flyP); //0.1
		flyWheel.setI(flyI); //0.0004895
		flyWheel.setD(flyD); //0.5
		flyWheel.setF(flyF);
	}
	double normPID(double target, double position, double pVal, double iVal){
		double PIDerror = target - position;
		double pOutput = (PIDerror*pVal);
		//double iOutput = (PIDerror*iVal);
		double speed = (pOutput);
		if(Math.abs(PIDerror) < 5){
			speed = 0;
		}
		
		return speed;
	}
	
	double deadBand(double joyStick){
		if(joyStick > -0.08 && joyStick < 0.08){
			joyStick = 0;
		}
		return joyStick;
	}
	
	double fGain(double rpm){
		double FGain;
		FGain = 1023/(4096*rpm/600);
		return FGain;
		
	}
	double hoodPID(double target, double position){
		double error = target - position;
		double speed = error * 0.007;
		System.out.println("Hood Position: " +hoodEnc.getValue() + "Error "+ error + " Speed: " + speed );

		
		return speed;
	}
	
	
}