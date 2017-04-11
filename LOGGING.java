
package org.usfirst.frc.team217.robot;

//import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.command.Scheduler;
//import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.*;

//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.*;

import com.ctre.*;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	/*
	 * public static final ExampleSubsystem exampleSubsystem = new
	 * ExampleSubsystem(); public static OI oi;
	 *
	 * Command autonomousCommand; SendableChooser chooser;
	 */
	// Defense Option Select
	int defenseSelected;

	// Position Option Select
	int positionSelected;

	// auton fire option select
	int autonFireSelected;

	// auton Timer.delay option select
	double autonWaitSelected;

	// Shooter PID
	double flyWheelSpeed;
	double flyWheelTarget = 3600;

	boolean isAtSpeed = false;

	double intakeArmSpeed = 0;
	double armKP, armKI;

	// Drive Motors TODO:
	CANTalon left, left2, right, right2;

	// Shooter Motors
	CANTalon intakeInner, intakeOuter, flyWheel, flyWheel2, arm, hood;

	// Controllers
	Joystick driver, oper;

	// Solenoids
	DoubleSolenoid rightPTO, leftPTO;
	Solenoid climberAngle;
	Solenoid climbLock;

	// Relay
	//Relay ledSpike;

	// LED bool
	boolean ledBool = false;

	// Gyros
	//AnalogGyro horzGyro;
	ADXRS450_Gyro vertGyro;
	double xAngle = 0, yAngle = 0;
	double target = 0;
	double gyroKP = 0.05;

	boolean climbIsReleased = false; // climber is locked
	boolean ptoEngaged = true;
	boolean angleSet = false;
	boolean pidArm = false;
	boolean angleReleased = true;
	boolean rampartsFlag = false;

	// Toggle variables for climber
	boolean stateAngle = true, releasedAngle = true;
	boolean stateLock = true, releasedLock = true;
	boolean statePTO = false, releasedPTO = true;

	// Toggle variables for anti-tip
	boolean stateGyro = true, releasedGyro = true, gyroDisable = false;

	// Banner Sensor for ball detection and Hood Limit Switch
	DigitalInput bannerInner, zeroHood;
	boolean ballHit = false;
	DigitalInput climberSwitch;

	// NetworkTables and Vision variables
	NetworkTable table;
	NetworkTable preferences;
	double cogx = 0;
	double cogy = 0;
	double cumcogPIDError = 0;
	double cogPIDError = 0;

	File file = new File("C:/Users/Alexandra/Desktop/Logging.txt");
//	file.getParentFile().mkdirs();

	PrintWriter logger;
	
	// x PID
	double cogxP = 0.005;
	double cogxI = 0.0009; // .05
	double cogxTar = 190; // TODO: get updated val

	// normPID/Y
	double PIDError = 0;
	double cumPIDError = 0;

	// shooting state machine
	enum shootState {
		intake, // Move around and accept a ball
		// Starts 2.5 sec after ball is fired. Ends if ball hits banner sensor
		reving, // Spin firing talon until its fast enough to throw a ball into
				// a goal
		// Starts after ball hits banner, ends if aborted or at 24000 speed
		firing // Ball is sent to the firing talon to fly through the air
		// Starts when reving is at full speed and ends after 2.5 seconds
	};

	shootState currShootState;

	// Timers
	Timer approachTimer; // auton
	Timer rampartsTimer;// ramparts timer
	Timer resetFiringState; // teleop shooterPID
	Timer posTwoTimer; // ummmmm uhhhhh position 2 timer
	Timer matchTimer; // amt of time the match has gone for
	
	double maxTime = 2.25;// the amt of time the logger will run for. chosen by us

	//logging variables
	double AvgCurrent;
	double LFCurrent;
	double RFCurrent;
	double LRCurrent;
	double RRCurrent;
	double gyroVal;
	double cogxVal;
	double cogyVal;
	double currMatchTime;
	
	// Auton Enums
	enum moatEnum {
		moatIntake, moatFirstBump, moatBack, moatSecondBump, moatDrive, moatDrive2, moatRotate, moatShoot
	};

	moatEnum moatState;

	enum rockWallEnum {
		rockWallIntake, rockWallApproach, rockWallCross, rockWallDrive, rockWallDrive2, rockWallRotate, rockWallShoot
	};

	rockWallEnum rockWallState;

	enum roughTerrainEnum {
		roughTerrainIntake, roughTerrainDrive, roughTerrainRotate, roughTerrainShoot
	};

	roughTerrainEnum roughTerrainState;

	enum rampartsEnum {
		rampartsIntake, rampartsRush, rampartsApproach, rampartsCross, rampartsDrive, rampartsRotate, rampartsShoot
	};

	rampartsEnum rampartsState;

	enum spyEnum {
		spyIntake, spyRotate, spyShoot
	};

	spyEnum spyState;

	enum autonShootState {
		autonAligning, // Robot turns left or right in order to see goal
		autonAligning2, // Robot aligns according to X and Y vision
		autonReving, // Shooter begins reving up to fire
		autonFiring // Shooter fires
	};

	autonShootState autonShoot;

	enum posTwo {
		turn1, forward, turn2
	};

	posTwo posTwoState;

	// Auton SmartDash Variables
	int defense, position;
	boolean autonTimerFlag = true;
	double gyroTar = 0;
	double antiTipAngle = -28;
	boolean antiTipEngaged = false;
	double angleTar = 0;
	// Auton Variables
	double autonTime = 0; // chosen via pos
	boolean turnRight = false; // rotate until vision targets
	// Live Stream-Intermediate
	int session;
	//Image frame;

	int myclimb = 0;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {

		// Gyros
		horzGyro = new AnalogGyro(0); // need to init still
		vertGyro = new ADXRS450_Gyro();

		horzGyro.initGyro();

		// Joysticks
		driver = new Joystick(0);
		oper = new Joystick(1);

		// Drive motors
		left = new CANTalon(14);
		left2 = new CANTalon(15);
		left2.changeControlMode(TalonControlMode.Follower);
		left2.set(14);

		right = new CANTalon(12);
		right2 = new CANTalon(13);
		right2.changeControlMode(TalonControlMode.Follower);
		right2.set(12);

		// Intake, shooter motors
		intakeInner = new CANTalon(6);
		intakeOuter = new CANTalon(3);

		flyWheel = new CANTalon(0);
		flyWheel.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		flyWheel.configEncoderCodesPerRev(1000);
		flyWheel.changeControlMode(TalonControlMode.Speed);
		flyWheel.setProfile(1);

		flyWheel2 = new CANTalon(1);
		flyWheel2.changeControlMode(TalonControlMode.Follower);
		flyWheel2.set(0);

		arm = new CANTalon(2);
		hood = new CANTalon(7);

		// Relay
		ledSpike = new Relay(0);

		// Solenoids
		rightPTO = new DoubleSolenoid(0, 3);
		leftPTO = new DoubleSolenoid(1, 2);
		climberAngle = new Solenoid(5);
		climbLock = new Solenoid(4);

		// NetworkTables
		table = NetworkTable.getTable("SmartDashboard");
		preferences = NetworkTable.getTable("Preferences");

		// Ball Recog Sensors
		//bannerInner = new DigitalInput(0); // 0 means go; 1 means stop

		// Hood Limit Switch
		zeroHood = new DigitalInput(1); // 1 means not pressed; 0 means pressed

		// Climber drive stop
		climberSwitch = new DigitalInput(9);

		// Timer
		approachTimer = new Timer();
		resetFiringState = new Timer();
		rampartsTimer = new Timer();
		posTwoTimer = new Timer();
		matchTimer = new Timer();
		
		//Reset Logging variables
		AvgCurrent    = 0;
		LFCurrent     = 0;
		RFCurrent     = 0;
		LRCurrent     = 0;
		RRCurrent     = 0;
		gyroVal       = 0;
		cogxVal       = 0;
		cogyVal       = 0;
		currMatchTime = 0;
		System.out.println("KILL MEEEEE");
	
		try {
			logger = new PrintWriter("logging.txt");
			logger.close();
		} catch (FileNotFoundException e) {
			
			e.printStackTrace();
		}
	
		System.out.println("KJLBHD");
		
		// Default auton shoot state
		autonShoot = autonShootState.autonAligning;
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	public void disabledInit() {
		ledSpike.set(Relay.Value.kOff);
		ledBool = false;
	}

	public void disabledPeriodic() {
		// Scheduler.getInstance().run();
		if (climberSwitch.get() == false) {
			vertGyro.calibrate();
			horzGyro.calibrate();
		}
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	public void autonomousInit() {
		// autonomousCommand = (Command) chooser.getSelected();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		// if (autonomousCommand != null) autonomousCommand.start();

		matchTimer.start();
		
		
		//ledBool = false;
		left.set(0);
		right.set(0);

		//Ramparts gyro target
		gyroTar = 0;


		//Select Defense and Position
		leftPTO.set(DoubleSolenoid.Value.kForward);
		rightPTO.set(DoubleSolenoid.Value.kReverse);
		//Reset Gyros for Auton Use
		vertGyro.reset();
		horzGyro.reset();
		xAngle = 0;
		yAngle = 0;

		//Wait a specified amount of time before auton
		Timer.delay(autonWaitSelected);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
	}

	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		// if (autonomousCommand != null) autonomousCommand.cancel();

		flyWheel.setEncPosition(0);
		approachTimer.stop();

		//Live Stream
		//IMAQdxStartAcquisition(session);

		//ensure solenoids are disengaged
		climberAngle.set(false);
		climbLock.set(false);

		//leftPTO.set(DoubleSolenoid.Value.kForward);
		//rightPTO.set(DoubleSolenoid.Value.kReverse);
		//ptoEngaged = true;
		//statePTO = false;
		gyroDisable = false;
		
		matchTimer.start();//TODO: comment out this line if ur doing auton stuff
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		// Scheduler.getInstance().run();
		disableGyro();

		flyWheelSpeed = flyWheel.getSpeed();

		if (driver.getRawButton(7)) {
			visionXAlign();
		} else {
			if (vertGyro.getAngle() < antiTipAngle && ptoEngaged == true
					&& !gyroDisable)	//pto is not engaged
					{
				left.set(-.65);
				right.set(.65);
			} else {
				Drivebase();
			}
		}
		if (oper.getRawButton(10)) {
			flyWheel.changeControlMode(TalonControlMode.PercentVbus);
			flyWheel.set(1);
			if (oper.getRawButton(14)) {
				setIntake(1);
			} else if (!oper.getRawButton(14)) {
				setIntake(0);
			}
		} else {
			Shooter();
		}
		Gyro();
		Arm();
		//stops hood from going past point
		if (hood.getEncPosition() < -1650) {
			hood.set(normPID(-1300, hood.getEncPosition(), .0048, 0));
		} else
			Hood();
		Climber();
//		LiveStream();
		
		LFCurrent     = left.getOutputCurrent();
		RFCurrent     = right.getOutputCurrent();
		LRCurrent     = left2.getOutputCurrent();
		RRCurrent     = right2.getOutputCurrent();
		AvgCurrent    = (LFCurrent + RFCurrent + LRCurrent + RRCurrent)/4;
		//gyroVal       = horzGyro.getAngle();
		//cogxVal       = ;
		//cogyVal       = ;
		currMatchTime = matchTimer.get();
		System.out.println("WE ALMOST GOT IT");
		try{
		logger.println(" { \"MatchTime\" : " + currMatchTime
				+ "\"Average Drivebase Current Draw\" : " + AvgCurrent + " } \n" );
		} catch(Exception e){
			//do some shit
		}
		System.out.println("WE GOT IT");
		//double[] logging = new double[]{currMatchTime, LFCurrent, 
			//	RFCurrent, LRCurrent, RRCurrent, AvgCurrent, gyroVal};
		
//		String logOutput = " { ";
		/*
		for(int i = 0; i < 10; i++)
		{
			if(i == 0){
				logOutput = logOutput + "\"MatchTime\" : " + Double.toString(logging[i]);
			}
			if(i == 1){
				logOutput = 
			}
			
			
			//logger.println( (logging[i]).toString )
			
		}
		*/
			logger.close();
				
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		// LiveWindow.run();
	}
	
	void disableGyro() {
		// manual disable antitip
		if (driver.getRawButton(13) && !stateGyro) // after second push
		{
			releasedGyro = false;
			gyroDisable = false; // Enable anti-tip
		} else if (driver.getRawButton(13) && stateGyro) // first push
		{
			releasedGyro = false;
			gyroDisable = true; // disable anti-tip
		} else if (!driver.getRawButton(13) && !releasedGyro) // happens first
		{
			stateGyro = !stateGyro;
			releasedGyro = true;
		}
	}

	// Function for operating drivebase. Allows for tank/Arcade option select
	// Includes gyro correction. Must be tested
	void Drivebase() {
		double speed = deadband(-driver.getY());
		double turn = deadband(driver.getZ());

		if (!driver.getRawButton(8)) {
			target = xAngle;
		}

		if (climberSwitch.get() == false && !ptoEngaged) {
			left.set(0);
			right.set(0);
		}

		else {
			left.set(speed + turn + gyroPID(target));
			right.set(-speed + turn + gyroPID(target));
		}
	}

	// Updates angle variables, resets x gyro when full rotation is completed
	void Gyro() {
		//xAngle = horzGyro.getAngle();
		yAngle = vertGyro.getAngle();
		if (xAngle == 360 || xAngle == -360) {
			xAngle = 0;
		}
		if (driver.getRawButton(6)) {
			//horzGyro.reset();
			vertGyro.reset();
		}
	}

	// Function for operating shooter
	void Hood() {

		// stops hood from going past point
		if (hood.getEncPosition() < -1650) {
			hood.set(normPID(-1300, hood.getEncPosition(), .0048, 0));
		} else {
			// reset hood when its all the way down
			if (zeroHood.get() == false) {

				if (oper.getRawButton(6) == true) {
					hood.set(0);
				}
				hood.setEncPosition(0);
				hood.set(normPID(50, hood.getEncPosition(), .0048, 0));
			} else if (driver.getRawButton(5)) // move hood up
			{
				visionYAlign();
			} else {
				if (oper.getRawButton(6)) {
					hood.set(.75);
				} else if (oper.getRawButton(8)) {
					hood.set(-.75);
				} else
					hood.set(0);
			}

			// Far far shot (position 2 outer works)
			if (oper.getRawButton(9)) {
				hood.set(normPID(640, hood.getEncPosition(), .0048, 0));// 610
				flyWheelTarget = 4050;
			}
			// lob shot, arm out
			if (oper.getPOV() == 0) {
				hood.set(normPID(815, hood.getEncPosition(), .0048, 0));
				flyWheelTarget = 3315;
			}
			// far shot
			if (oper.getPOV() == 90) {
				hood.set(normPID(624, hood.getEncPosition(), .0048, 0));// 679
				flyWheelTarget = 3950;
			}
			// mid shot
			if (oper.getPOV() == 180) {
				hood.set(normPID(655, hood.getEncPosition(), .0048, 0));// 660
				flyWheelTarget = 3600;
			}
			// close shot, edge of batter
			if (oper.getPOV() == 270) {
				hood.set(normPID(835, hood.getEncPosition(), .0048, 0));// 855
				flyWheelTarget = 3600;
			}
			// Batter shot, fully up touching tower
			if (oper.getRawButton(14) && !oper.getRawButton(10)) {
				hood.set(normPID(1125, hood.getEncPosition(), .0048, .0025)); // 1320=hard,1340=soft
				flyWheelTarget = 3000; // 3600=hard,3000=soft
			}
		}
	}

	// state machine for operating shooter
	void Shooter() {
		/*
		 * 1. Operator uses intake to claim a ball and can spit out as well. Arm
		 * comes down automatically. 2. Banner sensor is hit to stop ball. Ball
		 * can be spit out. Arm comes all the way up. 3. Operator revs up
		 * shooter and holds intake, intake is not activated until shooter is up
		 * to speed.
		 */
		/*ballHit = bannerInner.get();
		if (ballHit || ledBool) {
			ledSpike.set(Relay.Value.kOn);
		} else {
			ledSpike.set(Relay.Value.kOff);
		}*/
		switch (currShootState) {

		case intake:
			stopFlyWheel();
			isAtSpeed = false;

			// In this mode, the robot is waiting to obtain a ball
			if (oper.getRawButton(5)) {
				setIntake(-1);

				// If the regurgitation button is pressed, pull a ball in
			} else if (oper.getRawButton(7)) {
				setIntake(1);
				if (!oper.getRawButton(3)) {
					pidArm = true;
					armPID(-1250);
				}
				// If the intake button is pressed, spit the ball out
			} else {
				setIntake(0);
				pidArm = false;
			}

			if (ballHit == true) {
				// switch states because we've fully loaded a ball
				setIntake(0);
				currShootState = shootState.reving;
			}

			break;
		case reving:
			// In this mode, the robot is waiting for the shooter to reach 24K
			// speed
			if (oper.getRawButton(7) && !oper.getRawButton(3)) {
				armPID(0);
				pidArm = true;
			} else {
				pidArm = false;
			}

			if (oper.getRawButton(5)) {
				setIntake(-1);
				currShootState = shootState.intake;
				// Regurgitate, and go to intake because we're spitting out the
				// ball

			} else if (oper.getRawButton(2)) {
				intakeOuter.set(-1);

			} else if (!oper.getRawButton(2)) {
				intakeOuter.set(0);
			}
			if (oper.getRawButton(4)) {
				setFlyWheelRPM(flyWheelTarget);
				// This runs the rev up process, PID'ing the shooter to 24000.
			} else {
				stopFlyWheel();
			}

			if (isAtSpeed && oper.getRawButton(7)) // isAtSpeed
			{
				// switch states because we are completely reved up
				currShootState = shootState.firing;

				resetFiringState.reset();
				resetFiringState.start();
			}
			break;
		case firing:
			if (ballHit == true) {
				// If the ball is still at the banner sensor, push it into the
				// shooter
				intakeInner.set(-1);
			}
			if (resetFiringState.get() >= 1.5) {
				currShootState = shootState.intake;
			}
			break;
		}

	}

	void setFlyWheelRPM(double rpm) {
		flyWheel.changeControlMode(TalonControlMode.Speed);
		flyWheel.set(rpm);
		flyWheelSpeed = flyWheel.getSpeed();
		double flyWheelError = absVal(rpm - flyWheelSpeed) - 360;
		if (flyWheelError <= 500) {
			isAtSpeed = true;
		} else {
			isAtSpeed = false;
		}
	}

	void stopFlyWheel() {
		flyWheel.changeControlMode(TalonControlMode.PercentVbus);
		flyWheel.set(0);
	}

	// function for operating arm
	void Arm() {
		/*
		 * Arm comes up full speed, down half speed (0.5)
		 *
		 * Square - arm comes up to 0 X - arm becomes level with floor
		 */

		// left stick
		intakeArmSpeed = deadband(oper.getY());
		if (arm.getEncPosition() > -650 && intakeArmSpeed < 0) {
			intakeArmSpeed = 0;
		}
		if (oper.getRawButton(13)) {
			arm.setEncPosition(0);

		}

		if ((deadband(oper.getRawAxis(5)) != 0) && (deadband(oper.getRawAxis(5)) < 0)) // move
																						// intake
																						// arm
																						// up
																						// or
																						// down
		{
			arm.set(deadband(oper.getRawAxis(5)) * .8);
		} else if (intakeArmSpeed < 0) {
			arm.set(intakeArmSpeed * .5);
			pidArm = false;
		} else if (intakeArmSpeed > 0) {
			arm.set(intakeArmSpeed * 0.3);
			pidArm = false;
		} else if (oper.getRawButton(3)) {
			armPID(0);
		} else if (oper.getRawButton(1)) {
			armPID(-1000);
		} else if (oper.getRawButton(2)) {
			armPID(-1250);
		} else if (!pidArm) {
			arm.set(0);
		}
	}

	// Sets speeds of both drive motors to for/back
	void Climber() {
		// string climbString = "climb: " + std.Value.to_string(myclimb);
		// SmartDashboard.Value.PutString("DB/String 2", climbString);
		/*
		 * Square - Toggle for angle X - Toggle for release Circle - Toggle for
		 * PTOs
		 */

		// climber angle Toggle
		if (driver.getRawButton(1) && !stateAngle) // after second push
		{
			releasedAngle = false;
			climberAngle.set(false); // becomes 0 degrees
			myclimb = 1;

		} else if (driver.getRawButton(1) && stateAngle) // first push
		{
			releasedAngle = false;
			climberAngle.set(true); // becomes 30 degrees
			myclimb = 2;

		} else if (!driver.getRawButton(1) && !releasedAngle) // happens first
		{
			stateAngle = !stateAngle;
			releasedAngle = true;
			myclimb = 3;
		}

		// climber release Toggle
		if (driver.getRawButton(2) && !stateLock) // after second push
		{
			releasedLock = false;
			// climbLock.set(0);//re-engages lock
			myclimb = 4;

		} else if (driver.getRawButton(2) && stateLock) // first push
		{
			releasedLock = false;
			climbLock.set(false); // releases lock
			ledBool = true;
			myclimb = 5;
		} else if (!driver.getRawButton(2) && !releasedLock) // happens first
		{
			stateLock = !stateLock;
			releasedLock = true;
			myclimb = 6;
		}

		// climberPTO Toggle
		if (driver.getRawButton(3) && !statePTO) // after second push
		{
			releasedPTO = false;
			ptoEngaged = false;
			//leftPTO.set(DoubleSolenoid.Value.kReverse);
			//rightPTO.set(DoubleSolenoid.Value.kForward); // climb mode
			Timer.delay(.25);
			releasedAngle = false;
			climberAngle.set(false); // becomes 0 degrees
			myclimb = 7;

		} else if (driver.getRawButton(3) && statePTO) // first push
		{
			releasedPTO = false;
			ptoEngaged = true;
			//leftPTO.set(DoubleSolenoid.Value.kForward);
			//rightPTO.set(DoubleSolenoid.Value.kReverse); // drive mode
			myclimb = 8;

		} else if (!driver.getRawButton(3) && !releasedPTO) // happens first
		{
			statePTO = !statePTO;
			releasedPTO = true;
			myclimb = 9;
		}

	}

	// PID for x-axis alignment
	void visionXAlign() {
		cogx = table.getNumber("COG_X", 1000);
		left.set(-visionPID(cogxTar, cogx));
		right.set(-visionPID(cogxTar, cogx));
	}

	// PID for hood alignment
	// TODO: find actual values for this
	void visionYAlign() {
		cogy = table.getNumber("COG_Y", 217);
		if (cogy < 28) // backed up to defenses at courtyard
		{
			hood.set(normPID(640, hood.getEncPosition(), .0048, 0));
		} else if (cogy < 55) {
			hood.set(normPID(640, hood.getEncPosition(), .0048, 0));
		} else if (cogy < 82) {
			hood.set(normPID(640, hood.getEncPosition(), .0048, 0));
		} else if (cogy < 110) {
			hood.set(normPID(650, hood.getEncPosition(), .0048, 0));
		} else if (cogy < 140) {
			hood.set(normPID(750, hood.getEncPosition(), .0048, 0));
		} else if (cogy < 165) {
			hood.set(normPID(830, hood.getEncPosition(), .0048, 0));
		} else
			hood.set(normPID(880, hood.getEncPosition(), .0048, 0));

	}

	double gyroPID(double target) {
		double gyroError;

		gyroError = target - xAngle;
		return gyroError * gyroKP;

	}

	// PID for x-axis vision alignment use
	// TODO: DELETE THIS ONE AND UNCOMMENT THE OTHER ONE FOR IRI
	// TODO: YES YOU GWELLY
	// TODO: IM NOT KIDDING
	// TODO: IF I COME TO PITS AND THIS ISNT DONE I S2G

	double visionPID(double cogTar, double cogPos) {
		cogPIDError = cogTar - cogPos;
		double cogPIDSpeed;

		if (cogPIDError > 40) {
			cogPIDSpeed = .38; // was .37
		} else if (cogPIDError > 5) {
			cogPIDSpeed = .31; // was .27
		} else if (cogPIDError < -40) {
			cogPIDSpeed = -.38; // was -.37
		} else if (cogPIDError < -5) {
			cogPIDSpeed = -.31; // -.27
		} else {
			cogPIDSpeed = 0;
		}
		return (cogPIDSpeed);
	}

	// double visionPID(double cogTar, double cogPos)
	// {
	// cogPIDError = cogTar - cogPos;
	// double cogPIDSpeed;
	//
	// if (cogPIDError > 40)
	// {
	// cogPIDSpeed = .38;//was .37
	// }
	// else if (cogPIDError > 5)
	// {
	// cogPIDSpeed = .34;//was .27
	// }
	// else if (cogPIDError < -40)
	// {
	// cogPIDSpeed = -.38;//was -.37
	// }
	// else if (cogPIDError < -5)
	// {
	// cogPIDSpeed = -.34;//-.27
	// }
	// else
	// {
	// cogPIDSpeed = 0;
	// }
	// return (cogPIDSpeed);
	// }

	// PID for arm
	void armPID(double armTar) {
		double armCur = arm.getEncPosition();
		double armSpeed = -normPID(armTar * 4, -armCur, .00015, 0);
		if (armSpeed > .5) {
			armSpeed = .5;
		}
		if (armSpeed < -.5) {
			armSpeed = -.5;
		}
		arm.set(armSpeed);

	}

	// Standard PID function
	double normPID(double myTar, double myPos, double myP, double myI) {
		PIDError = myTar + myPos;
		cumPIDError = PIDError;
		double PIDPout = PIDError * myP;
		double PIDIout = PIDError * myI;
		double PIDSpeed = (PIDPout + PIDIout);

		if (absVal(PIDError) < 5) {
			PIDSpeed = 0;
		}
		return (PIDSpeed);
	}

	// Sets all wheels to drive at same speed
	void setSpeed(double speed) {
		left.set(speed);
		right.set(-speed);
	}

	// Sets speed of intake motors
	void setIntake(double speed) {
		intakeInner.set(-speed);
		intakeOuter.set(speed);
	}

	// Nulls idle stick input at 0.08
	double deadband(double input) {

		if (absVal(input) < .08) {
			return 0;
		}
		return input;
	}

	// choosing position for auton
	void positionChoosing() {
		if (positionSelected == 1)
			position = 1;
		if (positionSelected == 2)
			position = 2;
		if (positionSelected == 3)
			position = 3;
		if (positionSelected == 4)
			position = 4;
		if (positionSelected == 5)
			position = 5;
		if (positionSelected == 6)
			position = 6;
		if (positionSelected == 7)
			position = 7;
	}

	// choosing defense to cross in auton
	void defenseChoosing() {
		if (defenseSelected == 1) {
			defense = 1;
			moatState = moatEnum.moatIntake;
		}
		if (defenseSelected == 2) {
			defense = 2;
		}
		if (defenseSelected == 3) {
			defense = 3;
			rockWallState = rockWallEnum.rockWallIntake;
		}
		if (defenseSelected == 4) {
			defense = 4;
			roughTerrainState = roughTerrainEnum.roughTerrainIntake;
		}
		if (defenseSelected == 5) {
			defense = 5;
			rampartsState = rampartsEnum.rampartsIntake;
		}
		if (defenseSelected == 6) {
			defense = 6;
			spyState = spyEnum.spyIntake;
		}
	}

	// C++ abs function sucks
	double absVal(double input) {
		if (input < 0)
			return -input;
		return input;
	}
}
