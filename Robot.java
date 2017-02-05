
package org.usfirst.frc.team217.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.CANSpeedController.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;
import com.ctre.*;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;


/**
\* The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	//Joysticks
	
	Joystick driver, oper;
	
	//Drive talons
	
	CANTalon frontleft, frontright, backleft, backright;
	
	//shooting talons
	
	CANTalon turretRotation, flyWheel, hood, ballLift, intakeArm, funnel;
	
	//climbing talons
	
	CANTalon climber, climber2;
	
	//Solenoids
	
	Solenoid fRS, fLS, bLS, bRS, intakeS;
	
	Compressor compressor;
	
	//Timers
	
	//Gyro
	
	AnalogGyro horzGyro;
	
	//shooter PID
	
	double flyWheelSpeed;
	double targetspeed;
	
	//PID
	
	double PIDError = 0;
	double cumPIDError = 0;
	
	//auton variables
	boolean SafeCheck, ballsOut, gearGone;
	 	
	 	enum AutonEnum{
	 		Fire10,
	 		placeGear,
	 		LoadBalls,
	 		FireMore,
	 		stop
	 	};
	 	
	 	AutonEnum AutonState = AutonEnum.Fire10;
	 	
	 	enum StartPosEnum{
	 		Spos1,
	 		Spos2,
	 		Spos3
	 	};
	 	
	 	StartPosEnum SPos;
	 	
	 	enum GearPosEnum{
	 		Gpos1,
	 		Gpos2,
	 		Gpos3
	 	};
	 	
	 	GearPosEnum GPos;
	 	
	 	enum SideEnum{
	 		red,
	 		blue
	 	};
	 	
	 	SideEnum Side;
	 	
	 	enum shootingenum{
	 		
	 	}
	 	

	
    public void robotInit() {
    	
    	//Live Stream OpenCV Testing
    			/*
	  		new Thread(() -> {
	UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
  	camera.setResolution(640, 480);
	    			            
    CvSink cvSink = CameraServer.getInstance().getVideo();
	CvSource outputStream = CameraServer.getInstance().putVideo("cam0", 640, 480);
	    			            
 	Mat source = new Mat();
 	Mat output = new Mat();
	    			            
	while(true) {
	cvSink.grabFrame(source);
	Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2RGB);
	outputStream.putFrame(output);
      }
  		        }).start();
    	   */
    	
    	// Joysticks
    	driver = new Joystick(0);
    	oper = new Joystick(1);
    	
    	// Drive Motor Controllers
    	frontleft = new CANTalon(2);
    	backleft = new CANTalon(3);
    	backleft.changeControlMode(TalonControlMode.Follower);
    	backleft.set(2);
    	
    	frontright = new CANTalon(1);
    	backright = new CANTalon(0);
    	backright.changeControlMode(TalonControlMode.Follower);
    	backright.set(1);
    	
    	frontleft.setFeedbackDevice(FeedbackDevice.QuadEncoder);
    	frontright.setFeedbackDevice(FeedbackDevice.QuadEncoder);
   
    	//Solenoids
    	fRS = new Solenoid(0);
    	fLS = new Solenoid(1);
    	bRS = new Solenoid(2);
    	bLS = new Solenoid(3);
    	intakeS = new Solenoid(4);
    	    	
    	//Gyro
    	horzGyro = new AnalogGyro(0);
    	
    	//Shooting Talons
    	turretRotation = new CANTalon(10);
    	flyWheel = new CANTalon(11);
    	//hood = new CANTalon(12);
    	//ballLift = new CANTalon(13);
    	intakeArm = new CANTalon(7);
    	//funnel = new CANTalon(15);
    	
    	//Climber Talons
    	//climber = new CANTalon(16);
    	//climber2 = new CANTalon(17);
        
    }
    
	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
    public void autonomousInit() {
    	horzGyro.reset();
    	frontright.setEncPosition(0);
    	frontleft.setEncPosition(0);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    
    	switch(AutonState){
		case Fire10:
			
			Fire10();
			
			if(ballsOut){ //Check if we are doing ballsOut auton
								//ballsOut is the name for 40 point auton
				
				AutonState = AutonEnum.LoadBalls;
				
			}
			else{
				
				AutonState = AutonEnum.placeGear;
				
			}
			break;
		case placeGear:
			
			PlaceGear();
			
			if(gearGone){ //Check if the gear has been raised from the robot
				
				AutonState = AutonEnum.stop;
				
			}
			
			break;
		case LoadBalls:
			
			LoadBalls();
			
			break;
		case FireMore:
			
			FireMore();
			
			AutonState = AutonEnum.stop;
			
			break;
			
		case stop:
			
			frontright.set(0);
			frontleft.set(0);
			backright.set(0);
			backleft.set(0);
			
			break;
			
	}
          
       System.out.println("Left:" + frontleft.getEncPosition() + "   " + "Right:" + frontright.getEncPosition());
          
    }        
  
    public void teleopInit(){
    	
    	//This may need to be changed depending on whether omni is reverse or forward
    	fLS.set(false);
		fRS.set(false);
		bLS.set(false);
		bRS.set(false);
		intakeS.set(false);
		
		frontright.setEncPosition(0);
    	frontleft.setEncPosition(0);
    }

    
    public void teleopPeriodic() {
 	
    	drivebase();
	 		
	 	flyWheel.set((oper.getY()));		
		turretRotation.set(oper.getZ());	
		
		if(oper.getRawButton(7)){
				intakeS.set(true);
			}
			else{
				if(oper.getRawButton(8)){
					intakeS.set(false);
				}
			}
			
        System.out.println("Left:" + frontleft.getEncPosition() + "   " + "Right:" + frontright.getEncPosition());
        
    	if(driver.getRawButton(14)){
    		frontright.setEncPosition(0);
    		frontleft.setEncPosition(0);
    	}
    	
    	
    }

    public void testPeriodic() {
    
    }
    
    //When PIDing drivebase movement
    //left use:  -normPID, all the rest positive
    //right use: -normPID, -myTar
    double normPID(double myTar, double myPos, double myP, double myI) {
    	PIDError = myTar - myPos;
		cumPIDError = PIDError;
		double PIDPout = PIDError * myP;
		double PIDIout = PIDError * myI;
		double PIDSpeed = (PIDPout + PIDIout);

		if (absVal(PIDError) < 5) {
			PIDSpeed = 0;
		}
		return (PIDSpeed);
    }
    
    void drivebase(){
    	double speed = deadband(driver.getY());
    	double turn = deadband(-driver.getZ());
    	
    	if(driver.getRawButton(5)){
    		fLS.set(true);
    		fRS.set(true);
    		bLS.set(true);
    		bRS.set(true);
    	}
    	
    	if(driver.getRawButton(6)){
    		fLS.set(false);
    		fRS.set(false);
    		bLS.set(false);
    		bRS.set(false);
    	}
    
    		
    		frontleft.set(speed + turn);
    		frontright.set(-speed + turn);
    	}
    
    
   void Fire10() {
		//Auton method for firing the first 10
		switch(Side)
		{
			case red:
			{
					switch(SPos)
					{
						case Spos1:
							break;
						case Spos2:
							break;
						case Spos3:
							break;
					}
			}
			break;
			
			case blue:
			{
					switch(SPos)
					{
						case Spos1:
							break;
						case Spos2:
							break;
						case Spos3:
							break;
					}
			}
			break;
		}
	}
	
	void PlaceGear() {
		//Auton method for driving up and placing gear
		
		switch(GPos)
		{
			case Gpos1:
				break;
			case Gpos2:
				break;
			case Gpos3:
				break;
		}
		
	}
	
	void LoadBalls() {
		//Auton method to drive to the hopper and load balls
		
		switch(Side)
		{
			case red:
			{
					switch(SPos)
					{
						case Spos1:
							break;
						case Spos2:
							break;
						case Spos3:
							break;
					}
			}
			break;
			
			case blue:
			{
					switch(SPos)
					{
						case Spos1:
							break;
						case Spos2:
							break;
						case Spos3:
							break;
					}
			}
			break;
		}
		
	}
	
	void FireMore() {
		//Auton method to fire the balls from the hopper

		
	}
	
	void ballvision(){
		//Does alignment to the boiler for shooting
		//Align turret
		//Align hood
	}
	
	void gearvision(){
		//Does alignment to the gear for docking
		//Moves the bot until aligned to vision coordinates
		//Controls driving to dock perfectly
	}
	
	void shooting(){
		//This method contains all of the shooting code
		
		//align the turret according to vision values
		//align the hood according to vision values
		//run flywheel motor 
		
		
	}
	

    
    
    double deadband(double input) {

		if (absVal(input) < .08) {
			return 0;
		}else
			return input;	}

		
    double absVal(double input){
	if (input < 0){
		return -input;
	}else
		return input;
}


void setFlyWheelRPM(double rpm) {
	flyWheel.changeControlMode(TalonControlMode.Speed);
	flyWheel.set(normPID(rpm,flyWheel.getSpeed(),.00217,0));

}}



