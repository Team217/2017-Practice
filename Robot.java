package org.usfirst.frc.team217.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;
import com.ctre.*;
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
	
	double speedMulti = .6;
	Joystick driver, oper;
	
	//Talons
	
	CANTalon frontleft, frontright, backleft, backright, middle;
	
	//Solenoids
	
	DoubleSolenoid frontrightsolenoid, frontleftsolenoid, backleftsolenoid, backrightsolenoid;

    public void robotInit() {
    	
    	// Joysticks
    	
    	
    	driver = new Joystick(0);
    	oper = new Joystick(1);
    	
    	
    	// Motor Controllers
    	//Change the parameters for the current testing robot
    	
    	frontleft = new CANTalon(0);
    	backleft = new CANTalon(1);
    	backleft.changeControlMode(TalonControlMode.Follower);
    	backleft.set(0);
    	
    	frontright = new CANTalon(3);
    	backright = new CANTalon(9);
    	backright.changeControlMode(TalonControlMode.Follower);
    	backright.set(3);

    	
    	//Solenoids
    	//Change these
    	backrightsolenoid = new DoubleSolenoid(0, 1);
    	backleftsolenoid = new DoubleSolenoid(6, 7);
    	frontrightsolenoid = new DoubleSolenoid(2, 3);
    	frontleftsolenoid = new DoubleSolenoid(4, 5);
    }
    
	
    public void autonomousInit() {
   
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
  
    }
    
    public void teleopInit(){
    	
    	frontleftsolenoid.set(DoubleSolenoid.Value.kReverse);
		frontrightsolenoid.set(DoubleSolenoid.Value.kReverse);
		backleftsolenoid.set(DoubleSolenoid.Value.kReverse);
		backrightsolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    
    public void teleopPeriodic() {
    	
    	if(driver.getRawButton(5)){
    		frontleftsolenoid.set(DoubleSolenoid.Value.kForward);
    		frontrightsolenoid.set(DoubleSolenoid.Value.kForward);
    		backleftsolenoid.set(DoubleSolenoid.Value.kForward);
    		backrightsolenoid.set(DoubleSolenoid.Value.kForward);
    	}
    	
    	if(driver.getRawButton(6)){
    		frontleftsolenoid.set(DoubleSolenoid.Value.kReverse);
    		frontrightsolenoid.set(DoubleSolenoid.Value.kReverse);
    		backleftsolenoid.set(DoubleSolenoid.Value.kReverse);
    		backrightsolenoid.set(DoubleSolenoid.Value.kReverse);
    	}
    	
    	System.out.println(speedMulti);
    
    	
    	driveBase();
    	
    }

    public void testPeriodic() {

    }
    
    void driveBase(){
    	double speed = deadband(driver.getY());
    	double turn = deadband(driver.getZ());
    	
    	if(driver.getRawButton(7) && driver.getRawButton(8) && driver.getRawButton(4)){ //lt, rt, and triangle
    		speedMulti = 1;
    	}
    	if(driver.getRawButton(11)){ //L3
    		speedMulti = .6;
    	}
    		frontleft.set(speedMulti*(speed + -turn));
    		frontright.set(speedMulti*(-(speed) + -turn));
    	
    }

    double deadband(double input) {

		if (absVal(input) < .08) {
			return 0;
		}return input;	
	}

		
    double absVal(double input){
		if (input < 0){
			return -input;
		}else
			return input;
    }
    
}
