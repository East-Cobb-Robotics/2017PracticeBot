
package org.usfirst.frc.team4910.robot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Calendar;

import org.omg.PortableInterceptor.SYSTEM_EXCEPTION;
import org.usfirst.frc.team4910.iterations.*;
import org.usfirst.frc.team4910.subsystems.DriveTrain.DriveControlState;
import org.usfirst.frc.team4910.subsystems.*;
import org.usfirst.frc.team4910.util.CrashTracker;
import org.usfirst.frc.team4910.util.GyroHelper;
import org.usfirst.frc.team4910.util.Path;
import org.usfirst.frc.team4910.util.Path.PathType;

import com.opencsv.CSVWriter;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends IterativeRobot {
    
	
	Iterator iteratorEnabled = new Iterator();
	Iterator iteratorDisabled = new Iterator();
	public static DriveTrain drive;
	public static Shooter sh;
	static Elevator elev;
	static VisionProcessor vision;
	static Climber climb;
	static OI oi;
	static Path pat;
	static SendableChooser<String> autoChoose;
	static boolean compressorEnabled;
	private boolean tunePID=false;
	public static double closeLoopTime=0;
	
    public void robotInit() {
        try{
        	RobotMap.init();
        	oi = OI.getInstance();
        	drive = DriveTrain.getInstance();
        	sh = Shooter.getInstance();
        	elev = Elevator.getInstance();
        	vision = VisionProcessor.getInstance();
        	climb = Climber.getInstance();
        	pat = new Path();
        	CrashTracker.logRobotInit();
        	iteratorEnabled.register(RobotState.iter);
        	iteratorEnabled.register(drive.getLoop());
        	iteratorEnabled.register(sh.getLoop());
        	iteratorEnabled.register(elev.getLoop());
        	iteratorEnabled.register(vision.getLoop());
        	iteratorEnabled.register(climb.getLoop());
        	iteratorDisabled.register(RobotState.iter);
        	iteratorDisabled.register(new GyroCalibrator());
        	resetAllSensors();
        	
    		autoChoose = new SendableChooser<String>();
    		autoChoose.addObject("POSITION CHOOSER", "0");
    		autoChoose.addDefault("Do Nothing", "Do Nothing");
    		autoChoose.addObject("Red Left", "Red Left");
    		autoChoose.addObject("Red Middle", "Red Middle");
    		autoChoose.addObject("Red Right", "Red Right");
    		autoChoose.addObject("Blue Left", "Blue Left");
    		autoChoose.addObject("Blue Middle", "Blue Middle");
    		autoChoose.addObject("Blue Right", "Blue Right");
    		SmartDashboard.putData("Auto mode", autoChoose);
        	//RobotMap.g.calibrate();
        	
        }catch(Throwable t){
        	CrashTracker.logThrowableCrash(t);
        	throw t;
        }
    }
    
    public void disabledInit(){
        try{
        	CrashTracker.logDisabledInit();
        	iteratorEnabled.stop();
        	iteratorDisabled.start();
        	
        }catch(Throwable t){
        	CrashTracker.logThrowableCrash(t);
        	throw t;
        }
    }
    
    public void disabledPeriodic(){
        try{
        	System.gc();
//        	System.out.println("left enc: "+RobotMap.left1.getEncPosition());
//        	System.out.println("right enc: "+RobotMap.right1.getEncPosition());
        }catch(Throwable t){
        	CrashTracker.logThrowableCrash(t);
        	throw t;
        }
    }
    
    @SuppressWarnings("unused")
	public void autonomousInit() {
        try{
        	resetAllSensors();
        	CrashTracker.logAutoInit();
        	iteratorEnabled.start();
        	iteratorDisabled.stop();
        	Timer.delay(.03);
        	pat.reset();
        	RobotMap.gearShifter.set(DoubleSolenoid.Value.kReverse); //Start in low gear
        	RobotMap.gates.set(DoubleSolenoid.Value.kReverse); //Close gates
        	//vision.startPegTracking();
        	//The point of this is to drive partly to the peg, and then use vision tracking to correct itself,
        	//since we can't expect the human to place it in the right place each time
        	boolean nothing=false;
        	double initialTurnAngle = RobotMap.isCompBot ? 55.0 : 60.0; //We didn't have enough time to fully tune PID but it was close enough to this
        	switch((String)autoChoose.getSelected()){
        	case "Do Nothing":
        		nothing=true;
        		break;
        	case "Red Left":
        		pat.setPositionTimeThresh(7.25);
        		pat.register(Path.PathType.Position, -(80-14.5));
        		pat.register(Path.PathType.Heading, -initialTurnAngle);
        		pat.Iterate();
        		pat.setPositionTimeThresh(7.25);
        		trackAndMove();
        		break;
        	case "Red Middle":
        		pat.register(Path.PathType.Position, 100.39/2.0);
        		pat.Iterate();
        		
        		double ang=0.0, angThresh=0.5;
        		//correct angle
        		vision.startPegTracking();
        		while(vision.getCurrentIteration()<=2);
        		ang=-vision.getAveragePegAngle();
        		vision.stopPegTracking();
        		if(Math.abs(ang)>angThresh){
        			pat.register(PathType.Heading, ang);
        			pat.Iterate();
        		}
        		
        		pat.register(Path.PathType.Position, 100.39/2.0);
        		pat.Iterate();
        		
        		//correct angle
        		vision.startPegTracking();
        		while(vision.getCurrentIteration()<=2);
        		ang=-vision.getAveragePegAngle();
        		vision.stopPegTracking();
        		if(Math.abs(ang)>angThresh){
        			pat.register(PathType.Heading, 1.4*ang);
        			pat.Iterate();
        		}
        		
        		//After this we'd use an ultrasonic sensor to do the rest of the work
        		double ult = ((((RobotMap.ultra.getVoltage()) * 3.47826087) - 0.25)*12.0)-6.0;
        		ult = ult<4.0 ? 0.0 : -(ult+9.0);
        		pat.register(PathType.Position, ult);
        		pat.Iterate();
        		
        		//trackAndMove();
        		break;
        	case "Red Right":
        		pat.setPositionTimeThresh(7.25);
        		pat.register(Path.PathType.Position, -(80-14.5));
        		pat.register(Path.PathType.Heading, initialTurnAngle);
        		pat.Iterate();
        		pat.setPositionTimeThresh(7.25);
        		trackAndMove();
        		break;
        	case "Blue Left":
        		pat.setPositionTimeThresh(7.25);
        		pat.register(Path.PathType.Position, -(80-14.5));
        		pat.register(Path.PathType.Heading, -initialTurnAngle);
        		pat.Iterate();
        		pat.setPositionTimeThresh(7.25);
        		trackAndMove();
        		break;
        	case "Blue Middle":
        		pat.register(Path.PathType.Position, 100.39/2.0);
        		pat.Iterate();
        		
        		double ang2=0.0, angThresh2=0.5; //throws a duplicate variable error for some reason, which is illogical
        		//correct angle
        		vision.startPegTracking();
        		while(vision.getCurrentIteration()<=2);
        		ang2=-vision.getAveragePegAngle();
        		vision.stopPegTracking();
        		if(Math.abs(ang2)>angThresh2){
        			pat.register(PathType.Heading, ang2);
        			pat.Iterate();
        		}
        		
        		pat.register(Path.PathType.Position, 100.39/2.0);
        		pat.Iterate();
        		
        		//correct angle
        		vision.startPegTracking();
        		while(vision.getCurrentIteration()<=2);
        		ang2=-vision.getAveragePegAngle();
        		vision.stopPegTracking();
        		if(Math.abs(ang2)>angThresh2){
        			pat.register(PathType.Heading, 1.4*ang2);
        			pat.Iterate();
        		}
        		
        		//After this we'd use an ultrasonic sensor to do the rest of the work
        		double ult2 = ((((RobotMap.ultra.getVoltage()) * 3.47826087) - 0.25)*12.0)-6.0;
        		ult2 = ult2<4.0 ? 0.0 : -(ult2+9.0);
        		pat.register(PathType.Position, ult2);
        		pat.Iterate();
        		break;
        	case "Blue Right":
        		pat.setPositionTimeThresh(7.25);
        		pat.register(Path.PathType.Position, -(80-14.5));
        		pat.register(Path.PathType.Heading, initialTurnAngle);
        		pat.Iterate();
        		pat.setPositionTimeThresh(7.25);
        		trackAndMove();
        		break;
        	default:
        		break;
        	}
        	if(!nothing && RobotMap.isCompBot){
        		RobotMap.gates.set(DoubleSolenoid.Value.kForward); //Open gates
        		Timer.delay(.16);
        		pat.register(Path.PathType.Position, 24.0); //go back two feet
        		pat.Iterate();
        	}
        	//vision.stopPegTracking();
        	
        }catch(Throwable t){
        	CrashTracker.logThrowableCrash(t);
        	throw t;
        }
    }

    public void autonomousPeriodic() {
        try{
        	
        }catch(Throwable t){
        	CrashTracker.logThrowableCrash(t);
        	throw t;
        }
    }

    public void teleopInit(){
        try{
        	resetAllSensors();
        	CrashTracker.logTeleopInit();
        	//drive.updatePID();
        	iteratorEnabled.start();
        	iteratorDisabled.stop();
        	System.out.println("Testing");
        	closeLoopTime=0;
        	drive.disableHeadingMode();
        	RobotMap.gearShifter.set(DoubleSolenoid.Value.kReverse); //Start in low gear
        	RobotMap.gates.set(DoubleSolenoid.Value.kReverse); //Close gates
        	RobotMap.shootControl.setEncPosition(0);
        }catch(Throwable t){
        	CrashTracker.logThrowableCrash(t);
        	throw t;
        }
    }
    
    public void teleopPeriodic() {
        try{

        	if(RobotMap.testerCodeEnabled){
        		testerCode();
        	}
			if(OI.rightStick.getRawButton(OI.GearShiftToggle)){
				if(RobotMap.gearShifter.get().equals(DoubleSolenoid.Value.kForward)){
					RobotMap.gearShifter.set(DoubleSolenoid.Value.kReverse);
					System.out.println("Low gear");
				}else{
					RobotMap.gearShifter.set(DoubleSolenoid.Value.kForward);
					System.out.println("High gear");
				}
				while(OI.rightStick.getRawButton(OI.GearShiftToggle));
				
			}
			if(OI.rightStick.getRawButton(OI.Gates)){
				if(RobotMap.gates.get().equals(DoubleSolenoid.Value.kForward)){
					RobotMap.gates.set(DoubleSolenoid.Value.kReverse);
					System.out.println("Gates closed");
				}else{
					RobotMap.gates.set(DoubleSolenoid.Value.kForward);
					System.out.println("Gates opened");
				}
				while(OI.rightStick.getRawButton(OI.Gates));
			}
			if(OI.rightStick.getRawButton(OI.CompressorToggle)){
				while(OI.rightStick.getRawButton(OI.CompressorToggle));
				if(compressorEnabled){
					//Yes, this is actually necessary. c.isEnabled() just checks if its running, not if a start signal is active.
					//I also can't set a boolean value, I have to use stop() or start()
					compressorEnabled=false;
					RobotMap.c.stop();
					System.out.println("Compressor stopped");
				}else{
					compressorEnabled=true;
					RobotMap.c.start();
					System.out.println("Compressor started");
				}
			}

			
        }catch(Throwable t){
        	CrashTracker.logThrowableCrash(t);
        	throw t;
        }
    }
    
    public void testInit(){
        try{
        	
        }catch(Throwable t){
        	CrashTracker.logThrowableCrash(t);
        	throw t;
        }
    }
    
    public void testPeriodic() {
        try{
        	
        }catch(Throwable t){
        	CrashTracker.logThrowableCrash(t);
        	throw t;
        }
    }
    public void resetAllSensors(){
    	RobotMap.left1.setPosition(0);
    	RobotMap.right1.setPosition(0);
    	RobotState.resetGyro();
    }
    public void writeAllToCSV(){
        RobotMap.writer.writeNext(drive.valString().split("#"), false);
    }
    /**
     * Create CSV file with timestamp for keeping track of values over time
     * This was mainly used for tuning PID values and simply keeping track of outputs/errors over time for easy debugging.
     * 
     * To get the file, you have to install something like puTTY or winSCP. I use winSCP but I'd recommend puTTY instead
     * You then login with the username "admin" and the password is, by default, blank.
     * Then you copy the file over and run a plotter script, or just use excel. I will upload the script I used to github. (python 2.7)
     * You don't need to know python, just follow the same format I did.
     * 
     */
    public void createNewCSV(){
    	try{
    		double t=Timer.getFPGATimestamp();
    		Calendar now = Calendar.getInstance();
			String str = String.valueOf(now.get(Calendar.MONTH))+"."+String.valueOf(now.get(Calendar.DAY_OF_MONTH))+"."
					+String.valueOf(now.get(Calendar.HOUR_OF_DAY))+"."+String.valueOf(now.get(Calendar.MINUTE))+"."+String.valueOf(now.get(Calendar.SECOND));
			File f = new File("/home/lvuser/TestingData"+str+".csv");
			RobotMap.writer = new CSVWriter(new FileWriter(f), ',');
//			String[] tabNames = ("Time#LeftError#RightError#LeftPosition#RightPosition#LeftVelocity#RightVelocity#LeftSetpoint"
//					+ "#RightSetpoint#WeightedLeftError#WeightedRightError#WeightedLeftPosition#WeightedRightPosition"
//					+ "#WeightedLeftVelocity#WeightedRightVelocity#WeightedLeftSetpoint#WeightedRightSetpoint#kP#kI#kD#kFL#kFR#kV#kGP#kGI#kGD#Voltage#Heading#HeadingSetpoint").split("#");
			String[] tabNames = drive.keyString().split("#");
			Timer.delay(.01);
			RobotMap.writer.writeNext(tabNames, true);
			System.out.println("Created new CSV file with name: TestingData"+str+".csv in "+(Timer.getFPGATimestamp()-t)+" Seconds"); 
			//This tends to take around .1 seconds
    	}catch(IOException e){
    		e.printStackTrace();
    	}
    }
    /**
     * Any code I use for testing subsystems goes here, so that we don't destroy the rio CPU during comp
     * 
     * 
     */
    private void testerCode(){
    	if(OI.rightStick.getRawButton(OI.AutoTest)){
    		while(OI.rightStick.getRawButton(OI.AutoTest));
    		pat.setPositionTimeThresh(7.25);
    		pat.register(Path.PathType.Position, -(84-14.5));
    		pat.register(Path.PathType.Heading, 60.0);
    		pat.Iterate();
    		pat.setPositionTimeThresh(7.25);
    		//Timer.delay(.5);
    		trackAndMove();
    	}
    	//RobotMap.shootControl.set(OI.thirdStick.getY());
    	//SmartDashboard.putNumber("ShooterSpeed", RobotState.getShooterSpeed());
    	//SmartDashboard.putNumber("ShooterPosition", RobotMap.shootControl.getEncPosition());
    	//System.out.println(((((RobotMap.ultra.getVoltage()) * 3.47826087) - 0.25)*12.0)-6.0); //For anyone looking at this code, add +9 to get actual inches to it
    	if(OI.rightStick.getRawButton(OI.ResetGyro))
    		RobotState.resetGyro();
    	if(OI.rightStick.getRawButton(OI.PIDDistTest)){
    		while(OI.rightStick.getRawButton(OI.PIDDistTest));
    		pat.register(PathType.Position, -60.0);
    		pat.Iterate();
    	}
    	if(OI.rightStick.getRawButton(OI.PIDAngleTest)){
    		while(OI.rightStick.getRawButton(OI.PIDAngleTest));
    		pat.register(PathType.Heading, 60.0);
    		pat.Iterate();
    	}
    	if(OI.leftStick.getRawButton(OI.shooterPIDTest)){
    		while(OI.leftStick.getRawButton(OI.shooterPIDTest));
    		System.out.println("Shooter PID enabled");
    		RobotMap.shootPID.setOutputRange(-1.0, 0.0);
    		RobotMap.shootPID.setPIDF(
    				SmartDashboard.getNumber("ShootKp", 0.0), 
    				SmartDashboard.getNumber("ShootKi", 0.0), 
    				SmartDashboard.getNumber("ShootKd", 0.0),
    				SmartDashboard.getNumber("ShootKf", 0.0));
    		Timer.delay(.1);
    		SmartDashboard.putNumber("ShootKp", RobotMap.shootPID.getP());
    		SmartDashboard.putNumber("ShootKi", RobotMap.shootPID.getI());
    		SmartDashboard.putNumber("ShootKd", RobotMap.shootPID.getD());
    		SmartDashboard.putNumber("ShootKf", RobotMap.shootPID.getF());
    		RobotMap.shootPID.setSetpoint(1200.0*OI.thirdStick.getY()); //1200 is max speed
    		RobotMap.shootPID.setIZoneRange(0.0, 50.0);
    		RobotMap.shootPID.resetIntegrator();
    		Timer.delay(.2); //3.846E-4
//    		while(!OI.leftStick.getRawButton(OI.shooterPIDTest)){
//    			double output=RobotMap.shootPID.calculate(RobotState.getShooterSpeed());
//    			RobotMap.shootControl.set(output);
//    			SmartDashboard.putNumber("ShooterSpeed", RobotState.getShooterSpeed());
//    			SmartDashboard.putNumber("ShootErrorSum", RobotMap.shootPID.getErrorSum());
//    			SmartDashboard.putNumber("ShootError", RobotMap.shootPID.getSetpoint()-RobotState.getShooterSpeed());
//    			SmartDashboard.putNumber("ShootError2", RobotMap.shootPID.getError());
//    			SmartDashboard.putNumber("ShootOutput", output);
//    			SmartDashboard.putNumber("ShootSetpoint", RobotMap.shootPID.getSetpoint());
//    		}
    		while(OI.leftStick.getRawButton(OI.shooterPIDTest));
    		Timer.delay(.1);
    		System.out.println("Shooter PID disabled");
    		RobotMap.shootControl.set(0);
    	}
    	if(OI.leftStick.getRawButton(OI.HeadingPIDTest)){
    		//pat.register(Path.PathType.Position, 86.94-14.5);
    		pat.register(Path.PathType.Heading, 60.0);
    		//pat.register(Path.PathType.Position, (76.234-14.5)/3.0);
    		pat.Iterate();
    	}
    	//    	if(OI.leftStick.getRawButton(OI.EnablePIDTester) && !tunePID){
    	//		closeLoopTime=Timer.getFPGATimestamp();
    	//		createNewCSV(); //this MUST go before the next line
    	//		tunePID=true;
    	//		drive.setControlState(DriveControlState.velocity);
    	//		while(OI.leftStick.getRawButton(OI.EnablePIDTester));
    	//		
    	//		drive.setSetpoints(-10.0, -10.0);
    	//		drive.setControlState(DriveControlState.position);
    	//		RobotMap.drivePositionLeftPID.calculate(RobotState.getLeftPos());
    	//		RobotMap.drivePositionRightPID.calculate(RobotState.getRightPos());
    	//		RobotMap.drivePositionLeftPID.calculate(RobotState.getLeftPos());
    	//		RobotMap.drivePositionRightPID.calculate(RobotState.getRightPos());
    	////		double leftGain, rightGain;
    	////		leftGain = Math.abs(OI.leftStick.getY())<.15 ? 0 : -OI.leftStick.getY();
    	////		//rightGain = Math.abs(OI.rightStick.getY())<.15 ? 0 : OI.rightStick.getY();
    	////		rightGain=leftGain;
    	//		drive.setSetpoints(76.234-14.5 , 76.234-14.5); //86.94-14.5 //76.234-14.5
    	//		System.out.println(RobotMap.drivePositionLeftPID.getError());
    	//	}
    	if((OI.leftStick.getRawButton(OI.DisablePIDTester) || (RobotMap.drivePositionLeftPID.onTarget() && RobotMap.drivePositionRightPID.onTarget()
    			&& Timer.getFPGATimestamp()-closeLoopTime>10.0)) && tunePID){
    		tunePID=false;
    		drive.setControlState(DriveControlState.regular);
    		try{

    			RobotMap.writer.close();
    		}catch(IOException e){
    			e.printStackTrace();
    		}
    	}
    	if(OI.leftStick.getRawButton(OI.AutoPathTest)){ //9
    		while(OI.leftStick.getRawButton(OI.AutoPathTest));
    		trackAndMove();
    	}
    	if(tunePID){


    		if(OI.leftStick.getRawButton(OI.PIDTuningSnapshot) && !drive.isInHeadingMode()){
    			double currStart=Timer.getFPGATimestamp();


    			double leftGain, rightGain;
    			leftGain = Math.abs(OI.leftStick.getY())<.15 ? 0 : -OI.leftStick.getY();
    			//rightGain = Math.abs(OI.rightStick.getY())<.15 ? 0 : OI.rightStick.getY();
    			rightGain=leftGain;
    			//drive.setSetpoints(RobotMap.EncCountsPerRev*32*OI.leftStick.getY(), RobotMap.EncCountsPerRev*32*OI.rightStick.getY());

    			System.out.println("Driving to "+leftGain*RobotMap.leftMaxIPS+" inches per second");
    			Robot.drive.setControlState(DriveControlState.velocity);
    			Timer.delay(.07);
    			Robot.drive.setSetpoints(leftGain*RobotMap.leftMaxIPS, rightGain*RobotMap.rightMaxIPS);
    			RobotMap.driveVelocityLeftPID.setMinimumTimeToRun(Math.abs(leftGain*RobotMap.leftMaxIPS/(RobotMap.leftMaxIPSPS)));
    			//(inches) / (max inches / second)
    			RobotMap.driveVelocityRightPID.setMinimumTimeToRun(Math.abs(rightGain*RobotMap.rightMaxIPS/(RobotMap.rightMaxIPSPS)));
    			RobotMap.driveVelocityLeftPID.setTolerance(1.5);
    			RobotMap.driveVelocityRightPID.setTolerance(1.5);
    			Timer.delay(.07);
    			boolean hasReachedLeftThresh=false, hasReachedRightThresh=false;
    			while(!(OI.leftStick.getRawButton(OI.DisablePIDTester) || OI.leftStick.getRawButton(OI.PIDTuningSnapshot))){
    				System.out.println("Current Left Error: "+RobotMap.driveVelocityLeftPID.getError()+"Current Right Error: "+RobotMap.driveVelocityRightPID.getError());
    				if(Math.abs(RobotMap.driveVelocityLeftPID.getError())<2 && !hasReachedLeftThresh){
    					hasReachedLeftThresh=true;
    					System.out.println("Time to reach left threshold: "+(Timer.getFPGATimestamp()-currStart));
    					SmartDashboard.putNumber("LeftCloseLoopTime", (Timer.getFPGATimestamp()-currStart));
    				}
    				if(Math.abs(RobotMap.driveVelocityRightPID.getError())<2 && !hasReachedRightThresh){
    					hasReachedRightThresh=true;
    					System.out.println("Time to reach right threshold: "+(Timer.getFPGATimestamp()-currStart));
    					SmartDashboard.putNumber("RightCloseLoopTime", (Timer.getFPGATimestamp()-currStart));
    				}
    				//Robot.drive.updatePID();
    				writeAllToCSV();
    			}
    			Robot.drive.resetAll();
    			System.out.println("Time: " + (Timer.getFPGATimestamp()-currStart)+" Seconds");


    			//drive.setSetpoints(leftGain*30.0, rightGain*30.0);
    			while(OI.leftStick.getRawButton(OI.PIDTuningSnapshot));
    		}

    		//drive.setSetpoints(OI.leftStick.getY()*1000.0, OI.rightStick.getY()*1000.0);
    		//drive.setSetpoints(RobotMap.EncCountsPerRev*8*OI.leftStick.getY(), RobotMap.EncCountsPerRev*8*OI.rightStick.getY());
    		//drive.updatePID();
    		//		
    		//		if(closeLoopTime!=0) writeAllToCSV();

    	}
    }
    private void trackAndMove(){
//		pat.register(PathType.Position, -40.0);
//		pat.Iterate();
    	double angThresh=0.5;
    	//correct angle
		vision.startPegTracking();
		while(vision.getCurrentIteration()<=3);
		double ang = -vision.getAveragePegAngle();
		double dist= ((-vision.getAveragePegDistance())-((76.234-14.5)))/2.0;
		vision.stopPegTracking();
		if(Math.abs(ang)>angThresh){
			pat.register(PathType.Heading, 1.05*ang);
			pat.Iterate();
		}
		
		
		//third of distance
//		vision.startPegTracking();
//		while(vision.getCurrentIteration()<=14);
//		double dist=-vision.getAveragePegDistance();
//		vision.stopPegTracking();
		pat.register(Path.PathType.Position, dist/3.0);
//        		pat.register(Path.PathType.Heading, 60.0);
		pat.Iterate();
		
		//correct angle
		vision.startPegTracking();
		while(vision.getCurrentIteration()<=2);
		ang=-vision.getAveragePegAngle();
		vision.stopPegTracking();
		if(Math.abs(ang)>angThresh){
			pat.register(PathType.Heading, ang);
			pat.Iterate();
		}
		
		//second third of distance
		//vision.startPegTracking();
		//while(vision.getCurrentIteration()<=7);
		//dist=-vision.getCalculatedPegDistance();
		//vision.stopPegTracking();
		pat.register(Path.PathType.Position, dist/3.0);
//        		pat.register(Path.PathType.Heading, 60.0);
		pat.Iterate();
		
		//correct angle
		vision.startPegTracking();
		while(vision.getCurrentIteration()<=2);
		ang=-vision.getAveragePegAngle();
		vision.stopPegTracking();
		if(Math.abs(ang)>angThresh){
			pat.register(PathType.Heading, ang);
			pat.Iterate();
		}
		
		//last third of distance
		//vision.startPegTracking();
		//while(vision.getCurrentIteration()<=7);
		//dist=-vision.getCalculatedPegDistance();
		//vision.stopPegTracking();
		pat.register(Path.PathType.Position, dist/3.0);
//        		pat.register(Path.PathType.Heading, 60.0);
		pat.Iterate();

		
		//correct angle
		vision.startPegTracking();
		while(vision.getCurrentIteration()<=2);
		ang=-vision.getAveragePegAngle();
		vision.stopPegTracking();
		if(Math.abs(ang)>angThresh){
			pat.register(PathType.Heading, 1.1*ang);
			pat.Iterate();
		}
		
		//After this we'd use an ultrasonic sensor to do the rest of the work
		double ult = ((((RobotMap.ultra.getVoltage()) * 3.47826087) - 0.25)*12.0)-6.0;
		ult = ult<4.0 ? 0.0 : -(ult+10.0);
		pat.register(PathType.Position, ult);
		pat.Iterate();
    }
}
