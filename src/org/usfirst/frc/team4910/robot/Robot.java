
package org.usfirst.frc.team4910.robot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Calendar;

import org.omg.PortableInterceptor.SYSTEM_EXCEPTION;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team4910.iterations.*;
import org.usfirst.frc.team4910.subsystems.DriveTrain.DriveControlState;
import org.usfirst.frc.team4910.subsystems.*;
import org.usfirst.frc.team4910.util.CrashTracker;
import org.usfirst.frc.team4910.util.GyroHelper;
import org.usfirst.frc.team4910.util.Path;
import org.usfirst.frc.team4910.util.Path.PathType;
import org.usfirst.frc.team4910.util.Peg;

import com.opencsv.CSVWriter;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends IterativeRobot {
    
	Thread visionThread;
	Iterator iteratorEnabled = new Iterator();
	Iterator iteratorDisabled = new Iterator();
	public static Jetson jetson;
	public static DriveTrain drive;
	public static Shooter sh;
	static Elevator elev;
	static VisionProcessor vision;
	static Climber climb;
	static OI oi;
	public static Path pat;
	public static SendableChooser<String> autoChoose;
	static SendableChooser<String> gearAutoChoose;
	static boolean compressorEnabled;
	private boolean tunePID=false;
	public static double closeLoopTime=0;
	
    public void robotInit() {
        try{
        	double initTime=Timer.getFPGATimestamp();
        	RobotMap.init();
        	oi = OI.getInstance();
        	drive = DriveTrain.getInstance();
        	sh = Shooter.getInstance();
        	elev = Elevator.getInstance();
        	vision = VisionProcessor.getInstance();
        	climb = Climber.getInstance();
        	pat = new Path();
        	jetson = new Jetson();
        	jetson.startZMQ();
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
    		autoChoose.addDefault("JetsonTest","JetsonTest");
    		autoChoose.addObject("Red Left", "Red Left");
    		autoChoose.addObject("Red Middle", "Red Middle");
    		autoChoose.addObject("Red Right", "Red Right");
    		autoChoose.addObject("Blue Left", "Blue Left");
    		autoChoose.addObject("Blue Middle", "Blue Middle");
    		autoChoose.addObject("Blue Right", "Blue Right");
    		autoChoose.addObject("Just go forward", "Just go forward");
    		gearAutoChoose = new SendableChooser<String>();
    		gearAutoChoose.addObject("Open gates in auto", "Open gates in auto");
    		gearAutoChoose.addDefault("Keep gates closed in auto", "Keep gates closed in auto");
    		SmartDashboard.putData("Auto mode", autoChoose);
    		SmartDashboard.putData("Gates in auto chooser", gearAutoChoose);
    		System.out.println("Robot Init time: "+(Timer.getFPGATimestamp()-initTime));
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
    
	public void autonomousInit() {
        try{
        	resetAllSensors();
        	CrashTracker.logAutoInit();
        	iteratorEnabled.start();
        	iteratorDisabled.stop();
        	RobotMap.c.stop();
        	Timer.delay(.08);
        	pat.reset();
        	RobotMap.gearShifter.set(DoubleSolenoid.Value.kReverse); //Start in low gear
        	RobotMap.gates.set(DoubleSolenoid.Value.kReverse); //Close gates
        	Timer.delay(.1);
        	//vision.startPegTracking();
        	//The point of this is to drive partly to the peg, and then use vision tracking to correct itself,
        	//since we can't expect the human to place it in the right place each time
        	boolean nothing=false;
        	double initialTurnAngleLeft = RobotMap.isCompBot ? 60.0: 60.0; //We didn't have enough time to fully tune PID but it was close enough to this
        	double initialTurnAngleRight = RobotMap.isCompBot ? 60.0 : 60.0;
        	//double initDist= RobotMap.isCompBot ? 80 : 94.5;
        	double initDist= RobotMap.isCompBot ? 88.0 : 82;
        	switch((String)autoChoose.getSelected()){
        	case "JetsonTest":
        		RobotMap.gates.set(DoubleSolenoid.Value.kForward); 
        		Peg.PegLoc loc;
        		
        		for(int i = 0; i < 10; i++) {
        			loc = jetson.receave(); // Get the peg Location from the Jetson
        			System.out.println("Peg X Location: " + loc.getPegX());
        			System.out.println("Peg Y Location: " + loc.getPegY());
        			nothing=true;
        		}
        		break;
        	case "Red Left":
        		visionAlternateAlternate(-initialTurnAngleLeft);
        		//visionAlternate(-initialTurnAngleLeft);
        		
////        		pat.setPositionTimeThresh(7.25);
//        		pat.register(Path.PathType.Position, -(88-14.5));
//        		pat.register(Path.PathType.Heading, -initialTurnAngleLeft);
//        		pat.Iterate();
//        		
//        		vision.startPegTracking();
//        		while(vision.getCurrentIteration()<=2);
//        		double a=-vision.getAveragePegAngle();
//        		vision.stopPegTracking();
//        		pat.register(PathType.Heading, a);
//        		pat.register(Path.PathType.Position, (-(65.5-14.5))/2.0);
//        		pat.Iterate();        		
//        		
//        		vision.startPegTracking();
//        		while(vision.getCurrentIteration()<=2);
//        		double a2=-vision.getAveragePegAngle();
//        		vision.stopPegTracking();
//        		pat.register(PathType.Heading, a2);
//        		pat.register(Path.PathType.Position, (-(65.5-14.5))/2.0);
//        		pat.Iterate();
//        		
        		
//        		pat.setPositionTimeThresh(7.25);
        		//trackAndMove();
        		break;
        	case "Red Middle":
        		middleAutoAlternate();
        		break;
        	case "Red Right":
        		visionAlternateAlternate(initialTurnAngleRight);
//        		pat.setPositionTimeThresh(7.25);
//        		pat.register(Path.PathType.Position, -(initDist-14.5));
//        		pat.register(Path.PathType.Heading, initialTurnAngleRight);
//        		pat.Iterate();
//        		pat.setPositionTimeThresh(7.25);
        		//trackAndMove();
        		break;
        	case "Blue Left":
        		//visionAlternate(-initialTurnAngleLeft);
        		visionAlternateAlternate(-initialTurnAngleLeft);
//        		pat.register(Path.PathType.Position, -(88-14.5));
//        		pat.register(Path.PathType.Heading, -initialTurnAngleLeft);
//        		pat.Iterate();
//        		
//        		vision.startPegTracking();
//        		while(vision.getCurrentIteration()<=2);
//        		double a3=-vision.getAveragePegAngle();
//        		vision.stopPegTracking();
//        		pat.register(PathType.Heading, a3);
//        		pat.register(Path.PathType.Position, (-(65.5-14.5))/2.0);
//        		pat.Iterate();        		
//        		
//        		vision.startPegTracking();
//        		while(vision.getCurrentIteration()<=2);
//        		double a4=-vision.getAveragePegAngle();
//        		vision.stopPegTracking();
//        		pat.register(PathType.Heading, a4);
//        		pat.register(Path.PathType.Position, (-(65.5-14.5))/2.0);
//        		pat.Iterate();
//        		
        		
//        		pat.setPositionTimeThresh(7.25);
//        		pat.register(Path.PathType.Position, -(initDist-14.5));
//        		pat.register(Path.PathType.Heading, -initialTurnAngleLeft);
//        		pat.Iterate();
//        		pat.setPositionTimeThresh(7.25);
        		//trackAndMove();
        		break;
        	case "Blue Middle":
        		middleAutoAlternate();
        		break;
        	case "Blue Right":
        		visionAlternateAlternate(initialTurnAngleLeft);
//        		pat.setPositionTimeThresh(7.25);
//        		pat.register(Path.PathType.Position, -(initDist-14.5));
//        		pat.register(Path.PathType.Heading, initialTurnAngleRight);
//        		pat.Iterate();
//        		pat.setPositionTimeThresh(7.25);
        		//trackAndMove();
        		break;
        	case "Just go forward":
        		pat.setPositionTimeThresh(7.25);
        		pat.register(Path.PathType.Position, -90);
        		nothing=true;
        		pat.Iterate();
        	default:
        		break;
        	}
        	if(!nothing && RobotMap.isCompBot && ((String)gearAutoChoose.getSelected()).equals("Open gates in auto")){
        		RobotMap.gates.set(DoubleSolenoid.Value.kForward); //Open gates
        		RobotMap.gearShifter.set(DoubleSolenoid.Value.kForward);
        		Timer.delay(.9); //was .56
        		pat.register(Path.PathType.Position, 24.0); //go back two feet
        		pat.Iterate();
        	
        		
        		if(((String)autoChoose.getSelected()).contains("Left")){
        			pat.register(Path.PathType.Heading, 58.0);
        			pat.register(PathType.Position, -(90.0+(67.0+12.0-24.0))); 
        			//90 is dist from base line to neutral zone, 67 is half the neutral zone, and 24 is the ending distance from the base line after backing up
        			//and the 12.0 (or whatever I add to 67.0 if I don't update this comment) is the tuned value
        			pat.Iterate();
        		}
        	}
        	RobotMap.c.start();
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
        	RobotMap.c.start();
        	RobotMap.gearShifter.set(DoubleSolenoid.Value.kReverse);
        	RobotMap.gearShifter.set(DoubleSolenoid.Value.kForward); //Start in high gear, but switch quickly so it actually compresses
        	RobotMap.gates.set(DoubleSolenoid.Value.kReverse); //Close gates
        	//RobotMap.shootControl.setEncPosition(0);
        }catch(Throwable t){
        	CrashTracker.logThrowableCrash(t);
        	throw t;
        }
    }
    
    public void teleopPeriodic() {
        try{
//        	if(OI.thirdStick.getRawButton(OI.cameraDebugTest) /*&& !RobotMap.isCompBot*/){
//        		while(OI.thirdStick.getRawButton(OI.cameraDebugTest));
//        		if(RobotMap.isCompBot){
//        			visionAlternate(-60.0);
//            		RobotMap.gates.set(DoubleSolenoid.Value.kForward); //Open gates
//        		}else{
//        			visionAlternatePracticeBot(-60.0);
//        		}
//        		
////        		RobotMap.gates.set(DoubleSolenoid.Value.kForward); //Open gates
//        		RobotMap.gearShifter.set(DoubleSolenoid.Value.kForward);
//        		Timer.delay(.16);
//        		pat.register(Path.PathType.Position, 24.0); //go back two feet
//        		pat.Iterate();
//            	RobotMap.gearShifter.set(DoubleSolenoid.Value.kReverse);
//            	
//            	
//            	pat.register(Path.PathType.Heading, 58.0);
//            	pat.register(PathType.Position, -(90.0+(67.0+12.0-24.0))); 
//            	//90 is dist from base line to neutral zone, 67 is half the neutral zone, and 24 is the ending distance from the base line after backing up
//            	//and the 12.0 (or whatever I add to 67.0 if I don't update this comment) is the tuned value
//            	pat.Iterate();
//            	
//            	RobotMap.gearShifter.set(DoubleSolenoid.Value.kForward); //Start in high gear, but switch quickly so it actually compresses
//            	
//            	if(RobotMap.isCompBot)
//            		RobotMap.gates.set(DoubleSolenoid.Value.kReverse); //Close gates
////            	
//        	}
        	if(RobotMap.testerCodeEnabled){
        		testerCode();
        	}
        	
        	//check for c.enabled() so we aren't start()ing as we're compressing
			if(RobotMap.c.enabled()
					&&(
						RobotMap.gearShifter.get().equals(DoubleSolenoid.Value.kForward)
						|| Math.abs(RobotMap.shootControl.get())>.2 
						|| elev.getElevatorState().equals(Elevator.ElevatorState.Running)
						|| Math.abs(RobotMap.climbControl.get())>.2)
					){
				RobotMap.c.stop();
			}else{
				RobotMap.c.start();
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
//			if(OI.rightStick.getRawButton(OI.CompressorToggle)){
//				while(OI.rightStick.getRawButton(OI.CompressorToggle));
//				if(compressorEnabled){
//					//Yes, this is actually necessary. c.isEnabled() just checks if its running, not if a start signal is active.
//					//I also can't set a boolean value, I have to use stop() or start()
//					compressorEnabled=false;
//					RobotMap.c.stop();
//					System.out.println("Compressor stopped");
//				}else{
//					compressorEnabled=true;
//					RobotMap.c.start();
//					System.out.println("Compressor started");
//				}
//			}

			
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
     *  ////////////////Name changed after columbus, you can keep the "TestingDataPostColumbus" name, or change it back to "TestingData"////////////////
     */
    public void createNewCSV(){
    	try{
    		double t=Timer.getFPGATimestamp();
    		Calendar now = Calendar.getInstance();
			String str = String.valueOf(now.get(Calendar.MONTH))+"."+String.valueOf(now.get(Calendar.DAY_OF_MONTH))+"."
					+String.valueOf(now.get(Calendar.HOUR_OF_DAY))+"."+String.valueOf(now.get(Calendar.MINUTE))+"."+String.valueOf(now.get(Calendar.SECOND));
			File f = new File("/home/lvuser/TestingDataPostColumbus"+str+".csv");
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
    	if(OI.thirdStick.getRawButton(OI.cameraDebugTest) /*&& !RobotMap.isCompBot*/){
    		while(OI.thirdStick.getRawButton(OI.cameraDebugTest));
			RobotMap.gearShifter.set(DoubleSolenoid.Value.kReverse);
    		visionAlternateAlternate(-60.0);
    		
//    		RobotMap.gates.set(DoubleSolenoid.Value.kForward); //Open gates
//    		RobotMap.gearShifter.set(DoubleSolenoid.Value.kForward);
//    		Timer.delay(.16);
//    		pat.register(Path.PathType.Position, 24.0); //go back two feet
//    		pat.Iterate();
//        	RobotMap.gearShifter.set(DoubleSolenoid.Value.kReverse);
//        	RobotMap.gearShifter.set(DoubleSolenoid.Value.kForward); //Start in high gear, but switch quickly so it actually compresses
//        	RobotMap.gates.set(DoubleSolenoid.Value.kReverse); //Close gates
    	}
    	if(OI.thirdStick.getRawButton(OI.forwardAutoTest) /*&& !RobotMap.isCompBot*/){
    		while(OI.thirdStick.getRawButton(OI.forwardAutoTest));
//    		
//    		RobotMap.gearShifter.set(DoubleSolenoid.Value.kReverse); //Start in low gear
//        	Timer.delay(.05);
//    		pat.register(PathType.Position, -94+38.5);
//    		pat.Iterate();
//    		
//        	vision.startPegTracking();
//        	while(vision.getCurrentIteration()<=4);
//        	double vang=-vision.getAveragePegAngle();
//        	//double vdist=-vision.getAveragePegDistance();
//        	vision.stopPegTracking();
//        	pat.register(PathType.Heading, vang);
//        	pat.Iterate();
//    		
//    		Timer.delay(.07);
//    		double ult = ((((RobotMap.ultra.getVoltage()) * 3.47826087) - 0.25)*12.0)-6.0;
//    		pat.register(PathType.Position, -12-ult);
//    		pat.Iterate();
//    		
//    		
//    		RobotMap.gates.set(DoubleSolenoid.Value.kForward); //Open gates
//    		RobotMap.gearShifter.set(DoubleSolenoid.Value.kForward);
//    		Timer.delay(.16);
//    		pat.register(Path.PathType.Position, 24.0); //go back two feet
//    		pat.Iterate();
//        	RobotMap.gearShifter.set(DoubleSolenoid.Value.kReverse);
//        	RobotMap.gearShifter.set(DoubleSolenoid.Value.kForward); //Start in high gear, but switch quickly so it actually compresses
//        	RobotMap.gates.set(DoubleSolenoid.Value.kReverse); //Close gates
    		if(!RobotMap.isCompBot){
    			RobotMap.gearShifter.set(DoubleSolenoid.Value.kReverse);
    			middleAutoAlternate();
    		}
    	}
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
			pat.register(PathType.Heading, 0.6*ang);
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
			pat.register(PathType.Heading, 1.3*ang);
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
			pat.register(PathType.Heading, 1.1*ang);
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
			pat.register(PathType.Heading, 1.5*ang-2.8);
			pat.Iterate();
		}
		
		//After this we'd use an ultrasonic sensor to do the rest of the work
		if(RobotMap.isCompBot){
			double ult = ((((RobotMap.ultra.getVoltage()) * 3.47826087) - 0.25)*12.0)-6.0;
			ult = ult<4.0 ? 0.0 : -(ult+10.5);
			pat.register(PathType.Position, ult);
			pat.Iterate();
		}
    }
    private void middleAuto(){
    	//TODO: remember to switch all "true" statements before comp to RobotMap.isCompBot
		if(true){
    	double initialDist= true ? -73.0 : -58;
		double denom=1.8;
		pat.register(Path.PathType.Position, initialDist/denom);
		pat.Iterate();
		
		double ang=0.0, dist, angThresh=0.1;
		//correct angle
		vision.startPegTracking();
		while(vision.getCurrentIteration()<=4);
		ang=-vision.getAveragePegAngle();
		dist=2.0-vision.getAveragePegDistance();
		vision.stopPegTracking();
		if(Math.abs(ang)>angThresh){
			pat.register(PathType.Heading, 1.2*ang);
			pat.Iterate();
		}
		dist = 3+((dist+((initialDist-(initialDist/denom))))/2.0); //average with expected
		dist=Math.min(-35.0, dist);
		pat.register(Path.PathType.Position, dist);
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
		
//		double ult = ((((RobotMap.ultra.getVoltage()) * 3.47826087) - 0.25)*12.0)-6.0;
//		if(true){
//			ult = ult<4.0 ? -13.0 : -(ult+11.0);
//		}else{
//			ult=-12.0;
//		}
		pat.register(PathType.Position, -13.0);
		pat.Iterate();
		}else{
			middleAutoAlternate();
		}
    }
    /**
     * I intend for this method to use known variables and vision at the very beginning of 
     * the match to calculate my expected first and last distances.
     * This was written after the first district competition, but before the second one.
     * 
     * Apparently this works very well. The camera can hardly even see it so we had to rotate the camera and just hope our tuned values
     * would work, and they somewhat did. If they didn't, we would've done "vision.getAveragePegAngle() +/- 3"
     * 
     * 
     * @param initAng The pegs angle. This does not have to be explicitly equal to +/-60, but it's like this because the peg-vector is +/- 60 deg from the wall.
     */
    private void visionAlternate(double initAng){
    	if(RobotMap.isCompBot){
    	RobotMap.gearShifter.set(DoubleSolenoid.Value.kReverse); //Start in low gear
    	Timer.delay(.05);
    	vision.startPegTracking();
    	double vang=0.0,vdist=0.0, vAvgOffset; final double initDist=125.5-38.5; //38.5 is robot length, 131.0 is dist from peg to wall
    	while(vision.getCurrentIteration()<=4);
    	vang=-vision.getAveragePegAngle(); //this is always backwards, by the way
    	vang+=Math.signum(vang)*14.0; //
    	vdist=vision.getAveragePegDistance(); //this is only positive for the purpose of the calculation
    	vision.stopPegTracking();
    	double offsetDistCalc=Math.signum(vang)*Math.sqrt(vdist*vdist-initDist*initDist); //calculates distance based on distance
    	double offsetAngCalc=initDist*Math.tan(vang*Math.PI/180.0); //calculates distance based on angle
    	//vAvgOffset = (240.0*offsetAngCalc+320.0*offsetDistCalc)/560.0; //weighted average to account for either:
    																	//camera distortion, top-bottom or left-right noise shifting the image center,
    																	//and whatever linear regression messes up on
    	vAvgOffset=offsetAngCalc; //Apparently the offsetDistCalc doesn't return a good number when its at an angle.
    							//This makes some logical sense, and it'd probably return a better value if it were a
    							//higher resolution or zoomed in more.
    	double firstDist=initDist-31.0-Math.signum(vAvgOffset)*vAvgOffset/Math.tan(initAng*Math.PI/180.0); //I think this 31 came out of the same place 19.25 did
    	double secondDist=19.25+Math.hypot(vAvgOffset, vAvgOffset/Math.tan(initAng*Math.PI/180.0));
    	firstDist=-16.25-firstDist; //19.25, 9.625 (19.25 is 1/4th our robot length, I don't quite understand why this works)
    	secondDist=45.0-secondDist; //yes 12.0 should be positive (12.0 came out of nowhere as well)
    	//System.out.println(vang+" "+vdist+" "+offsetDistCalc+" "+offsetAngCalc+" "+vAvgOffset);
    	System.out.println("First Dist: "+firstDist+"\nTurning angle: "+initAng+"\nSecond Dist: "+secondDist);
    	
    	
//    	
//    	//check if it's giving the right values before doing this
    	
    	pat.register(PathType.Position, firstDist);
    	pat.Iterate();

    	pat.register(PathType.Heading, initAng);
		pat.Iterate();
    	vision.startPegTracking();
    	while(vision.getCurrentIteration()<=4);
    	vang=-vision.getAveragePegAngle();
    	vdist=-vision.getAveragePegDistance();
    	vision.stopPegTracking();
    	pat.register(PathType.Heading, vang);
    	pat.register(PathType.Position, ((2*secondDist+vdist)/3.0)/2.0); //Bias for mathematically calculated values to correct for distortion
    	pat.Iterate();
    	
		vision.startPegTracking();
		while(vision.getCurrentIteration()<=2);
		double ang=-vision.getAveragePegAngle();;
		//double ult = ((((RobotMap.ultra.getVoltage()) * 3.47826087) - 0.25)*12.0)-6.0;
		vdist=-4.0-vision.getAveragePegDistance(); //In theory this shouldn't work, since it'd go the distance 
												//to the peg + the big ultrasonic reading + 4 more inches, but I blame that error
												//on the fact that we had to tilt the camera and the values it gives are now definitely off.
		vision.stopPegTracking();
		//if(Math.abs(ang)>1.0){ //If I want to add this back, I have to split this into two loops so it runs the position one
			pat.register(PathType.Heading, ang);
			pat.register(PathType.Position, ((2*secondDist+vdist)/3.0)/2.0);
			pat.Iterate();
		//}
    	}else{
    		visionAlternatePracticeBot(initAng);
    	}
    }
    /**
     * Goes up until it's within a foot, corrects itself with vision, and uses the ultrasonic sensor to do the rest
     * 
     * Apparently I forgot to make middle auto do "middleAutoAlternate" and only did middleAuto(), and that somehow worked for a couple matches
     * But then it stopped working and so I switched back to this method, which we know worked at our build site, and it did not work.
     * 
     * I don't exactly know what to do here, but I'll test it out on practice bot some and then rework it at states.
     * 
     */
    private void middleAutoAlternate(){
    
		pat.register(PathType.Position, -94+38.5);
		pat.Iterate();
		
    	vision.startPegTracking();
    	while(vision.getCurrentIteration()<=4);
    	double vang=-vision.getAveragePegAngle();
    	//double vdist=-vision.getAveragePegDistance();
    	vision.stopPegTracking();
    	//vang+=4.0;
    	//vang = vang<0 ? vang-4.0 : vang; //if left, get rid of -4
    	pat.register(PathType.Heading, vang);
    	pat.Iterate();
		
		Timer.delay(.07);
		double ult = ((((RobotMap.ultra.getVoltage()) * 3.47826087) - 0.25)*12.0)-6.0;
		pat.register(PathType.Position, -11.5-ult); //was -12, then -7.5
		pat.Iterate();
		
		
		
    }
    
    /**
     * If I don't include compatability for practice bot, the mentors will get confused again.
     * 
     * @see #visionAlternate
     */
    private void visionAlternatePracticeBot(double initAng){
    	RobotMap.gearShifter.set(DoubleSolenoid.Value.kReverse); //Start in low gear
    	Timer.delay(.05);
    	vision.startPegTracking();
    	double vang=0.0,vdist=0.0, vAvgOffset; final double initDist=125.5-38.5; //38.5 is robot length, 131.0 is dist from peg to wall
    	while(vision.getCurrentIteration()<=4);
    	vang=-vision.getAveragePegAngle(); //this is always backwards, by the way
    	vdist=vision.getAveragePegDistance(); //this is only positive for the purpose of the calculation
    	vision.stopPegTracking();
    	double offsetDistCalc=Math.signum(vang)*Math.sqrt(vdist*vdist-initDist*initDist); //calculates distance based on distance
    	double offsetAngCalc=initDist*Math.tan(vang*Math.PI/180.0); //calculates distance based on angle
    	vAvgOffset = (240.0*offsetAngCalc+320.0*offsetDistCalc)/560.0; //weighted average to account for either:
    																	//camera distortion, top-bottom or left-right noise shifting the image center,
    																	//and whatever linear regression messes up on
    	vAvgOffset=offsetAngCalc; //tester code
    	double firstDist=initDist-31.0-Math.signum(vAvgOffset)*vAvgOffset/Math.tan(initAng*Math.PI/180.0);
    	double secondDist=19.25+Math.hypot(vAvgOffset, vAvgOffset/Math.tan(initAng*Math.PI/180.0));
    	firstDist=-14.4375-firstDist; //19.25, 9.625
    	secondDist=0.0-secondDist; //yes 3.5 should be positive
    	//System.out.println(vang+" "+vdist+" "+offsetDistCalc+" "+offsetAngCalc+" "+vAvgOffset);
    	System.out.println("First Dist: "+firstDist+"\nTurning angle: "+initAng+"\nSecond Dist: "+secondDist);
    	//check if it's giving the right values before doing this
    	pat.register(PathType.Position, firstDist);
    	pat.register(PathType.Heading, initAng);
		pat.Iterate();
    	vision.startPegTracking();
    	while(vision.getCurrentIteration()<=4);
    	vang=-vision.getAveragePegAngle();
    	vdist=-vision.getAveragePegDistance();
    	vision.stopPegTracking();
    	pat.register(PathType.Heading, vang);
    	pat.register(PathType.Position, ((2*secondDist+vdist)/3.0)/2.0); //Bias for mathematically calculated values to correct for distortion
    	pat.Iterate();
    	
		vision.startPegTracking();
		while(vision.getCurrentIteration()<=2);
		double ang=-vision.getAveragePegAngle();;
		double ult = ((((RobotMap.ultra.getVoltage()) * 3.47826087) - 0.25)*12.0)-6.0;
		vdist=-ult-vision.getAveragePegDistance();
		vision.stopPegTracking();
		//if(Math.abs(ang)>1.0){
			pat.register(PathType.Heading, ang);
			pat.register(PathType.Position, ((2*secondDist+vdist)/3.0)/2.0);
			pat.Iterate();
		//}
    	
		
    }
    /**
     * Goes some amount of inches, then runs middleAutoAlternate() code
     * @param initAng -60 is left (turning right), 60 is right (turning left)
     */
    private void visionAlternateAlternate(double initAng){
    	
    	
		pat.register(PathType.Position, -106.0+22.25);
		pat.register(Path.PathType.Heading, initAng);
		pat.Iterate();
		
		
		if(((String)autoChoose.getSelected()).contains("Right")){
			vision.startPegTracking();
    		while(vision.getCurrentIteration()<=4);
    		double vang=-vision.getAveragePegAngle(); //negative for some reason (I don't trust it)
    		//double vdist=-vision.getAveragePegDistance();
    		vision.stopPegTracking();
    		pat.register(PathType.Heading, -vang);
    		pat.Iterate();
		}else{ //normal case
			vision.startPegTracking();
    		while(vision.getCurrentIteration()<=4);
    		double vang=vision.getAveragePegAngle(); //normal
    		//double vdist=-vision.getAveragePegDistance();
    		vision.stopPegTracking();
    		pat.register(PathType.Heading, -vang);
    		pat.Iterate();
		}
    	
		pat.register(PathType.Position, -18.75); //35
		pat.Iterate();
		Timer.delay(.3);
    	
		
		if(((String)autoChoose.getSelected()).contains("Right")){
	    	vision.startPegTracking();
	    	while(vision.getCurrentIteration()<=4);
	    	double vang2=vision.getAveragePegAngle(); //this will probably correct whatever the first one does wrong
	    	//double vdist=-vision.getAveragePegDistance();
	    	vision.stopPegTracking();
	    	pat.register(PathType.Heading, vang2);
	    	pat.Iterate();
		}else{ //normal case
	    	vision.startPegTracking();
	    	while(vision.getCurrentIteration()<=4);
	    	double vang2=-vision.getAveragePegAngle(); //normal case as well for some reason
	    	//double vdist=-vision.getAveragePegDistance();
	    	vision.stopPegTracking();
	    	pat.register(PathType.Heading, vang2);
	    	pat.Iterate();
		}
		
		Timer.delay(.07);
		double ult = ((((RobotMap.ultra.getVoltage()) * 3.47826087) - 0.25)*12.0)-6.0;
		pat.register(PathType.Position, -7.5-ult); //was -12
		pat.Iterate();
    	
    }
    
}
