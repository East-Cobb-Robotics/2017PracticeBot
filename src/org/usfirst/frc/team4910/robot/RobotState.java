package org.usfirst.frc.team4910.robot;

import org.usfirst.frc.team4910.iterations.Iterate;
import org.usfirst.frc.team4910.subsystems.DriveTrain;
import org.usfirst.frc.team4910.util.ContinuousAngleTracker;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 * @author Jason Cobb
 *
 * This is meant to keep track of sensors and robot position, so we aren't recalculating multiple times.
 * While this may seem like a useless micro-optimization at first, implementing this will save both time and load.
 * This will also keep track of all robot parts e.g. shooter speed.
 * 
 * Idea and class name stolen from team 254, but almost none of the class contents are based off their code.
 */
public class RobotState {
	//I keep navX stuff separate just so I can test accuracy.
	//Chances are, we're going to average the results for compbot
	private static double posX=0.0, posY=0.0, posXNavX=0.0, posYNavX=0.0, lastLeftPos=0.0, lastRightPos=0.0, currentLeftPos=0.0, currentRightPos=0.0;
	private static double accelerationX=0.0, accelerationY=0.0, jerkX=0.0, jerkY=0.0; //likely unused
	private static double lastAccelX=0.0, lastAccelY=0.0;
	private static double leftSpeed, rightSpeed=0.0; 
	private static double navxFullHeading=0.0;
	private static double spigHeading=0.0;
	private static double spigHeadingOffset=0.0; //Value heading is subtracted by (when reset)
	private static double protectedSpigHeading=0.0;//This can only be zeroed at the beginning of the match
												//or whenever enabled, otherwise it will always give a full heading
	private static double protectedLeftEnc=0.0, protectedRightEnc=0.0, leftEncOffset=0.0, rightEncOffset=0.0;
	
	private static double shooterSpeed=0.0; //Inaccurate. I think we broke the CIMcoder. I wouldn't recommend it anyway.
	private static ContinuousAngleTracker fusedAngle = new ContinuousAngleTracker();
	private static double time;
	private static double iteration=0.0;
	
	protected static final Iterate iter = new Iterate(){

		@Override
		public void init() {
			RobotMap.left1.setEncPosition(0);
			RobotMap.right1.setEncPosition(0);
			RobotMap.spig.reset();
			reset();
			iteration=0.0;
			//fusedAngle.setAngleAdjustment(RobotMap.navxGyro.getFusedHeading());
			posX=posY=posXNavX=posYNavX=lastLeftPos=lastRightPos=currentLeftPos=currentRightPos=protectedSpigHeading=spigHeadingOffset=spigHeading=0.0;
			protectedLeftEnc=protectedRightEnc=0.0; //Implement later
			leftEncOffset=rightEncOffset=0.0;
			time = Timer.getFPGATimestamp();
			//TODO: check if we're just now coming out of auto so we can save any valuable coordinate value
			//low priority - auto is the only real time where coordinates matter
		}

		@Override
		public void exec() {
			synchronized(this){
				protectedSpigHeading=RobotMap.spig.getAngle();
				protectedLeftEnc = DriveTrain.countsToInches(-RobotMap.left1.getEncPosition());
				protectedRightEnc = DriveTrain.countsToInches(RobotMap.right1.getEncPosition());
				spigHeading=protectedSpigHeading-spigHeadingOffset;
				//fusedAngle.nextAngle(RobotMap.navxGyro.getFusedHeading());
				navxFullHeading = (double)-fusedAngle.getAngle();
				leftSpeed = DriveTrain.rpmToInchesPerSecond(RobotMap.left1.getSpeed());
				rightSpeed = DriveTrain.rpmToInchesPerSecond(RobotMap.right1.getSpeed());
				
				currentLeftPos = protectedLeftEnc-leftEncOffset;
				currentRightPos = protectedRightEnc-rightEncOffset;
				accelerationX = 385.827*RobotMap.RIOAccel.getX(); //inches per second^2
				accelerationY = 385.827*RobotMap.RIOAccel.getY();
				//shooterSpeed = 600.0*(RobotMap.shootControl.getEncVelocity()/80.0);
		        shooterSpeed = -RobotMap.shootControl.getSpeed();
				//We want it so that relative to the start, x is left and right, y is up and down.
		        //Also, the navX gyro is backwards
		        posY += (Math.cos(Math.toRadians(spigHeading))
		        		*(protectedLeftEnc-lastLeftPos+protectedRightEnc-lastRightPos)/2.0); //I would integrate instead, but for whatever reason
		        																		//position is more accurate than velocity.
		        posX -= (Math.sin(Math.toRadians(spigHeading))
		        		*(protectedLeftEnc-lastLeftPos+protectedRightEnc-lastRightPos)/2.0);
		        posYNavX += (Math.cos(Math.toRadians(navxFullHeading))
		        		*(protectedLeftEnc-lastLeftPos+protectedRightEnc-lastRightPos)/2.0);
		        posXNavX -= (Math.sin(Math.toRadians(navxFullHeading))
		        		*(protectedLeftEnc-lastLeftPos+protectedRightEnc-lastRightPos)/2.0);
		        lastLeftPos=protectedLeftEnc;
		        lastRightPos=protectedRightEnc;
		        SmartDashboard.putNumber("PositionX", posX);
		        SmartDashboard.putNumber("PositionY", posY);
		        SmartDashboard.putNumber("PositionXNavX", posXNavX);
		        SmartDashboard.putNumber("PositionYNavX", posYNavX);
				
				jerkX=(accelerationX-lastAccelX)/(Timer.getFPGATimestamp()-time); //For whatever reason, NavX just leaves out the whole "dt" part in their collision detector.
				jerkY=(accelerationY-lastAccelY)/(Timer.getFPGATimestamp()-time);
				lastAccelX=accelerationX;
				lastAccelY=accelerationY;
				time = Timer.getFPGATimestamp();
				iteration++;
				//Timer.delay(.04);
			}
		}

		@Override
		public void end() {
		}
		
	};
	
	public static void reset(){
		//fusedAngle.reset();
		resetGyro();
		//RobotMap.navxGyro.reset();
		RobotMap.left1.setEncPosition(0);
		RobotMap.right1.setEncPosition(0);
	}
	//All the getters are automatically generated, thank God.
	public static double getPosX() {
		return posX;
	}

	public static double getPosY() {
		return posY;
	}

	public static double getPosXNavX() {
		return posXNavX;
	}

	public static double getPosYNavX() {
		return posYNavX;
	}
	/**
	 * 
	 * @return Left distance in inches
	 */
	public static double getLeftPos() {
		return currentLeftPos;
	}

	/**
	 * 
	 * @return Right distance in inches
	 */
	public static double getRightPos() {
		return currentRightPos;
	}
	
//	public static double getLastLeftPos() {
//		
//	}
//	
//	public static double getLastRightPos() {
//		
//	}
	
	public static double getAccelerationX() {
		return accelerationX;
	}

	public static double getAccelerationY() {
		return accelerationY;
	}

	public static double getJerkX() {
		return jerkX;
	}

	public static double getJerkY() {
		return jerkY;
	}

	public static double getLeftSpeed() {
		return leftSpeed;
	}

	public static double getRightSpeed() {
		return rightSpeed;
	}

	public static double getNavxFullHeading() {
		return navxFullHeading;
	}

	public static double getSpigHeading() {
		return spigHeading;
	}

	public static double getShooterSpeed() {
		return shooterSpeed;
	}
	public static double getIter(){
		return iteration;
	}
	public static void resetGyro(){
		//System.out.println("Gyro has been reset");
		spigHeadingOffset=protectedSpigHeading;
	}
	public static double getProtectedHeading(){
		//"protected" just means you cannot set its value
		return protectedSpigHeading;
	}
	public static void resetPosition(){
		leftEncOffset=protectedLeftEnc;
		rightEncOffset=protectedRightEnc;
	}
	public static double getFullLeftPos(){
		return protectedLeftEnc;
	}
	public static double getFullRightPos(){
		return protectedRightEnc;
	}
}
