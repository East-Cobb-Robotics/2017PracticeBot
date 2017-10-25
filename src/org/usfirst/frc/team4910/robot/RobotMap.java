package org.usfirst.frc.team4910.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Calendar;
import java.io.FileWriter;
import java.io.IOException;

import org.usfirst.frc.team4910.subsystems.DriveTrain;
import org.usfirst.frc.team4910.util.*;
import com.ctre.*;
import com.kauailabs.navx.frc.AHRS;
import com.opencsv.CSVWriter;
public class RobotMap {
	
	public static CANTalon left1,left2,right1,right2;
	public static CANTalon elevControl, shootControl, climbControl, agitateControl, shootGuide;
	public static DoubleSolenoid gearShifter, gates;
	public static Compressor c;
	public static SynchronousPID drivePositionLeftPID,drivePositionRightPID,driveVelocityLeftPID,driveVelocityRightPID;
	public static SynchronousPID driveGyroPID;
	public static SynchronousPID shootPID, shootGuidePID; //both should be a simple PD+F loop
	public static GyroHelper spig;
	//public static AHRS navxGyro;
	public static CSVWriter writer;
	public static BuiltInAccelerometer RIOAccel;
	public static AnalogInput ultra;
	
	public static final boolean isCompBot=true;
	
	public static final double DriveWheelDiameter=6;
	public static final double EncCountsPerRev= (isCompBot&&false) ? 1440 : 4096; //4096 for practice bot, 1440 for compbot
	public static final double rightMinIPS=-210.0;
	public static final double rightMaxIPS=210.0;
	public static final double leftMinIPS=-210.0;
	public static final double leftMaxIPS=210.0;
	public static final double leftMinIPSPS=-1009.4170013419557;
	public static final double leftMaxIPSPS=1038.9106737835907;
	public static final double rightMinIPSPS=-1069.4313941540784;
	public static final double rightMaxIPSPS=982.7519404608955;
	
	//Position, velocity, and gyro PIDF values are all for high gear only
	public static double PositionKp=0.036; //0.0125
	public static double PositionKi=0.0;//0.0002; 
	public static double PositionKd=0.0; //0.04
	public static double PositionKf=0.0;
	public static final double VelocityKp=.0006; //Unused valuess
	public static final double VelocityKi=0.00026;
	public static final double VelocityKd=0.0;
	public static final double VelocityKf1=0.0053;
	public static final double VelocityKf2=0.0053;
	public static final double GyroKp= isCompBot ? 0.0325: 0.0308; //0.0325 (high gear)
	public static final double GyroKi=0.0; //0.0013
	public static final double GyroKd= isCompBot ? 0.026: 0.0257; //1.0
	
	//Our solution for not needing this was just starting in low gear
	//Apparently, the shifter gears actually have a use for me.
//	public static final double GyroSmallKp=0.0;
//	public static final double GyroSmallKi=0.0;
//	public static final double GyroSmallKd=0.0;
	
	public static final double shooterKp= /*isCompBot ?*/ 0.04186;// : .00237; //.00237
	public static final double shooterKi= /*isCompBot ?*/ 1.81395E-4;// : 0.869E-5; // 0.869E-5
	public static final double shooterKd=0.0;
	public static final double shooterKf= /*isCompBot ?*/ 0.011627;// : 6.3E-4;//6.3E-4
	public static final double shooterGuideKp=0.0;
	public static final double shooterGuideKd=0.0;
	public static final double shooterGuideKf=0.0;
	public static final double shooterGuideOptimalSpeed=0.0;
	public static final double shooterGuideOptimalTime=0.5;
	public static final double shooterSpinupTime=2.2;
	public static final double shooterTimeToShoot=60.0;
	
	public static final String PegIP = isCompBot ? "10.49.10.17" : "10.49.10.18"; //both have manual static IPs
	public static final String ShooterIP = isCompBot ? "10.49.10.44" : "10.49.10.62"; //both have quasi-dynamic dhcp assigned IPs
	
	public static final boolean testerCodeEnabled=false; //This enables functions like writing data down to a CSV file, or
														//putting (more) data on SmartDashboard, or anything involving tuning
														//pid loops. Running this code would be rather intense for a competition
														//and could therefore cause misclicks or other code-based problems to occur.
	public static final double VelocityRampRate=0.0;
	
	public static void init(){
		left1 = new CANTalon(1);
		left2 = new CANTalon(2);
		right1 = new CANTalon(3);
		right2 = new CANTalon(4);
		elevControl = new CANTalon(5);
		climbControl = new CANTalon(6);
		shootGuide = new CANTalon(7);
		shootControl = new CANTalon(8);
		//connect pc to 10.49.10.5
		ultra = new AnalogInput(0);
		gearShifter = isCompBot ? new DoubleSolenoid(7,6) : new DoubleSolenoid(4,5) ; //4,5 on practice bot
		gates = isCompBot ? new DoubleSolenoid(4,5) : new DoubleSolenoid(6,7); //6,7 on comp bot
		spig = new GyroHelper(SPI.Port.kOnboardCS0);
		//navxGyro = new AHRS(SPI.Port.kMXP);
		RIOAccel = new BuiltInAccelerometer();
		left1.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
		right1.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
        left1.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        left1.set(0);
        left2.changeControlMode(CANTalon.TalonControlMode.Follower);
        left2.set(left1.getDeviceID());
        right1.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        right1.set(0);
        right2.changeControlMode(CANTalon.TalonControlMode.Follower);
        right2.set(right1.getDeviceID());
        shootControl.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        shootControl.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
        shootControl.setVoltageRampRate(30);
        shootControl.setEncPosition(0);
        c = new Compressor(0); //TODO: Schedule the compressor for certain times during the match    
        c.setClosedLoopControl(true);
        c.start();
        drivePositionLeftPID = new SynchronousPID(PositionKp,PositionKi,PositionKd,PositionKf);
        drivePositionRightPID = new SynchronousPID(PositionKp,PositionKi,PositionKd,PositionKf);
        driveVelocityLeftPID = new SynchronousPID(VelocityKp,VelocityKi,VelocityKd,VelocityKf1);
        driveVelocityRightPID = new SynchronousPID(VelocityKp,VelocityKi,VelocityKd,VelocityKf2);
        driveVelocityLeftPID.setOutputRange(leftMinIPS, leftMaxIPS);
        driveVelocityRightPID.setOutputRange(rightMinIPS, rightMaxIPS);
        driveGyroPID = new SynchronousPID(GyroKp, GyroKi, GyroKd);
        shootPID = new SynchronousPID(shooterKp, shooterKi, shooterKd, shooterKf);
        shootGuidePID = new SynchronousPID(shooterGuideKp, 0.0, shooterGuideKd, shooterGuideKf);
        left1.configEncoderCodesPerRev((int)(EncCountsPerRev));
        right1.configEncoderCodesPerRev((int)(EncCountsPerRev));
        shootControl.configEncoderCodesPerRev(75);
        
        left1.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        right1.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
         
    	left1.setVoltageRampRate(0);
    	right1.setVoltageRampRate(0);
        shootControl.setVoltageRampRate(24);
    	RobotMap.driveGyroPID.setIZoneRange(.1,12.0);
    	RobotMap.drivePositionLeftPID.setIZoneRange(0.0,6.0);
    	RobotMap.drivePositionRightPID.setIZoneRange(0.0,6.0);
    	RobotMap.driveVelocityLeftPID.setIZoneRange(-1.0,70.0);
    	RobotMap.driveVelocityRightPID.setIZoneRange(-1.0,70.0);
    	
        left1.reverseSensor(true);
        left1.reverseOutput(true);
        left2.reverseOutput(false);
        
        right1.reverseSensor(false);
        right1.reverseOutput(true);
        right2.reverseOutput(false);
        Robot.compressorEnabled=false; //potentially unused
        
		
        
        
	}
	
}
