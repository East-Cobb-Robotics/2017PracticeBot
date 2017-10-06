package org.usfirst.frc.team4910.subsystems;

import org.zeromq.ZMQ;
import org.zeromq.ZMQ.Context;
import org.zeromq.ZMQ.Socket;

import com.google.protobuf.*;

import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team4910.util.Peg;

/**
 * This is the class that comuicates with the jetson
 */
public class Jetson
{
	// This stores the context of the system
	private Context context = ZMQ.context(1);
	
	// The socket for commuicating with the jetson
	private Socket sub = context.socket(ZMQ.SUB);

	/**
	 * This command runs on robot init
	 * It sets up the context and socket and connects to the jetson PUB 
	 */
	public void startZMQ() {
		sub.setConflate(true); // Only get the most recent messages
		sub.connect("tcp://10.49.10.200:49100");
		sub.subscribe("".getBytes());
	}

	public Peg.PegLoc receave() {
		Peg.PegLoc loc;
		byte[] message;

		System.out.println("Starting recv");
		message = sub.recv();
		try {
			loc = Peg.PegLoc.parseFrom(message);
		} catch (Exception e) {
			System.out.println("Protocall Buffer Error");
			e.printStackTrace();
			return null;
		}
		
		return loc;
	}
	
	/**
	 * Cleans up ZMQ to prevent memory attacks
	 * NOTE: This fuction MUST BE CALLED in the robot deinitalsa procces
	 */
	public void destroyZMQ(){
		sub.close();
		context.term();
	}
}
