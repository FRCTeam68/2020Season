/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.File;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class PathFollower extends SubsystemBase {
  /**
   * Creates a new PathFollower.
   */
  	double timeStep = 0.02; //amount of points created and read per second

    double maxVel = 110; // max velocity in inches per second
	double maxAccel = 120; // max acceleration in inches per second
	double maxJerk = 1200; 
	double wheelBaseWidth = 24; // distance between center of wheels(in inches)
	int ticksPerRevLeft = 25780;  
	int ticksPerRevRight = 25520;  
	double wheelDiameter = 7.50; //wheel diameter in inches

	/* PID values and kV, kA, 
	 * Recommend reading Drivetrain Characterization from Blair Witch to get velocityRatio, maxVel, etc*/
	double p = 6.0; // calculate this 
	double i = 0.0; // dont touch
	double d = 0.6; // calculate this
	double velocityRatio = 1/maxVel;
	double accelGain = 0.0;	
	// The first argument is the proportional gain. Usually this will be quite high
		// The second argument is the integral gain. This is unused for motion profiling
		// The third argument is the derivative gain. Tweak this if you are unhappy with the tracking of the trajectory
		// The fourth argument is the velocity ratio. This is 1 over the maximum velocity you provided in the 
		//	      trajectory configuration (it translates m/s to a -1 to 1 scale that your motors can read)
		// The fifth argument is your acceleration gain. Tweak this if you want to get to a higher or lower speed quicker
		// ethic is key!
		double l;
		double r;

		/* Find and create trajectory, create followers with input from encoders */
		public Trajectory forwardLeftTrajectory;
		public Trajectory forwardRightTrajectory;
		Trajectory forwardTrajectory;
		public EncoderFollower encLeft;
		public EncoderFollower encRight;
		

  public PathFollower(File leftPath, File rightPath) {
		try{	
			/* Create trajectory from CSV files */
			System.out.println("Generating trajectory...");
			Trajectory trajectoryLW = Pathfinder.readFromCSV(leftPath);
			Trajectory trajectoryRW = Pathfinder.readFromCSV(rightPath);


			System.out.println("Trajectory Generation completed");
			
			/* Create and configure the encoder followers using the previous settings */
			encLeft = new EncoderFollower(trajectoryLW);
			encRight = new EncoderFollower(trajectoryRW);
			/*egs (Encsoder vals, ticksPerRev, wheelDiameter) */
			encLeft.configureEncoder(Robot.driveTrain.getLeftEnc(), ticksPerRevLeft, wheelDiameter);
			encRight.configureEncoder(Robot.driveTrain.getRightEnc(), ticksPerRevRight, wheelDiameter);
			
			// Configure PIDVA
			encLeft.configurePIDVA(p, i, d, velocityRatio, accelGain);
			encRight.configurePIDVA(p, i, d, velocityRatio, accelGain);
			
			
		}catch(Exception e){
			e.printStackTrace();
			System.out.println("Error in Path Construction" + e.getMessage());
		}
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
