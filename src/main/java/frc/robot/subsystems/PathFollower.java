/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

//import jaci.pathfinder.Trajectory;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

public class PathFollower extends SubsystemBase {

  public static PathFollower pathFollower;

  public static DriveTrainAuton m_robotDrive;

	public static PathFollower getPathFollower() {
		if (pathFollower == null) {
			pathFollower = new PathFollower();
    }
    return pathFollower;
  }
public PathFollower() {

  runAuton("Simple");

  // Sets the distance per pulse for the encoders
  }
  public void runAuton(String pathName){
    try{
      final Trajectory exampleTrajectory = TrajectoryUtil
          .fromPathweaverJson(Paths.get("/home/lvuser/deploy/paths/"+ pathName + ".wpilib.json"));
  
      final RamseteCommand ramseteCommand = new RamseteCommand(
      exampleTrajectory,
      m_robotDrive::getPose,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      new SimpleMotorFeedforward(Constants.ksVolts,
                                 Constants.kvVoltSecondsPerMeter,
                                 Constants.kaVoltSecondsSquaredPerMeter),
      Constants.kDriveKinematics,
      m_robotDrive::getWheelSpeeds,
      new PIDController(Constants.kPDriveVel, 0, 0),
      new PIDController(Constants.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      m_robotDrive::tankDriveVolts,
      m_robotDrive
  );
    } catch(final IOException e) {
      System.out.print("hahahha very stinky did not work");
    }
  }
}

