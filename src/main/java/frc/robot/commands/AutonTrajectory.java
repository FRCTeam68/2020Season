/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.File;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.PathFollower;
import jaci.pathfinder.Pathfinder;


public class AutonTrajectory extends CommandBase {
  /**
   * Creates a new AutonTrajectory.
   */

  PathFollower path;
  boolean firstRun = true;
  double l;
  double r;
  boolean isFinished = false;
  boolean backwards;

  public AutonTrajectory(File leftPath, File rightPath) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
    path = new PathFollower(leftPath, rightPath);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    l = path.encLeft.calculate(Robot.driveTrain.getLeftEnc());
    r = path.encRight.calculate(Robot.driveTrain.getRightEnc());

    /* Get navX values to compensate for the errors */
    double theta = Robot.driveTrain.getHeading();
    if (backwards) {
      theta = theta * -1;
    }
    
    /* Find the heading and angle */
    double desiredHeading = Pathfinder.r2d(path.encLeft.getHeading());
    double angleDifference = Pathfinder.boundHalfDegrees(desiredHeading-theta);
    
    /* kP gain on the Gyroscope. Units are %output / degree. 
     * The 1/80 reduces it from degrees to 1/4.5th of a circle, and 0.8 is the actual gain. 
     * What it's saying is "at 80 degrees of error, make this term worth 5" */
    double turn = 5 * (-1.0/80.0) * angleDifference;

    /* Add power to motors to drive */
    Robot.driveTrain.setSpeedFalcon(l+turn, r-turn);

    /* Check if the path is finished */
      if(path.encRight.isFinished() && path.encRight.isFinished()){
        Robot.driveTrain.positionalMode(0, 0);
        System.out.println("Both trajectories finished");
        isFinished = true;
      }

    firstRun = false;
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
