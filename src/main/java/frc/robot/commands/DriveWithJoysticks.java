/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Robot;
import jaci.pathfinder.followers.EncoderFollower;
//import frc.robot.RobotContainer;
//import frc.robot.Robot;
public class DriveWithJoysticks extends CommandBase {
  /**
   * Creates a new DriveWithJoysticks.
   */
  public DriveWithJoysticks() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //System.out.println(Robot.m_robotContainer);
    //System.out.println("LEFT : "+Robot.driveTrain.getLeftEnc());
    //System.out.println("RIGHT : "+Robot.driveTrain.getRightEnc());

    DriveTrain.getDriveTrain().setSpeedFalcon(Robot.m_robotContainer.getLeftXboxJoystickValue(), Robot.m_robotContainer.getRightXboxJoystickValue());
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
