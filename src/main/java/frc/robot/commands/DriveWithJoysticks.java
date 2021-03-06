/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.Robot;
import frc.robot.RobotContainer;
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
    if(RobotContainer.getXboxDriveRB() == true){ 
      DriveTrain.getDriveTrain().setSpeedFalcon(Robot.driveTrain.leftVisionAdjusted(),-Robot.driveTrain.rightVisionAdjusted());
    }
    else{
      RobotContainer.getRobotContainer();
      RobotContainer.getRobotContainer();
      DriveTrain.getDriveTrain().setSpeedFalcon(-RobotContainer.getLeftXboxJoystickValue(),
          RobotContainer.getRightXboxJoystickValue());
    }
    
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
}
