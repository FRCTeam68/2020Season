/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.paths.SixBallp1;
import frc.paths.SixBallp2;

public class RunAuton extends CommandBase {
  /**
   * Creates a new RunAuton.
   */
  public RunAuton() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new SequentialCommandGroup(new ChangeIntakePos(),
    new ShootMedium(),
    );
    /*
    new SequentialCommandGroup(new PathFollower(new SixBallp1()));
    new ParallelCommandGroup(new ShootLow(), new SetAgitator());
   // new WaitCommand(3);
    new ParallelCommandGroup(new Zero());
    new ParallelCommandGroup(new ChangeIntakePos(), new SetIntakeWithDouble(1));
    new SequentialCommandGroup(new PathFollower(new SixBallp2()));
    new ParallelCommandGroup(new SetIntakeWithDouble(0));
    new ParallelCommandGroup(new ShootLow(), new SetAgitator());
  //  new WaitCommand(3);
    new ParallelCommandGroup(new Zero());
    */
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
