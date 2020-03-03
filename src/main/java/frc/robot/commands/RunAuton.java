/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.paths.FullAuton;
import frc.paths.FullAutonp2;

public class RunAuton extends CommandGroup {
  /**
   * Add your docs here.
   */
  public RunAuton() {
    new ParallelCommandGroup(new ChangeIntakePos(), new IntakeCommand()
    }
}
