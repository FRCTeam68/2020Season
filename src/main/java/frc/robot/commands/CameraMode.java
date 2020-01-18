package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class CameraMode extends CommandBase {

    public CameraMode(){
        addRequirements(Robot.vision);
    }

    @Override
    public void initialize() {

    }

    @Override
  public void execute() {
  
    if(RobotContainer.getXboxDriveRB() == true){
        Vision.getVision().setCameraMode(3, 0);
     }
     else{
        Vision.getVision().setCameraMode(1, 1);
     }
    }

    @Override
    public void end(final boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
      return false;
    }
}