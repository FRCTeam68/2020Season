package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DriveWithJoysticks extends CommandBase {
    public DriveWithJoysticks() {
        addRequirements(Robot.driveTrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        DriveTrain.getDriveTrain().setSpeedFalcon(RobotContainer.getRobotContainer().getLeftXboxJoystickValue(), RobotContainer.getRobotContainer().getRightXboxJoystickValue());
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}