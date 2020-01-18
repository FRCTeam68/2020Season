/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
///import frc.robot.commands.Auton;
//import frc.robot.subsystems.PathFollower;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

//import frc.robot.Robot;
/*import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
*/
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
// import frc.robot.commands.DriveWithJoysticks;
// import frc.robot.subsystems.DriveTrain;
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  XboxController xboxDrive;

  private static RobotContainer robotContainer;

  // private final PathFollower m_exampleSubsystem = new PathFollower();

  private static DriveTrain m_robotDrive = new DriveTrain();
  // public final Auton m_autoCommand;

  public static RobotContainer getRobotContainer() {
    if (robotContainer == null) {
      robotContainer = new RobotContainer();
    }
    return robotContainer;
  }

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    xboxDrive = new XboxController(Constants.XBOX_DRIVE);
    // m_autoCommand = new Auton();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    xboxDrive = new XboxController(Constants.XBOX_DRIVE);
  }

  public double getLeftXboxJoystickValue() {
    double leftAxis;
    leftAxis = xboxDrive.getY(Hand.kLeft);
    // Allow for up to 10% of joystick noises\
    leftAxis = (Math.abs(leftAxis) < 0.1) ? 0 : leftAxis;
    return leftAxis;
  }

  // Drivetrain Tank Drive Right
  public double getRightXboxJoystickValue() {
    double rightAxis;
    rightAxis = xboxDrive.getY(Hand.kRight);
    // Allow for up to 10% of joystick noise
    rightAxis = (Math.abs(rightAxis) < 0.1) ? 0 : rightAxis;
    return rightAxis;

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * @throws IOException
   */
  SequentialCommandGroup getAutonomousCommand() throws IOException {
     //An ExampleCommand will run in autonomous
    //return m_autoCommand;
    Trajectory exampleTrajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/paths/EZPZ.wpilib.json"));
    RamseteCommand ramseteCommand = new RamseteCommand(

      exampleTrajectory,
      m_robotDrive::getPose,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      new SimpleMotorFeedforward(Constants.ksVolts,
                                 Constants.kvVoltSecondsPerMeter,
                                 Constants.kaVoltSecondsSquaredPerMeter),
      Constants.kDriveKinematics,
      m_robotDrive::getWheelSpeeds,
      new PIDController(Constants.DRIVETRAIN_LEFT_PID_P, Constants.DRIVETRAIN_LEFT_PID_I, Constants.DRIVETRAIN_LEFT_PID_D),
      new PIDController(Constants.DRIVETRAIN_RIGHT_PID_P, Constants.DRIVETRAIN_RIGHT_PID_I, Constants.DRIVETRAIN_RIGHT_PID_D),
      // RamseteCommand passes volts to the callback
      m_robotDrive::tankDriveVolts,
      m_robotDrive
  );

  // Run path following command, then stop at the end.
  return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  }
}
