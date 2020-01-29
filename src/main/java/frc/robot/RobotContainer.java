/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Robot;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  XboxController xboxDrive = new XboxController(Constants.XBOX_DRIVE);

  private POVButton xboxDrivePOVUp;
  private POVButton xboxDrivePOVDown;
  private POVButton xboxDrivePOVLeft;
  private POVButton xboxDrivePOVRight;
  private static JoystickButton xboxDriveRB;
  private static JoystickButton xboxDriveLB;
  private static JoystickButton xboxDriveRTButton;
  private static JoystickButton xboxDriveLTButton;
  private static JoystickButton xboxDriveTriangle;
  private static JoystickButton xboxDriveCircle;
  private static JoystickButton xboxDriveSquare;
  private static JoystickButton xboxDriveX;
  private static JoystickButton xboxDriveStart;
  private static JoystickButton xboxDriveSL;
  private static JoystickButton xboxDriveSR;

  private static RobotContainer robotContainer;

  public static RobotContainer getRobotContainer() {
    if (robotContainer == null) {
      robotContainer = new RobotContainer();
    }
    return robotContainer;
  }

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    xboxDriveRB = new JoystickButton(xboxDrive, Constants.XBOX_DRIVE_RB);

  }

  public double getLeftXboxJoystickValue() {
    double leftAxis;
    leftAxis = xboxDrive.getY(Hand.kLeft);
    leftAxis = (Math.abs(leftAxis) < 0.1) ? 0 : leftAxis;
      return leftAxis;
  }

  public double getRightXboxJoystickValue() {
    double rightAxis;
    rightAxis = xboxDrive.getY(Hand.kRight);
    rightAxis = (Math.abs(rightAxis) < 0.1) ? 0 : rightAxis;
      return rightAxis;
  }

  public static boolean getXboxDriveRB() {
    boolean buttonPressed = false;
    if (xboxDriveLB.get()) {
      buttonPressed = true;
    }
    return buttonPressed;
  }

  public static boolean getXboxDriveLB() {
    boolean buttonPressed = false;
    if (xboxDriveLB.get()) {
      buttonPressed = true;
    }
    return buttonPressed;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return m_autoCommand;
  //}
}
