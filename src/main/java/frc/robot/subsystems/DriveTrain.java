/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveWithJoysticks;

import frc.robot.Constants;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */ 
  private TalonFX fr; //front right
  private TalonFX br; //back right
  private TalonFX bl; //back left
  private TalonFX fl; //front left

  int currentLimit = 40;

  public static DriveTrain driveTrain;

  public static DriveTrain getDriveTrain() {
    if (driveTrain == null) {
      driveTrain = new DriveTrain();
    }
    return driveTrain;
  }

  public DriveTrain() {
    fr = new TalonFX(Constants.FALCON_FR);
    br = new TalonFX(Constants.FALCON_BR);
    bl = new TalonFX(Constants.FALCON_BL);
    fl = new TalonFX(Constants.FALCON_FL);

    fr.setNeutralMode(NeutralMode.Coast);
    br.setNeutralMode(NeutralMode.Coast);
    fl.setNeutralMode(NeutralMode.Coast);
    bl.setNeutralMode(NeutralMode.Coast);


    fr.configPeakOutputForward(1);
    br.configPeakOutputForward(1);
    fl.configPeakOutputForward(1);
    bl.configPeakOutputForward(1);
    fr.configPeakOutputReverse(-1);
    br.configPeakOutputReverse(-1);
    bl.configPeakOutputReverse(-1);
    fl.configPeakOutputReverse(-1);
  }

  @Override
  public void periodic() {
    CommandScheduler.getInstance().setDefaultCommand(Robot.driveTrain, new DriveWithJoysticks());
  }

  public void setSpeedFalcon(final double left, final double right) {
    fr.set(ControlMode.PercentOutput,right);
    br.set(ControlMode.PercentOutput,right);
    fl.set(ControlMode.PercentOutput,left);
    bl.set(ControlMode.PercentOutput,left);
  }

  
  
  public double leftVisionAdjusted(){
    double leftSpeed;
    double joystickSpeed;
    double visionCorrect;
    joystickSpeed = RobotContainer.getLeftXboxJoystickValue();
    visionCorrect = Robot.vision.steeringAdjust();
    leftSpeed = joystickSpeed += visionCorrect;
    return leftSpeed;
  }
  public double rightVisionAdjusted(){
    double rightSpeed;
    double joystickSpeed;
    double visionCorrect;
    joystickSpeed = RobotContainer.getRightXboxJoystickValue();
    visionCorrect = Robot.vision.steeringAdjust();
    rightSpeed =  joystickSpeed -= visionCorrect;
    return rightSpeed;
  }
}
