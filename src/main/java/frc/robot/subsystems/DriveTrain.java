/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.DriveWithJoysticks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.SPI;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */ 
  private WPI_TalonSRX fr; //front right
  private WPI_TalonSRX br; //back right
  private WPI_TalonSRX bl; //back left
  private WPI_TalonSRX fl; //front left\



// The robot's drive

// The left-side drive encoder


// The right-side drive encoder

// The gyro sensor
private AHRS m_gyro = new AHRS();

  public static DriveTrain driveTrain;


	public static DriveTrain getDriveTrain() {
		if (driveTrain == null) {
			driveTrain = new DriveTrain();
		}
		return driveTrain;
	}
  public DriveTrain() {
    // NAVX CONFIG
    m_gyro = new AHRS(SPI.Port.kMXP);
    m_gyro.reset();
    m_gyro.zeroYaw();
    // DriveTrain Motors Config
    fr = new WPI_TalonSRX(Constants.TALONSRX_FR);
    br = new WPI_TalonSRX(Constants.TALONSRX_BR);
    bl = new WPI_TalonSRX(Constants.TALONSRX_BL);
    fl = new WPI_TalonSRX(Constants.TALONSRX_FL);

    br.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
    br.selectProfileSlot(Constants.DRIVETRAIN_RIGHT_PID_SLOT, 0);
		br.config_kF(Constants.DRIVETRAIN_RIGHT_PID_SLOT, Constants.DRIVETRAIN_RIGHT_PID_F, 0);
		br.config_kP(Constants.DRIVETRAIN_RIGHT_PID_SLOT, Constants.DRIVETRAIN_RIGHT_PID_P, 0);
		br.config_kI(Constants.DRIVETRAIN_RIGHT_PID_SLOT, Constants.DRIVETRAIN_RIGHT_PID_I, 0);
    br.config_kD(Constants.DRIVETRAIN_RIGHT_PID_SLOT, Constants.DRIVETRAIN_RIGHT_PID_D, 0);
    br.setSensorPhase(false);


    fl.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
    fl.selectProfileSlot(Constants.DRIVETRAIN_LEFT_PID_SLOT, 0);
		fl.config_kF(Constants.DRIVETRAIN_LEFT_PID_SLOT, Constants.DRIVETRAIN_LEFT_PID_F, 0);
		fl.config_kP(Constants.DRIVETRAIN_LEFT_PID_SLOT, Constants.DRIVETRAIN_LEFT_PID_P, 0);
		fl.config_kI(Constants.DRIVETRAIN_LEFT_PID_SLOT, Constants.DRIVETRAIN_LEFT_PID_I, 0);
    fl.config_kD(Constants.DRIVETRAIN_LEFT_PID_SLOT, Constants.DRIVETRAIN_LEFT_PID_D, 0);
    fl.setSensorPhase(false);

    bl.set(ControlMode.Follower, fl.getDeviceID());
    fr.set(ControlMode.Follower, br.getDeviceID());


    fr.setNeutralMode(NeutralMode.Brake);
    br.setNeutralMode(NeutralMode.Brake);
    fl.setNeutralMode(NeutralMode.Brake);
    bl.setNeutralMode(NeutralMode.Brake);


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
  public void setSpeedFalcon(double left, double right){
    
    fl.set(ControlMode.PercentOutput,left);
    br.set(ControlMode.PercentOutput,right);
    
  }
  public void setSpeedAuto(double left, double right){
    
    fl.set(left);
    br.set(right);
    
  }


  /**
   * Zeroes the heading of the robot.
   */
  public void ResetEncoders(){
    fl.setSelectedSensorPosition(10,0,0);
    br.setSelectedSensorPosition(10,0,0);
    m_gyro.zeroYaw();
    /*m_gyro.reset(); for auton experimental we may have to reset thee navx to re run a new path */
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return m_gyro.getAngle();
  }

  public double getYAW() {
    return m_gyro.getYaw();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public int getLeftEnc(){
    return fl.getSelectedSensorPosition(0);
  }
  public int getRightEnc(){
    return br.getSelectedSensorPosition(0);

  }
}