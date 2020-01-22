/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
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

private final DifferentialDriveOdometry m_odometry;

// The right-side drive encoder

// The gyro sensor
private final AHRS m_gyro = new AHRS();

  public static DriveTrain driveTrain;


	public static DriveTrain getDriveTrain() {
		if (driveTrain == null) {
			driveTrain = new DriveTrain();
		}
		return driveTrain;
	}
  public DriveTrain() {
    fr = new WPI_TalonSRX(Constants.TALONSRX_FR);
    br = new WPI_TalonSRX(Constants.TALONSRX_BR);
    bl = new WPI_TalonSRX(Constants.TALONSRX_BL);
    fl = new WPI_TalonSRX(Constants.TALONSRX_FL);

    fl.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
    fl.selectProfileSlot(Constants.DRIVETRAIN_RIGHT_PID_SLOT, 0);
		fl.config_kF(Constants.DRIVETRAIN_RIGHT_PID_SLOT, Constants.DRIVETRAIN_RIGHT_PID_F, 0);
		fl.config_kP(Constants.DRIVETRAIN_RIGHT_PID_SLOT, Constants.DRIVETRAIN_RIGHT_PID_P, 0);
		fl.config_kI(Constants.DRIVETRAIN_RIGHT_PID_SLOT, Constants.DRIVETRAIN_RIGHT_PID_I, 0);
    fl.config_kD(Constants.DRIVETRAIN_RIGHT_PID_SLOT, Constants.DRIVETRAIN_RIGHT_PID_D, 0);
    fl.setSensorPhase(false);


    br.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
    br.selectProfileSlot(Constants.DRIVETRAIN_LEFT_PID_SLOT, 0);
		br.config_kF(Constants.DRIVETRAIN_LEFT_PID_SLOT, Constants.DRIVETRAIN_LEFT_PID_F, 0);
		br.config_kP(Constants.DRIVETRAIN_LEFT_PID_SLOT, Constants.DRIVETRAIN_LEFT_PID_P, 0);
		br.config_kI(Constants.DRIVETRAIN_LEFT_PID_SLOT, Constants.DRIVETRAIN_LEFT_PID_I, 0);
    br.config_kD(Constants.DRIVETRAIN_LEFT_PID_SLOT, Constants.DRIVETRAIN_LEFT_PID_D, 0);
    bl.setSensorPhase(false);

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

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    
    m_gyro.reset();
    m_gyro.setAngleAdjustment(0);


  }

  @Override
  public void periodic() {
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), .5,
    .5);
    CommandScheduler.getInstance().setDefaultCommand(Robot.driveTrain, new DriveWithJoysticks());
  }
  public void setSpeedFalcon(double left, double right){
    fl.set(ControlMode.PercentOutput,left);
    br.set(ControlMode.PercentOutput,right);
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */


  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */


  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void positionalMode(double leftVolts, double rightVolts) {
    fl.set(ControlMode.Position,leftVolts);
    br.set(ControlMode.Position, rightVolts);
    
   }


  /**
   * Zeroes the heading of the robot.
   */
  public void ResetEncoders(){
    fl.setSelectedSensorPosition(100,0,0);
    br.setSelectedSensorPosition(100,0,0);

  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return m_gyro.getAngle();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (Constants.kGyroReversed ? -1.0 : 1.0);
  }
  public int getLeftEnc(){
    return fl.getSelectedSensorPosition(0);
  }
  public int getRightEnc(){
    return br.getSelectedSensorPosition(0);

  }
}