/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.SetShooterSpeed;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  private CANSparkMax shooterWheel1;
  private CANSparkMax shooterWheel2;
  private WPI_TalonSRX shooterAngle;
  private WPI_VictorSPX feeder;
  private CANEncoder shooterWheel1Enc;
  private CANEncoder shooterWheel2Enc;
  private CANPIDController pidController1;
  private CANPIDController pidController2;
  private SimpleMotorFeedforward smFF_pid1;
  private SimpleMotorFeedforward smFF_pid2;

  public Shooter() {
    shooterWheel1 = new CANSparkMax(Constants.SHOOTER_WHEELSPINNER_1, MotorType.kBrushless);
    shooterWheel2 = new CANSparkMax(Constants.SHOOTER_WHEELSPINNER_2, MotorType.kBrushless);
    shooterWheel1.restoreFactoryDefaults();
    shooterWheel2.restoreFactoryDefaults();
    
    feeder = new WPI_VictorSPX(Constants.SHOOTER_FEEDER);
    shooterAngle = new WPI_TalonSRX(Constants.SHOOTER_ANGLE);
    shooterAngle.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
    shooterAngle.selectProfileSlot(Constants.SHOOTER_PID_SLOT, 0);

    shooterAngle.setSensorPhase(false);
    feeder.setSensorPhase(false);

    shooterWheel1Enc = shooterWheel1.getEncoder();
    shooterWheel2Enc = shooterWheel2.getEncoder();
    pidController1 = shooterWheel1.getPIDController();
    pidController2 = shooterWheel2.getPIDController();

    pidController1.setP(Constants.SHOOTER_PID_P);
    pidController1.setI(Constants.SHOOTER_PID_I);
    pidController1.setD(Constants.SHOOTER_PID_D);
    pidController1.setFF(Constants.SHOOTER_PID_F);
    pidController2.setP(Constants.SHOOTER_PID_P);
    pidController2.setI(Constants.SHOOTER_PID_I);
    pidController2.setD(Constants.SHOOTER_PID_D);
    pidController2.setFF(Constants.SHOOTER_PID_F);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    CommandScheduler.getInstance().setDefaultCommand(Robot.shooter, new SetShooterSpeed());
    SmartDashboard.putNumber("Velocity", shooterWheel1Enc.getVelocity());
    SmartDashboard.putNumber("Velocity 2", shooterWheel2Enc.getVelocity());
    SmartDashboard.putNumber("Angle Enc", shooterAngle.getSelectedSensorPosition());

  }


  public void setShooterSpeed(double shooterSpeed) {
    shooterWheel1.set(-shooterSpeed);
    shooterWheel2.set(shooterSpeed);
    feeder.set(shooterSpeed);
  }
  public void setShooterAngle(double angle){
    shooterAngle.set(ControlMode.Position,angle);
  }
  public void setAnglePIDF(double p,double i,double d,double f){
    shooterAngle.config_kP(Constants.SHOOTER_PID_SLOT, p);
    shooterAngle.config_kI(Constants.SHOOTER_PID_SLOT, i);
    shooterAngle.config_kD(Constants.SHOOTER_PID_SLOT, d);
    shooterAngle.config_kF(Constants.SHOOTER_PID_SLOT, f);
  }


  public void setShooterPID(double p, double i, double d, double p_2, double i_2, double d_2) {
    pidController1.setP(p);
    pidController1.setI(i);
    pidController1.setD(d);
    pidController2.setP(p_2);
    pidController2.setI(i_2);
    pidController2.setD(d_2);
    pidController2.setOutputRange(-1, 1);
    pidController1.setOutputRange(-1, 1);
  }

  public void setShooterVelocity(double shooterVelocity, double shooterVelocity_2) {
    // the value for pidController1 is - because it is backward
    pidController1.setReference(-shooterVelocity, ControlType.kVelocity, 0, smFF_pid1.calculate(shooterVelocity / 60, (shooterVelocity - shooterWheel1Enc.getVelocity()) / 60));
    pidController2.setReference(shooterVelocity_2, ControlType.kVelocity, 0, smFF_pid2.calculate(shooterVelocity_2 / 60, (shooterVelocity_2 - shooterWheel2Enc.getVelocity()) / 60));
    
  }
  public void zeroEncoders(){
    shooterAngle.setSelectedSensorPosition(0);
  }
}
