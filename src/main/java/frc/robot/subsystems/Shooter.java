/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

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
  private NetworkTableEntry ent;

  public Shooter() {
    shooterWheel1 = new CANSparkMax(Constants.SHOOTER_WHEELSPINNER_1, MotorType.kBrushless);
    shooterWheel2 = new CANSparkMax(Constants.SHOOTER_WHEELSPINNER_2, MotorType.kBrushless);
    shooterWheel1.restoreFactoryDefaults();
    shooterWheel2.restoreFactoryDefaults();
    
    feeder = new WPI_VictorSPX(Constants.SHOOTER_FEEDER);
    shooterAngle = new WPI_TalonSRX(Constants.SHOOTER_ANGLE);

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

    /*    Robot.smartPID.SmartPIDEntry("CUNT", 0, 1, 0);

     * pidController1.setP(SmartDashboard.getNumber("kProportion", 0));
     * pidController1.setI(SmartDashboard.getNumber("kIntegral", 0));
     * pidController1.setD(SmartDashboard.getNumber("kDerivitive", 0));
     * 
     * pidController1.setFF(SmartDashboard.getNumber("kFeedForward", 0));
     * pidController2.setP(SmartDashboard.getNumber("kProportion", 0));
     * pidController2.setI(SmartDashboard.getNumber("kIntegral", 0));
     * pidController2.setD(SmartDashboard.getNumber("kDerivitive", 0));
     * pidController2.setFF(SmartDashboard.getNumber("kFeedForward", 0));
     */
  }


  public void setShooterSpeed(double shooterSpeed) {
    shooterWheel1.set(shooterSpeed);
    shooterWheel2.set(-shooterSpeed);
  }

  public void setShooterPID(double p, double i, double d, double f, double iZ) {
    pidController1.setIZone(iZ);
    pidController1.setP(p);
    pidController1.setI(i);
    pidController1.setD(d);
    pidController1.setFF(f); // CALCULATED F = .0002 for 100% and F = .000266667 for 75%
    pidController2.setIZone(iZ);
    pidController2.setP(p);
    pidController2.setI(i);
    pidController2.setD(d);
    pidController2.setFF(f);
    pidController2.setOutputRange(-1, 1);
    pidController1.setOutputRange(-1, 1);

    /*
     * pidController1.setP(SmartDashboard.getNumber("kProportion", 0));
     * pidController1.setI(SmartDashboard.getNumber("kIntegral", 0));
     * pidController1.setD(SmartDashboard.getNumber("kDerivitive", 0));
     * pidController1.setFF(SmartDashboard.getNumber("kFeedForward", 0));
     * 
     * 
     * pidController1.setFF(SmartDashboard.getNumber("kFeedForward", 0));
     * pidController2.setP(SmartDashboard.getNumber("kProportion", 0));
     * pidController2.setI(SmartDashboard.getNumber("kIntegral", 0));
     * pidController2.setD(SmartDashboard.getNumber("kDerivitive", 0));
     * pidController2.setFF(SmartDashboard.getNumber("kFeedForward", 0));
     */
  }

  public void setShooterVelocity(double shooterVelocity) {

    pidController1.setReference(-shooterVelocity, ControlType.kVelocity);
    pidController2.setReference(shooterVelocity, ControlType.kVelocity);
  }
  public double p(){
    return  Robot.smartPID.createCustomEntry(ent, "Test", 0, 3);
  }
}
