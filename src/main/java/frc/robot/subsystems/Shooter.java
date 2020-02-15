/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
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

  public Shooter() {

    shooterWheel1 = new CANSparkMax(Constants.SHOOTER_WHEELSPINNER_1, MotorType.kBrushless);
    shooterWheel2 = new CANSparkMax(Constants.SHOOTER_WHEELSPINNER_2, MotorType.kBrushless);
    feeder = new WPI_VictorSPX(Constants.SHOOTER_FEEDER);
    shooterAngle = new WPI_TalonSRX(Constants.SHOOTER_ANGLE);

    shooterAngle.setSensorPhase(false);
    feeder.setSensorPhase(false);
    shooterWheel1.setInverted(false);
    shooterWheel2.setInverted(false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooterSpeed(double shooterSpeed){
    shooterWheel1.set(shooterSpeed);
    shooterWheel2.set(-shooterSpeed);
  }
}
