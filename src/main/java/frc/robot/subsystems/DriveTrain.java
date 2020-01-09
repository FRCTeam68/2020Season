/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.DriveWithJoysticks;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */ 
  private CANSparkMax fr; //front right
  private CANSparkMax br; //back right
  private CANSparkMax bl; //back left
  private CANSparkMax fl; //front left

  int currentLimit = 40;

  public static DriveTrain driveTrain;

	public static DriveTrain getDriveTrain() {
		if (driveTrain == null) {
			driveTrain = new DriveTrain();
		}
		return driveTrain;
	}
  public DriveTrain() {
    fr = new CANSparkMax(Constants.NEO_FR, MotorType.kBrushless);
    br = new CANSparkMax(Constants.NEO_BR, MotorType.kBrushless);
    bl = new CANSparkMax(Constants.NEO_BL, MotorType.kBrushless);
    fl = new CANSparkMax(Constants.NEO_FL, MotorType.kBrushless);

    fr.setSmartCurrentLimit(currentLimit);
    br.setSmartCurrentLimit(currentLimit);
    fl.setSmartCurrentLimit(currentLimit);
    bl.setSmartCurrentLimit(currentLimit);
  }
  public void setSpeedFalcon(double left, double right){
    fr.set(right);
    br.set(right);
    fl.set(left);
    bl.set(left);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void initDefaultCommand() {
		setDefaultCommand(new DriveWithJoysticks());
	}
}
