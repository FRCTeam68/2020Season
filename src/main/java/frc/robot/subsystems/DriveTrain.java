/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */ 
  private WPI_TalonSRX fr; //front right
  private WPI_TalonSRX br; //back right
  private WPI_TalonSRX bl; //back left
  private WPI_TalonSRX fl; //front left


  public static DriveTrain driveTrain;

	public static DriveTrain getDriveTrain() {
		if (driveTrain == null) {
			driveTrain = new DriveTrain();
		}
		return driveTrain;
	}
  public DriveTrain() {
    fr = new WPI_TalonSRX(Constants.FALCON_FR);
    br = new WPI_TalonSRX(Constants.FALCON_BR);
    bl = new WPI_TalonSRX(Constants.FALCON_BL);
    fl = new WPI_TalonSRX(Constants.FALCON_FL);

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
  public void setSpeedFalcon(double left, double right){
    fr.set(ControlMode.PercentOutput,right);
    br.set(ControlMode.PercentOutput,right);
    fl.set(ControlMode.PercentOutput,left);
    bl.set(ControlMode.PercentOutput,left);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}