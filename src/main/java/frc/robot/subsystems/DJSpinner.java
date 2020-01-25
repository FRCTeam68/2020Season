/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DJSpinner extends SubsystemBase {
  /**
   * Creates a new DJSpinner.
   */
  //private final WPI_TalonSRX spinner;

  private final ColorSensorV3 sensor;

  private static DJSpinner djSpinner;

  public static DJSpinner getDJSpinner() {
    if (djSpinner == null) {
      djSpinner = new DJSpinner();
    }
    return djSpinner;
  }

  private DJSpinner() {
    //spinner = new WPI_TalonSRX(Constants.SPINNER_MOTOR);
    //spinner.setNeutralMode(NeutralMode.Brake);

    //spinner.configPeakOutputForward(1);
    //spinner.configPeakOutputReverse(-1);

    sensor = new ColorSensorV3(Constants.i2cPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public int getBlueValue() {
    int RGB_Blue;
    RGB_Blue = sensor.getBlue();
    return RGB_Blue;
  }

  public int getGreenValue() {
    int RGB_Green;
    RGB_Green = sensor.getGreen();
    return RGB_Green;
  }

  public int getRedValue() {
    int RGB_Red;
    RGB_Red = sensor.getRed();
    return RGB_Red;
  }
}