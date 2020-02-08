/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Compressor;

public class Pnuematics extends SubsystemBase {
  /**
   * Creates a new Pnuematics.
   */

  public Compressor compressor;
  private DoubleSolenoid gearShifter;

  public Pnuematics() {
    //compressor = new Compressor(16);
    gearShifter = new DoubleSolenoid(16, Constants.DRIVE_SHIFTER_PCM_A, Constants.DRIVE_SHIFTER_PCM_B);
    setShifterHigh();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShifterHigh() {
    gearShifter.set(Value.kReverse);
  }

  public void setShiftLow() {
    gearShifter.set(Value.kForward);
  }
  public Value getShifter() {
    return gearShifter.get();
  }
  public void gearShifter() {
    if (this.getShifter() == Value.kReverse) {
      this.setShiftLow();
    } else {
      this.setShifterHigh();
    }
  }
}
