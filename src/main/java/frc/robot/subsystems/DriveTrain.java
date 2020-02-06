package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.DriveWithXboxJoysticks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */
  private TalonFX fr; // front right
  private TalonFX br; // back right
  private TalonFX bl; // back left
  private TalonFX fl; // front left

  //Solenoid Gear Shifter
  private DoubleSolenoid shiftGear;

    
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

    // Gear Shift Solenoid

    shiftGear = new DoubleSolenoid(Constants.DRIVE_SHIFTER_PCM_A, Constants.DRIVE_SHIFTER_PCM_B);
    this.setShiftLow(); 
  }

     public void setShifterHigh() {
        shiftGear.set(Value.kForward);
    } 

      public void setShiftLow() {
        shiftGear.set(Value.kReverse);
    }

    public void shiftGear() {
      if(this.getShifter() == Value.kForward){
        this.setShiftLow();
      } 
      else {
      this.setShifterHigh();
      }
 
    }

      public DoubleSolenoid.Value getShifter() {
        return shiftGear.get();
      }


  public void setSpeedFalcon(double left, double right) {
    fr.set(ControlMode.PercentOutput, right);
    br.set(ControlMode.PercentOutput, right);
    fl.set(ControlMode.PercentOutput, left);
    bl.set(ControlMode.PercentOutput, left);
  }

  @Override
  public void periodic() {
    CommandScheduler.getInstance().setDefaultCommand(Robot.driveTrain, new DriveWithXboxJoysticks());
  }
}