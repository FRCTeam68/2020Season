package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.DriveWithJoysticks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class DriveTrain extends SubsystemBase{
    private WPI_TalonSRX fr;
    private WPI_TalonSRX br;
    private WPI_TalonSRX bl;
    private WPI_TalonSRX fl;

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
        fl = new WPI_TalonSRX(Constants.FALCON_FL);
        bl = new WPI_TalonSRX(Constants.FALCON_BL);

        fr.setNeutralMode(NeutralMode.Brake); //Set to Brake When Neutral
        br.setNeutralMode(NeutralMode.Brake);
        fl.setNeutralMode(NeutralMode.Brake);
        bl.setNeutralMode(NeutralMode.Brake);

        fr.configPeakOutputForward(1); //Setting Peak Forward Speed
        br.configPeakOutputForward(1);
        fl.configPeakOutputForward(1);
        bl.configPeakOutputForward(1);

        fr.configPeakOutputReverse(-1); //Setting Peak Reverse Speed
        br.configPeakOutputReverse(-1);
        fl.configPeakOutputReverse(-1);
        bl.configPeakOutputReverse(-1);
    }
    public void setSpeedFalcon(double left, double right) {
        fr.set(ControlMode.PercentOutput,right);
        br.set(ControlMode.PercentOutput,right);
        fl.set(ControlMode.PercentOutput,left);
        bl.set(ControlMode.PercentOutput,left);
    }

    @Override
    public void periodic() {
        CommandScheduler.getInstance().setDefaultCommand(Robot.driveTrain, new DriveWithJoysticks());
    }
}
