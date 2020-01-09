

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision extends SubsystemBase {

    private final NetworkTable Limelight;
    private final NetworkTableEntry V;
    private final NetworkTableEntry X;
    private final NetworkTableEntry Y;
    private final NetworkTableEntry A;
    

    public static Vision vision;

    public static Vision getVision() {
        if (vision == null) {
            vision = new Vision();
        }
        return vision;
    }

    public Vision() {
        Limelight = NetworkTableInstance.getDefault().getTable("LimeLight");
        V = Limelight.getEntry("tv");
        X = Limelight.getEntry("tx");
        Y = Limelight.getEntry("ty");
        A = Limelight.getEntry("ta");

    }

    @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean getTarget(){
   
    double v = V.getDouble(0.0);
    boolean isThereTarget;
    isThereTarget = false;
    if(v == 1){
      isThereTarget = true;
    }
    
    return isThereTarget;
  }

  public double getXValue(){
    double x = X.getDouble(0.0);
    return x;
  }
  public double getYValue(){
    double y = Y.getDouble(0.0);
    return y;
  }
  public double getArea(){
    double area = A.getDouble(0.0);
    return area;
  }

  public void setCameraMode(double ledMode, double camMode){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ledMode);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(camMode);
  }
  

  public double steeringAdjust(){
        
     double steeringAdjust = 0; 
     double x = X.getDouble(0.0);
     float Kp = -0.1f;
     float min_command = 0.05f;
     double heading_error = -x;

        if(x>1.0){
          steeringAdjust = Kp*heading_error-min_command;
        }
        else if(x<1.0){
          steeringAdjust = Kp*heading_error+min_command;
        }
        return steeringAdjust;
  }


}


