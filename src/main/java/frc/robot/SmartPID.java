/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Map;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class SmartPID {

    NetworkTableEntry entry_p = Shuffleboard.getTab("SmartPID").add("kProportion", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 10)).getEntry();
    NetworkTableEntry entry_i = Shuffleboard.getTab("SmartPID").add("kIntegral", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 10)).getEntry();
    NetworkTableEntry entry_d = Shuffleboard.getTab("SmartPID").add("kDerivitive", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 10)).getEntry();
    NetworkTableEntry entry_f = Shuffleboard.getTab("SmartPID").add("kFeedForward", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 10)).getEntry();
    NetworkTableEntry entry_setpoint = Shuffleboard.getTab("SmartPID").add("kSetPoint", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 7200)).getEntry();
    NetworkTableEntry entry_iZone = Shuffleboard.getTab("SmartPID").add("kiZone", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 10)).getEntry();

    /**
     * @param
     * These are prebuilt widgets that are already integrated into the
     * Shuffleboard app
    */
    public double getEntrySetPoint() {
        double rpm = entry_setpoint.getDouble(0);
        return rpm;
    }

    public double getEntryP() {
        double p = entry_p.getDouble(0);
        return p;
    }

    public double getEntryI() {
        double i = entry_i.getDouble(0);
        return i;
    }

    public double getEntryD() {
        double d = entry_d.getDouble(0);
        return d;
    }

    public double getEntryF() {
        double f = entry_f.getDouble(0);
        return f;
    }

    public double getEntryiZone() {
        double iZone = entry_iZone.getDouble(0);
        return iZone;
    }
    
    /** 
    * This will create a network table that will return the value that has
    * been set in the shuffle board app, this way you can dynamically tune PID
    * without having to restart the robot and redeploy code
    * @param entry The network table entry that will create the dashboard widget
    * @param VarName This string wll be the name of the dashboard widget
    * @param min The minimum input of the dashboard widget
    * @param max The maximum input of the dashboard widget
    */
    public double createCustomEntry(NetworkTableEntry entry ,String VarName,double min,double max){
        entry =  Shuffleboard.getTab("SmartPID").add(VarName, 0)
        .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", min, "max", max)).getEntry();
        double entryValue = entry.getDouble(0);
        return entryValue;
    }


}
