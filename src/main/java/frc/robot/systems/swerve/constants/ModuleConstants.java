package frc.robot.systems.swerve.constants;

import com.pathplanner.lib.util.PIDConstants;

import frc.robot.systems.swerve.ModuleInterface;

public class ModuleConstants {
    
    public static PIDConstants kPivotPIDConstants = new PIDConstants(0.5, 0.0, 0.0);
    public static double kDriveGearRatio = 1 / 8.14;
    public static double kPivotGearRatio = 1 / 21.4285714286;
    public static double kMaxModuleSpeed = 12.5;

    public static ModuleInterface leftFrontInterface = new ModuleInterface(
        1, 
        2, 
        3, 
        false
    );

    public static ModuleInterface rightFrontInterface = new ModuleInterface(
        4, 
        5, 
        6, 
        true
    );

    public static ModuleInterface leftBackInterface = new ModuleInterface(
        7, 
        8, 
        9, 
        false
    );

    public static ModuleInterface rightBackInterface = new ModuleInterface(
        10, 
        11, 
        12, 
        true
    );

}
