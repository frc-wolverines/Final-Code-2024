package frc.robot.systems.swerve.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DriveConstants {

    public static double kTrackDistance = Units.inchesToMeters(24.0);
    
    public static Translation2d kLeftFrontFromCenter = new Translation2d(kTrackDistance, kTrackDistance);
    public static Translation2d kRightFrontFromCenter = new Translation2d(kTrackDistance, -kTrackDistance);
    public static Translation2d kLeftBackFromCenter = new Translation2d(-kTrackDistance, kTrackDistance);
    public static Translation2d kRightBackFromCenter = new Translation2d(-kTrackDistance, -kTrackDistance);

    public static SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        kLeftFrontFromCenter,
        kRightFrontFromCenter,
        kLeftBackFromCenter,
        kRightBackFromCenter
    );

    public static double kPrimarySpeed = 7.5;
    public static double kSecondarySpeed = 12.0;
    public static double kTertiarySpeed = 4.5;

    public static double kPrimaryAngularSpeed = 3 * Math.PI;
    public static double kSecondaryAngularSpeed = 6 * Math.PI;
    public static double kTertiaryAngularSpeed = 2 * Math.PI;
}
