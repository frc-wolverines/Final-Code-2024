package frc.robot.systems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.montylib.hardware.NavX2;
import frc.robot.systems.swerve.constants.DriveConstants;
import frc.robot.systems.swerve.constants.ModuleConstants;

public class Drive extends SubsystemBase{
    
    private Module leftFrontModule = new Module(ModuleConstants.leftFrontInterface, ModuleConstants.kPivotPIDConstants);
    private Module rightFrontModule = new Module(ModuleConstants.rightFrontInterface, ModuleConstants.kPivotPIDConstants);
    private Module leftBackModule = new Module(ModuleConstants.leftBackInterface, ModuleConstants.kPivotPIDConstants);
    private Module rightBackModule = new Module(ModuleConstants.rightBackInterface, ModuleConstants.kPivotPIDConstants);

    private NavX2 gyroscope = new NavX2();

    private StructArrayPublisher<SwerveModuleState> moduleStatePublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

    public Drive() {
        new Thread(() -> {
            try {

                Thread.sleep(1000);
                resetHeading();
                zeroModules();

            } catch (Exception e) {}
        })
        .start();
    }

    @Override
    public void periodic() {
        moduleStatePublisher.set(getStates());
    }

    public void resetHeading() {
        gyroscope.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyroscope.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public SwerveModuleState[] getStates() {
        return new SwerveModuleState[] {
            leftFrontModule.getState(),
            rightFrontModule.getState(),
            leftBackModule.getState(),
            rightBackModule.getState()
        };
    }

    public void setDesiredSpeeds(ChassisSpeeds speeds) {

        SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, 1);

        leftFrontModule.setDesiredState(states[0]);
        rightFrontModule.setDesiredState(states[1]);

        leftBackModule.setDesiredState(states[2]);
        rightBackModule.setDesiredState(states[3]);
    }

    public void stopModules() {
        leftFrontModule.stop();
        rightFrontModule.stop();

        leftBackModule.stop();
        rightBackModule.stop();
    }

    public void resetModules() {
        stopModules();

        leftFrontModule.resetEncoders();
        rightFrontModule.resetEncoders();

        leftBackModule.resetEncoders();
        rightBackModule.resetEncoders();
    }

    public void zeroModules() {
        stopModules();

        leftFrontModule.zeroPivotEncoder();
        rightFrontModule.zeroPivotEncoder();

        leftBackModule.zeroPivotEncoder();
        rightBackModule.zeroPivotEncoder();
    }
}
