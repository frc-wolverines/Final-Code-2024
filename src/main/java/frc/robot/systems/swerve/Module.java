package frc.robot.systems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.montylib.hardware.NEOv1;
import frc.robot.systems.swerve.constants.ModuleConstants;

public class Module {

    private NEOv1 driveNEO, pivotNEO = null;
    
    private RelativeEncoder driveEncoder, pivotEncoder = null;
    private CANcoder absoluteEncoder = null;

    private PIDController pivotController = null;

    public Module(ModuleInterface module_interface, PIDConstants pivot_control_constants) {

        driveNEO = new NEOv1(module_interface.drive_motor_can_id);
        pivotNEO = new NEOv1(module_interface.pivot_motor_can_id);
        
        driveEncoder = driveNEO.getEncoder();
        configureDriveEncoder();
        pivotEncoder = pivotNEO.getEncoder();
        configurePivotEncoder();

        absoluteEncoder = new CANcoder(module_interface.can_coder_can_id);

        pivotController = new PIDController(
            pivot_control_constants.kP, 
            pivot_control_constants.kI, 
            pivot_control_constants.kD
        );
        pivotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    //Feedback
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }
    
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getPivotPosition() {
        return pivotEncoder.getPosition();
    }

    public double getPivotVelocity() {
        return pivotEncoder.getVelocity();
    }

    public double getAbsolutePosition() {
        return -(absoluteEncoder.getAbsolutePosition().getValueAsDouble() * Math.PI * 2);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromRadians(getPivotPosition());
    }

    //Movement
    public void setDesiredState(SwerveModuleState state) {

        if (state.speedMetersPerSecond < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getRotation2d());

        driveNEO.set(state.speedMetersPerSecond / ModuleConstants.kMaxModuleSpeed);
        pivotNEO.set(pivotController.calculate(getPivotPosition(), state.angle.getRadians()));

    }

    public void stop() {
        driveNEO.stopMotor();
        pivotNEO.stopMotor();
    }

    //Utility
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        pivotEncoder.setPosition(0);
    }

    public void zeroPivotEncoder() {

    }

    public void configureDriveEncoder() {
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveGearRatio * Units.inchesToMeters(4));
        driveEncoder.setVelocityConversionFactor((ModuleConstants.kDriveGearRatio * Units.inchesToMeters(4)) / 60);
    }

    public void configurePivotEncoder() {
        pivotEncoder.setPositionConversionFactor(ModuleConstants.kPivotGearRatio * 2 * Math.PI);
        pivotEncoder.setVelocityConversionFactor((ModuleConstants.kPivotGearRatio * 2 * Math.PI) / 60);
    }
}
