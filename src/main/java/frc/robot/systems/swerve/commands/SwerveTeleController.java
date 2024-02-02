// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.swerve.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.montylib.profiles.ConfigurableMotionProfile;
import frc.montylib.profiles.ConfigurableMotionProfile.VariableSpeedMode;
import frc.robot.systems.swerve.Drive;
import frc.robot.systems.swerve.constants.DriveConstants;
import frc.robot.systems.swerve.constants.ModuleConstants;

public class SwerveTeleController extends Command {

  private Supplier<Double> xSupplier, ySupplier, rSupplier, slowSupplier, fastSupplier = null;
  private ConfigurableMotionProfile xMotionProfile, yMotionProfile, rMotionProfile = null;
  private Drive subsystem;

  public SwerveTeleController(
    Drive subsystem,
    CommandXboxController controller
  ) {

    this.subsystem = subsystem;

    xSupplier = () -> -controller.getRawAxis(1);
    ySupplier = () -> controller.getRawAxis(0);
    rSupplier = () -> controller.getRawAxis(4);
    slowSupplier = () -> controller.getRawAxis(2);
    fastSupplier = () -> controller.getRawAxis(3);

    xMotionProfile = new ConfigurableMotionProfile(ModuleConstants.kMaxModuleSpeed, true, true);
    yMotionProfile = new ConfigurableMotionProfile(ModuleConstants.kMaxModuleSpeed, true, true);
    rMotionProfile = new ConfigurableMotionProfile(ModuleConstants.kMaxModuleSpeed, true, true);

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    subsystem.resetHeading();
    subsystem.zeroModules();
  }

  @Override
  public void execute() {

    double x = xSupplier.get();
    double y = ySupplier.get();
    double r = rSupplier.get();
    double slow = slowSupplier.get();
    double fast = fastSupplier.get();

    x = xMotionProfile.calculate(VariableSpeedMode.BOTH, x, fast, slow);
    y = yMotionProfile.calculate(VariableSpeedMode.BOTH, y, fast, slow);
    r = rMotionProfile.calculate(VariableSpeedMode.BOTH, r, fast, slow);

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, r, subsystem.getRotation2d());

    subsystem.setDesiredSpeeds(speeds);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  public void configureMotionProfiles() {
    xMotionProfile.configureAcceleration(3);
    xMotionProfile.configureTriSpeedControl(DriveConstants.kPrimarySpeed, DriveConstants.kSecondarySpeed, DriveConstants.kTertiarySpeed);

    yMotionProfile.configureAcceleration(3);
    yMotionProfile.configureTriSpeedControl(DriveConstants.kPrimarySpeed, DriveConstants.kSecondarySpeed, DriveConstants.kTertiarySpeed);

    rMotionProfile.configureAcceleration(3);
    rMotionProfile.configureTriSpeedControl(DriveConstants.kPrimaryAngularSpeed, DriveConstants.kSecondaryAngularSpeed, DriveConstants.kTertiaryAngularSpeed);
  }
} 
