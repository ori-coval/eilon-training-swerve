// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Subsystem.swerve.*;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  public final static CommandXboxController driverJoystick = new CommandXboxController(0); // My driverJoystick
  public final static CommandXboxController operatorJoystick = new CommandXboxController(1); // My operatorJoystick
  public final CommandSwerveDrivetrain swerve = TunerConstants.Swerve; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  /* Path follower */
  private Command runAuto = swerve.getAutoPath("Tests");

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    swerve.setDefaultCommand( // Drivetrain will execute this command periodically
        swerve.applyRequest(() -> drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    driverJoystick.a().whileTrue(swerve.applyRequest(() -> brake));
    driverJoystick.b().whileTrue(swerve
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driverJoystick.leftBumper().onTrue(swerve.runOnce(() -> swerve.seedFieldRelative()));

    swerve.registerTelemetry(logger::telemeterize);

    driverJoystick.pov(0).whileTrue(swerve.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    driverJoystick.pov(180).whileTrue(swerve.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));


    /* Bindings for drivetrain characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
    driverJoystick.back().and(driverJoystick.y()).whileTrue(swerve.sysIdDynamic(Direction.kForward));
    driverJoystick.back().and(driverJoystick.x()).whileTrue(swerve.sysIdDynamic(Direction.kReverse));
    driverJoystick.start().and(driverJoystick.y()).whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
    driverJoystick.start().and(driverJoystick.x()).whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return runAuto;
  }
}
