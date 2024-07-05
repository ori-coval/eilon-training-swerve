package frc.robot.Commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import frc.robot.RobotContainer;
import frc.robot.Subsystem.Climb.ClimbSubsystem;
import frc.robot.Subsystem.Intake.IntakeSubsystem;
import frc.robot.Subsystem.Shooter.ShooterConstants;
import frc.robot.Subsystem.Shooter.ShooterSubsystem;
import frc.robot.Subsystem.ShooterArm.ShooterArmConstants;
import frc.robot.Subsystem.ShooterArm.ShooterArmSubsystem;
import frc.robot.Subsystem.swerve.CommandSwerveDrivetrain;
import frc.robot.Subsystem.swerve.TunerConstants;

public class CommandGroupsFactory {
    private static boolean climbing = false;// are we climbing or intaking

    private static final CommandXboxController driverJoystick = RobotContainer.driverJoystick;
    private static final CommandXboxController operatorJoystick = RobotContainer.operatorJoystick;

    private static final CommandSwerveDrivetrain swerve = TunerConstants.Swerve; // My drivetrain
    private static final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private static final ShooterArmSubsystem shooterArm = ShooterArmSubsystem.getInstance();
    private static final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private static final ClimbSubsystem climb = ClimbSubsystem.getInstance();

    private static double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private static double SlowSpeed = 0.3 * MaxSpeed;
    private static double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    public static SwerveRequest.FieldCentric teleopDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

    public static SwerveRequest.FieldCentricFacingAngle driveAlignedToSpeaker = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) 
            .withDriveRequestType(DriveRequestType.Velocity);

    //swerve commands
    public static Command getDriveAlignedToSpeakerCommand() {//TODO: target direction
        return swerve.applyRequest(() -> driveAlignedToSpeaker
                .withVelocityX(-driverJoystick.getLeftY() * MaxSpeed)
                .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed)
                .withTargetDirection(null))
                .ignoringDisable(true);
    }
    
    public static Command getTeleopDriveCommand() {
        if (driverJoystick.rightBumper().getAsBoolean()){
            return swerve.applyRequest(() -> teleopDrive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate))
                    .ignoringDisable(true);
        }
        return swerve.applyRequest(() -> teleopDrive.withVelocityX(-driverJoystick.getLeftY() * SlowSpeed)
                    .withVelocityY(-driverJoystick.getLeftX() * SlowSpeed)
                    .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate))
                    .ignoringDisable(true);

    }
    // shotter commands
    /**
     * shoot from base with out camera or swerve
     */
    public static Command getShootBaseCommand(){
        return new ParallelDeadlineGroup(Commands.waitSeconds(0.02)//until 1 rio cycle is complted
        .andThen(Commands.waitUntil(() -> shooter.isBothAtVelocity(ShooterConstants.SHOOT_CLOSE_SPEED) && shooterArm.isArmReady())
        .andThen(() -> intake.feedShooterCommand())), // feed the note to the Shooter
         shooterArm.moveArmTo(0),//TODO:constants
         shooter.setShootingSpeed(ShooterConstants.SHOOT_CLOSE_SPEED));
    }
    /**
     * shoot with camera and swerve
     */
    public static Command getShootSpeakerCommand(){
        return new ParallelDeadlineGroup(Commands.waitSeconds(0.02)//until 1 rio cycle is complted
        .andThen(Commands.waitUntil(() -> shooter.isBothAtVelocity(ShooterConstants.SHOOT_CLOSE_SPEED) && shooterArm.isArmReady())
        .andThen(() -> intake.feedShooterCommand())) // feed the note to the Shooter

        , new InstantCommand(() -> swerve.setDefaultCommand(getDriveAlignedToSpeakerCommand())),
        shooterArm.moveArmTo(0),//TODO:constants
        shooter.setShootingSpeed(ShooterConstants.SHOOT_FAR_SPEED));
    }

    public static Command getPrepareShooterToShootFar(){
        return new ParallelCommandGroup(
            shooterArm.moveArmTo(0),//TODO:interpolation
            shooter.setShootingSpeed(ShooterConstants.SHOOT_FAR_SPEED));
    }
    public static Command getPrepareShooterToShootBase(){
        return new ParallelCommandGroup(
            shooterArm.moveArmTo(0),//TODO:constants
            shooter.setShootingSpeed(ShooterConstants.SHOOT_CLOSE_SPEED));
    }
    //TODO: add clmbing commands


}
