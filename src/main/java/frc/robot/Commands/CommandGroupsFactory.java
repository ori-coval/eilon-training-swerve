package frc.robot.Commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

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

    private static final CommandSwerveDrivetrain swerve = TunerConstants.Swerve; // My drivetrain
    private static final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private static final ShooterArmSubsystem shooterArm = ShooterArmSubsystem.getInstance();
    private static final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private static final ClimbSubsystem climb = ClimbSubsystem.getInstance();

    private static double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private static double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    public static SwerveRequest.FieldCentric teleopDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

    public static SwerveRequest.FieldCentricFacingAngle driveAlignedToSpeaker = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) 
            .withDriveRequestType(DriveRequestType.Velocity);

    public static Command getShootBaseCommand(){
        return new ParallelDeadlineGroup(Commands.waitSeconds(0.02)//until 1 rio cycle is complted
        .andThen(Commands.waitUntil(() -> shooter.isDownReady(ShooterConstants.SHOOT_CLOSE_SPEED) && shooter.isUpReady(ShooterConstants.SHOOT_FAR_SPEED) && shooterArm.isArmReady())
        .andThen(() -> intake.feedShooterCommand())), // feed the note to the Shooter
         shooterArm.moveArmTo(0),//constants
         shooter.setSpeed());
    }

}
