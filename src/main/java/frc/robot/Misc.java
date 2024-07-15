package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Subsystem.ObjectDetectionCamera;
import frc.robot.Subsystem.Vision.VisionConstants;
import frc.robot.Subsystem.swerve.TunerConstants;

public class Misc {

    public static final String CAN_BUS_NAME = "canBus";
    //field dimensions
    public static final Translation2d BSpeakerPose = new Translation2d(0, 5.54);
    public static final Translation2d RSpeakerPose = new Translation2d(16.39, 5.54);

    DoubleSupplier distanceFromSpeaker = () -> TunerConstants.Swerve.getPose().getTranslation().getDistance(
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? RSpeakerPose
                    : BSpeakerPose);

    ObjectDetectionCamera objectDetectionCamera = new ObjectDetectionCamera(VisionConstants.K_NOTE_CAMERA_NAME);
    BooleanSupplier isWithinShootingRange = () -> distanceFromSpeaker.getAsDouble() <= 5; // TODO: update
}