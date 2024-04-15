package frc.robot.Subsystem.ShooterArm;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

public interface ShooterArmConstants {
    //Technical Constants
    int SHOOTER_ARM_ID = 5;
    int SWITCH_ID = 9;
    int ENCODER_COUNTS_PER_REVOLUTION = 1;
    double GEAR_RATIO = 100 / 22 * 100;
    double TICKS_PER_DEGREE = ENCODER_COUNTS_PER_REVOLUTION * GEAR_RATIO / 360.0;

    // MotionMagic Constants
    double MM_CRUISE = 80;
    double MM_ACCELERATION = 300;
    double MM_JERK = 1600;

    double KP = 0.1;
    double KD = 0.0;
    double KS  = 0.032658;
    double KA = 0.001121;
    double KV = 0.13707;

    double PEEK_FORWARD_VOLTAGE = 11.5;
    double PEEK_REVERSE_VOLTAGE = -11.5;

    double FOWORD_LIMIT = 80;
    double BACKWARD_LIMIT = -1;

    MotionMagicVoltage MOTION_MAGIC_VOLTAGE = new MotionMagicVoltage(0,
    false,
    0.0,
    0,
    true,
    true,
    true);

    // Condition Constants
    double MINIMUM_ERROR = 1;
}
