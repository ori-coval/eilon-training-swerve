package frc.robot.subsystem.ShooterArm;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import frc.robot.util.exterpolation.ExterpolationMap;

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

    double PEAK_CURRENT = 0; //TODO: Find out what this is

    double FOWORD_LIMIT = 80;
    double BACKWARD_LIMIT = -1;

    MotionMagicVoltage MOTION_MAGIC_VOLTAGE = new MotionMagicVoltage(0,
    true,
    0.0,
    0,
    true,
    true,
    true);

    // Condition Constants
    double MINIMUM_ERROR = 1;
    double RESET_SPEED = -0.1;

    ExterpolationMap SPEAKER_ANGEL_EXTERPOLATION = new ExterpolationMap().put(2.9, 62.0)
    .put(2.52, 55.0)
    .put(2.15, 50.0)
    .put(3.59, 63.0)
    .put(3.04, 56.0); //TODO:update
}
