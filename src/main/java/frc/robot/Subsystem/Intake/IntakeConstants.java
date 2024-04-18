package frc.robot.Subsystem.Intake;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public interface IntakeConstants {
    int MOTOR_ID = 8;
    int OPTIC_SENSOR_ID = 8;

    //configs
    double CURRENT_LIMIT = 40;   
    boolean LIMIT_ENABLE = true; 
    double CURRENT_THRESHOLD = 100;
    double TIME_THRESHOLD = 0.2;
    InvertedValue MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive;
    NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;

    double SHOOTING_SPEED = 0;
}
