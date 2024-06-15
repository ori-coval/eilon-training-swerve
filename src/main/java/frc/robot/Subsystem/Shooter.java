// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase implements ShooterConstants{
  private TalonFX m_upMotor;
  private TalonFX m_downMotor;
  private final MotionMagicExpoTorqueCurrentFOC mm = new MotionMagicExpoTorqueCurrentFOC(0);

  //singelton
  private static Shooter instance;
  public static Shooter getInstance() {
      if (instance == null){instance = new Shooter();}
      return instance;
  }
  /** Creates a new Shooter. */
  private Shooter() {
    m_upMotor = new TalonFX(UP_MOTOR_ID,Constants.CAN_BUS_NAME);
    m_downMotor = new TalonFX(DOWN_MOTOR_ID,Constants.CAN_BUS_NAME);
    upConfigs();
    downConfigs();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  private void upConfigs(){
    var talonFXConfigs = new TalonFXConfiguration();
    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = UP_KS; // Add 0.25 V output to overcome static friction
    slot0Configs.kA = UP_KA; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = UP_KP; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = UP_KI; // no output for integrated error
    slot0Configs.kD = UP_KD; // no output for error derivative

    // set Motion Magic Velocity settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = ACCELERATION; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = JERK; // Target jerk of 4000 rps/s/s (0.1 seconds)

    //Limits
    var limitConfigs = new CurrentLimitsConfigs();
    // enable stator current limit
    limitConfigs.StatorCurrentLimit = CURRENT_LIMIT;
    limitConfigs.StatorCurrentLimitEnable = true;

    //upload configs to motor
    StatusCode statusCode = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++){
      statusCode = m_upMotor.getConfigurator().apply(talonFXConfigs);
      if (statusCode.isOK())
        break;
    }
    if (!statusCode.isOK())
      System.out.println("Shooter Arm could not apply config, error code:" + statusCode.toString());
  }
  private void downConfigs(){
    var talonFXConfigs = new TalonFXConfiguration();
    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = DOWN_KS; // Add 0.25 V output to overcome static friction
    slot0Configs.kA = DOWN_KA; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = DOWN_KP; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = DOWN_KI; // no output for integrated error
    slot0Configs.kD = DOWN_KD; // no output for error derivative

    // set Motion Magic Velocity settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = ACCELERATION; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = JERK; // Target jerk of 4000 rps/s/s (0.1 seconds)

    //Limits
    var limitConfigs = new CurrentLimitsConfigs();
    // enable stator current limit
    limitConfigs.StatorCurrentLimit = CURRENT_LIMIT;
    limitConfigs.StatorCurrentLimitEnable = true;

    //upload configs to motor
    StatusCode statusCode = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++){
      statusCode = m_downMotor.getConfigurator().apply(talonFXConfigs);
      if (statusCode.isOK())
        break;
    }
    if (!statusCode.isOK())
      System.out.println("Shooter Arm could not apply config, error code:" + statusCode.toString());
  }
}
