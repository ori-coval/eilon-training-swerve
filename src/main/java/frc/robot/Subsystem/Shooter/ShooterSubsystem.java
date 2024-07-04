// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Shooter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase implements ShooterConstants{
  private TalonFX m_upMotor;
  private TalonFX m_downMotor;
  private final MotionMagicVelocityTorqueCurrentFOC mmVel = new MotionMagicVelocityTorqueCurrentFOC(0);

  //singelton
  private static ShooterSubsystem instance;
  public static ShooterSubsystem getInstance() {
      if (instance == null){instance = new ShooterSubsystem();}
      return instance;
  }
  /** Creates a new Shooter. */
  private ShooterSubsystem() {
    m_upMotor = new TalonFX(UP_MOTOR_ID,Constants.CAN_BUS_NAME);
    m_downMotor = new TalonFX(DOWN_MOTOR_ID,Constants.CAN_BUS_NAME);
    upConfigs();
    downConfigs();

  }

    /**
     * set the up motor at a given speed 
     * @param speed
     */
    public void setSpeedUp(double speed){
        m_upMotor.setControl(mmVel.withVelocity(speed));
    }
    /**
     * set the down motor at a given speed 
     * @param speed
     */
    public void setSpeedDown(double speed){
        m_downMotor.setControl(mmVel.withVelocity(speed));
    }
    /**
     * set both motors at a given speed 
     * @param speed
     * @return runOnce Command
     */
    public Command setShootingSpeed(double speed){
        return runOnce(() -> {
            setSpeedUp(speed);
            setSpeedDown(speed);
        });
    }
    /**
     * set both motos a a constants speed
     * @return runOnce Command
     */
    public Command setShootingSpeed(){
        return runOnce(() -> {
            setSpeedUp(SHOOT_FAR_SPEED);
            setSpeedDown(SHOOT_FAR_SPEED);
        });
    }
    /**
     * Check if the up motor is ready based on its velocity.
     *
     * @return true if the up motor velocity is less than the minimum error, false otherwise
     */
    public boolean isUpAtVelocity(double speed){
        return (Math.abs(m_upMotor.getVelocity().getValue()- speed) < MINIMUM_ERROR);
    }

    /**
     * Compute if the down motor is ready based on its velocity.
     *
     * @param  speed   the speed to compare the down motor velocity with
     * @return         true if the down motor velocity is close to the given speed within an error margin, false otherwise
     */
    public boolean isDownAtVelocity(double speed){
        return (Math.abs(m_downMotor.getVelocity().getValue()- speed) < MINIMUM_ERROR);
    }

    /**
     * Check if both up and down motors are ready based on their velocities.
     *
     * @param  speed   the speed to compare both motor velocities with
     * @return         true if both motors' velocities are close to the given speed within an error margin, false otherwise
     */
    public boolean isBothAtVelocity(double speed){
        return (isUpAtVelocity(speed) && isDownAtVelocity(speed));
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  private void upConfigs(){
    var talonFXConfigs = new TalonFXConfiguration();
    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = UP_KS; 
    slot0Configs.kA = UP_KA; 
    slot0Configs.kP = UP_KP; 
    slot0Configs.kI = UP_KI; 
    slot0Configs.kD = UP_KD; 

    talonFXConfigs.Feedback.SensorToMechanismRatio = SENSOR_TO_MEC_RATIO; // set sensor to mechanism ratio

    // set Motion Magic Velocity settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = ACCELERATION; 
    motionMagicConfigs.MotionMagicJerk = JERK; 

    //Limits
    var limitConfigs = new CurrentLimitsConfigs();
    // enable stator current limit
    limitConfigs.StatorCurrentLimit = CURRENT_LIMIT;
    limitConfigs.StatorCurrentLimitEnable = true;

    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // invert motor output
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;// set motor to coast mode

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
    slot0Configs.kS = DOWN_KS; 
    slot0Configs.kA = DOWN_KA; 
    slot0Configs.kP = DOWN_KP; 
    slot0Configs.kI = DOWN_KI; 
    slot0Configs.kD = DOWN_KD; 

    talonFXConfigs.Feedback.SensorToMechanismRatio = SENSOR_TO_MEC_RATIO; // set sensor to mechanism ratio

    // set Motion Magic Velocity settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = ACCELERATION; 
    motionMagicConfigs.MotionMagicJerk = JERK; 

    //Limits
    var limitConfigs = new CurrentLimitsConfigs();
    // enable stator current limit
    limitConfigs.StatorCurrentLimit = CURRENT_LIMIT;
    limitConfigs.StatorCurrentLimitEnable = true;

    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // invert motor output
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;// set motor to coast mode

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
