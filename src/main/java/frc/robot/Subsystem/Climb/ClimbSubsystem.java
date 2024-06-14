/**
 * This subsystem is resposible for the "climbing"
 * @arthur Eilon.h
 * @Version 2.1.4
 */


package frc.robot.Subsystem.Climb;

import com.revrobotics.CANSparkMax;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase implements ClimbConstants{
  private CANSparkMax m_rightMotor;
  private CANSparkMax m_leftMotor;
  
  private static ClimbSubsystem instance; // singelton
  public static ClimbSubsystem getInstance(){ 
    if (instance == null){
      instance = new ClimbSubsystem();
    }
    return instance;
  }

  /** Creates a new ClimbSubsystem. */
  private ClimbSubsystem() {
    m_rightMotor = new CANSparkMax(M_CLIMBING_RIGHT_MOTOR_ID, MotorType.kBrushless);
    m_leftMotor = new CANSparkMax(M_CLIMBING_LEFT_MOTOR_ID, MotorType.kBrushless);

    configs();
  }

  public DoubleSupplier getSpeed()
  {
    return (() -> m_rightMotor.getAppliedOutput());
  }
  
  /**
   * set the speed of both motors
   * @param speed electrecity between -1 to 1 
   */
  public Command moveClimb(DoubleSupplier speed){
    return run(() -> {
      m_leftMotor.set(speed.getAsDouble());
      m_rightMotor.set(speed.getAsDouble());
    });
  }

  /**
   * Creates a command that makes the climb motors move with custom left and right speeds.
   *
   * @param  leftSpeed  a DoubleSupplier that provides the speed for the left motor
   * @param  rightSpeed a DoubleSupplier that provides the speed for the right motor
   * @return            a Command that moves the climb motors with the provided speeds
   */
  public Command moveClimb(DoubleSupplier rightMotorSpeed, DoubleSupplier leftMotorSpeed){
  return run(() ->{
    m_leftMotor.set(leftMotorSpeed.getAsDouble());
    m_rightMotor.set(rightMotorSpeed.getAsDouble());
  });
  }

  @Override
  public void periodic() {}

  private void configs(){
     m_leftMotor.setInverted(false);
    m_rightMotor.setInverted(true);

    m_rightMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    m_leftMotor.setSmartCurrentLimit(CURRENT_LIMIT);

    m_leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }
}