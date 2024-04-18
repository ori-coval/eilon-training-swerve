/**
 * This subsystem is resposible for the "climbing"
 * @arthur Eilon.h
 * @Version 2.0.0
 */


package frc.robot.Subsystem.Climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

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
    m_rightMotor = new CANSparkMax(M_CLIMBINGRIGHT_MOTOR_ID, MotorType.kBrushless);
    m_leftMotor = new CANSparkMax(M_CLIMBINGLEFT_MOTOR_ID, MotorType.kBrushless);
    m_leftMotor.setInverted(false); //TODO: understand
    m_rightMotor.setInverted(true);
  }
  
  @Override
  public void periodic() {}

  /**
   * set the speed of both motors
   * @param speed electrecity between -1 to 1 
   */
  public void setSpeed(double speed){
    m_rightMotor.set(speed);
    m_leftMotor.set(speed);
  }

  /**
   * set the speed of the right motors
   * @param speed electrecity between -1 to 1 
   */
  public void setRightSpeed(double speed){
    m_rightMotor.set(speed);
  }

   /**
   * set the speed of the left motors
   * @param speed electrecity between -1 to 1 
   */
  public void setLeftSpeed(double speed){
    m_leftMotor.set(speed);
  }
}
