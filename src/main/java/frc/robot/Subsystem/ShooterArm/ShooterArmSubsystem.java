/**
 * This subsystem is resposible for the "line up shot"
 * @arthur Eilon.h
 * @Version 2.0.1
 */

package frc.robot.Subsystem.ShooterArm;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterArmSubsystem extends SubsystemBase implements ShooterArmConstants{
  //
  private TalonFX m_shooterArmMotor;
  private static ShooterArmSubsystem instance;
  private double targetPose = 0;

  public static ShooterArmSubsystem getInstance(){
    if (instance == null)
      instance = new ShooterArmSubsystem();
    return instance;
  }

  /** Creates a new ShooterArmSubsystem. */
  private ShooterArmSubsystem() {
    m_shooterArmMotor = new TalonFX(SHOOTER_ARM_ID, Constants.CAN_BUS_NAME); // crearts new motor

    // create the full MotionMagic
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    MotionMagicConfigs mm = new MotionMagicConfigs();

    mm.MotionMagicCruiseVelocity = MM_CRUISE;
    mm.MotionMagicAcceleration = MM_ACCELERATION;
    mm.MotionMagicJerk = MM_JERK;

    //Slot:
    configuration.Slot0.kP = KP;
    configuration.Slot0.kD = KD;
    configuration.Slot0.kV = KV;
    configuration.Slot0.kS = KS;

  //Peeks:
    configuration.Voltage.PeakForwardVoltage = PEEK_FORWARD_VOLTAGE;
    configuration.Voltage.PeakReverseVoltage = PEEK_REVERSE_VOLTAGE;

    // forward and backward limits 
    configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FOWORD_LIMIT;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = BACKWARD_LIMIT;
    configuration.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    configuration.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;
    configuration.HardwareLimitSwitch.ReverseLimitEnable = true;

    configuration.Feedback.SensorToMechanismRatio = TICKS_PER_DEGREE; 
    configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    StatusCode statusCode = StatusCode.StatusCodeNotInitialized;

    //upload to motor
    for (int i = 0; i < 5; i++){
      statusCode = m_shooterArmMotor.getConfigurator().apply(configuration);
      if (statusCode.isOK())
        break;
    }
    if (!statusCode.isOK())
      System.out.println("Shooter Arm could not apply config, error code:" + statusCode.toString());
  }

   /**
    * Set the motor position
    * @param pose
    */
   public void setPosition(double pose) {
    m_shooterArmMotor.setPosition(pose);
  }


  public void moveArmTo(double degree){
    targetPose = degree;
    m_shooterArmMotor.setControl(MOTION_MAGIC_VOLTAGE.withPosition(degree));
  }

  /**
   * Gets the arm position
   * @return The posirion in degrees
   */
  public double getArmPose(){
    return m_shooterArmMotor.getPosition().getValue();
  }

  
  public boolean isArmReady(){
    return (Math.abs(getArmPose() - targetPose) < MINIMUM_ERROR);
  }

  /**
   * Check if the switch is open
   * @return If the switch is open or not 
   */
  public boolean getSwitch(){
    return !(m_shooterArmMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround);
  }

  /**
   * Set the speed of the motor
   * @param speed Mhe wanted speed in double
   */
  public void setSpeed(double speed){
    m_shooterArmMotor.set(speed);
  }

  /**
   * Makes the motor go in to coast mode
   */
  public void coast() { //TODO: understand 
    m_shooterArmMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  /**
   * Makes the motor go in to break mode
   */
  public void breakMode() {
    m_shooterArmMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /**
   * Reset the arm
   */
  public void manualZeroShooterArm() {
    coast();
    if (getSwitch()) {
      while (getSwitch()) {
      }
    }
    while (!getSwitch()) {
    }
    SmartDashboard.putBoolean("zerod out shooter", true);
    setPosition(0);
    breakMode();
  }
  //TODO: Add speakerInterpolate 
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
