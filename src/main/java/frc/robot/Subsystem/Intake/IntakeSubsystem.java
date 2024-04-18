/**
 * This subsystem is resposible for the "intake"
 * @arthur Eilon.h
 * @Version 2.0.0
 */

package frc.robot.Subsystem.Intake;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase implements IntakeConstants{
  private TalonFX m_motor;
  private DigitalInput m_opticSensor;

  private VoltageOut voltageOut = new VoltageOut(0, false, true, false, false);

  private static IntakeSubsystem instance; //singelton
  public static IntakeSubsystem getInstance(){
    if (instance == null){
      instance = new IntakeSubsystem();
    }
    return instance;
  }

  /** Creates a new IntakeSubsystem. */
  private IntakeSubsystem() {
    m_motor = new TalonFX(MOTOR_ID, Constants.CAN_BUS_NAME);
    m_opticSensor = new DigitalInput(OPTIC_SENSOR_ID);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration(); //configs

    motorConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = LIMIT_ENABLE;
    motorConfig.CurrentLimits.SupplyCurrentThreshold = CURRENT_THRESHOLD;
    motorConfig.CurrentLimits.SupplyTimeThreshold = TIME_THRESHOLD;
    motorConfig.MotorOutput.Inverted = MOTOR_DIRECTION;
    motorConfig.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;

    //upload configs to motor
    StatusCode statusCode = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++){
      m_motor.getConfigurator().apply(motorConfig);
      if (statusCode.isOK())
        break;
    }
    if (!statusCode.isOK())
      System.out.println("Intake could not apply config, error code:" + statusCode.toString());
  }

  /**
   * Set the speed of the intake
   * @param voltage amount of voltage
   */
  public void setSpeed(double voltage){
    m_motor.setControl(voltageOut.withOutput(voltage));
  }

   /**
    * Cheks if the sensor is true or false
    * @return the state of the sensor
    */
  public boolean getOpticSensorValue() {
    return m_opticSensor.get();
  }

  /**
   * Get the Velocity of the motor
   * @return the velocity
   */
  public double getVelocity() {
    return m_motor.getVelocity().getValueAsDouble();
  }

  /**
   * If the motor is at shooting speed
   * @return if the motor is at shooting speed
   */
  public boolean atShootSpeed(){
    return getVelocity() >= SHOOTING_SPEED;
  }


  @Override
  public void periodic() {
  }
}
