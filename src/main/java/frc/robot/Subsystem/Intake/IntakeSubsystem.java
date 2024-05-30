/**
 * This subsystem is resposible for the "intake"
 * @arthor Eilon.H
 * @Version 2.1.0
 */

package frc.robot.Subsystem.Intake;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase implements IntakeConstants{
  private TalonFX m_motor;
  private DigitalInput m_opticSensor;
  private TorqueCurrentFOC currentFOC = new TorqueCurrentFOC(0);
  
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
   * @param Current amount of voltage
   */
  public Command setCurrent(double current){
    return runOnce(() -> m_motor.setControl(currentFOC.withOutput(current)));
  }

  /**
   * set the speed without FOC
   * @param speed
   * @return
   */
   public Command setSpeed(double speed){
    return runOnce(() -> m_motor.set(speed));
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
   * feed the intake
   * @return
   */
  public Command intakeCommand(){
    return runEnd(() -> setCurrent(INTAKE_CURRENT),() -> m_motor.stopMotor()).until(() -> getOpticSensorValue());
  }

  /**
   * feed the shooter
   * @return
   */
  public Command feedShooterCommand(){
    return runEnd(() -> setCurrent(FEED_SHOOTER_CURRENT),() -> m_motor.stopMotor()).withTimeout(FEED_SHOOTER_TIME);
  }


  @Override
  public void periodic() {
  }
}
