package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    // Variable definition
    private TalonFX shootMotor = new TalonFX(Constants.SHOOT_MOTOR_ID);
    private static final double FIRE_SPEED = 0.8;

  public Shooter() {
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

    // TODO: figure out if this is counterclock of clock
    shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // TODO: Figure out if coast or break
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  }

  public Command shooterReverseCommand() {
    // intake into the robot
    return run(
        () -> {
            setShooterPower(FIRE_SPEED);
        }).finallyDo(
        () -> {
            stopShooter();
        });
  }

  public Command shooterShootCommand() {
    // reverse the motor to remove balls
    return run(
        () -> {
            setShooterPower(-FIRE_SPEED);
        }).finallyDo(
        () -> {
            stopShooter();
        });
  }

  private void setShooterPower(double power) {
    shootMotor.set(power);
  }

  private void stopShooter() {
    setShooterPower(0.0);
  } 

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
