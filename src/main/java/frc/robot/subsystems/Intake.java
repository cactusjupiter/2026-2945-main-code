package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    // Variable definition
    private TalonFX intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_ID);
    private static final double INTAKE_SPEED = 0.5;

  public Intake() {
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

    // TODO: figure out if this is counterclock of clock
    intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // TODO: Figure out if coast or break
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  }

  public Command intakeInCommand() {
    // intake into the robot
    return run(
        () -> {
            setIntakePower(INTAKE_SPEED);
        }).finallyDo(
        () -> {
            stopIntake();
        });
  }

  public Command intakeOutCommand() {
    // reverse the motor to remove balls
    return run(
        () -> {
            setIntakePower(-INTAKE_SPEED);
        }).finallyDo(
        () -> {
            stopIntake();
        });
  }

  private void setIntakePower(double power) {
    intakeMotor.set(power);
  }

  private void stopIntake() {
    setIntakePower(0.0);
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
