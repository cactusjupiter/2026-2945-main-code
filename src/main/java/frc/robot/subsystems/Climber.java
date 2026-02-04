package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    // Variable definition
    private TalonFX climb1Motor = new TalonFX(Constants.CLIMB_1_ID);
    private TalonFX climb2Motor = new TalonFX(Constants.CLIMB_2_ID);
    private TalonFX pivot1Motor = new TalonFX(Constants.PIVOT_1_ID);
    private TalonFX pivot2Motor = new TalonFX(Constants.PIVOT_2_ID);
    private static final double FIRE_SPEED = 0.8;
    private static final boolean isPivoted = true;

  public Climber() {
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

    // TODO: figure out if this is counterclock of clock
    shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // TODO: Figure out if coast or break
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  }

  public Command climbUpCommand() {
    // intake into the robot
    return run(
        () -> {
            setClimberPower(FIRE_SPEED);
        }).finallyDo(
        () -> {
            stopClimber();
        });
  }

  public Command climbDownCommand() {
    // reverse the motor to remove balls
    return run(
        () -> {
            setClimberPower(-FIRE_SPEED);
        }).finallyDo(
        () -> {
            stopClimber();
        });
  }

  public Command pivotUpCommand() {
    // reverse the motor to remove balls
    return run(
        () -> {
            setPivotPower(FIRE_SPEED);
        }).finallyDo(
        () -> {
            stopPivot();
        });
  }

  public Command pivotDownCommand() {
    // reverse the motor to remove balls
    return run(
        () -> {
            setPivotPower(-FIRE_SPEED);
        }).finallyDo(
        () -> {
            stopPivot();
        });
  }

  private void setClimberPower(double power) {
    climb1Motor.set(power);
    climb2Motor.set(power);
  }

  private void stopClimber() {
    setClimberPower(0.0);
  } 

  private void setPivotPower(double power) {
    pivot1Motor.set(power);
    pivot2Motor.set(power);
  }

  private void stopPivot() {
    setPivotPower(0.0);
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
