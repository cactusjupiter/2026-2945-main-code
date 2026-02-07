package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    // Variable definition
    private TalonFX climb1Motor = new TalonFX(Constants.CLIMB_1_ID);
    private TalonFX climb2Motor = new TalonFX(Constants.CLIMB_2_ID);
    private TalonFX pivot1Motor = new TalonFX(Constants.PIVOT_1_ID);
    private TalonFX pivot2Motor = new TalonFX(Constants.PIVOT_2_ID);
    private static final double CLIMB_SPEED = 0.8;
    private static final double PIVOT_SPEED = 0.8;
    private static final boolean isPivotFwd = false;

    private DigitalInput pivotFwdSwitch = new DigitalInput(Constants.PIVOT_LIMIT_FWD);
    private DigitalInput pivotBckSwitch = new DigitalInput(Constants.PIVOT_LIMIT_BCK);

  public Climber() {
    TalonFXConfiguration climberConfig = new TalonFXConfiguration();

    // TODO: figure out if this is counterclock of clock
    climberConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

    // TODO: figure out if this is counterclock of clock
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  }

  public Command climbUpCommand() {
    // intake into the robot
    return run(
        () -> {
            setClimberPower(CLIMB_SPEED);
        }).finallyDo(
        () -> {
            stopClimber();
        });
  }

  public Command climbDownCommand() {
    // reverse the motor to remove balls
    return run(
        () -> {
            setClimberPower(-CLIMB_SPEED);
        }).finallyDo(
        () -> {
            stopClimber();
        });
  }

  public Command pivotUpCommand() {
    // reverse the motor to remove balls
    return run(
        () -> {
            setPivotPower(CLIMB_SPEED);
        }).finallyDo(
        () -> {
            stopPivot();
        });
  }

  public Command pivotDownCommand() {
    // reverse the motor to remove balls
    return run(
        () -> {
            setPivotPower(-CLIMB_SPEED);
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

  public Command pivotCommand() {
    // close because is currently open
    if (isPivotFwd) {
      return pivotBckCommand();
    } else { //open because is currently closed
      return pivotFwdCommand();
    }
  }

  private Command pivotFwdCommand() {
    return run(
      () -> {
          setPivotPower(PIVOT_SPEED);
      }).until(
          pivotFwdSwitch::get
      ).finallyDo(
      () -> {
          stopPivot();
      });
  }

  private Command pivotBckCommand() {
    return run(
      () -> {
          setPivotPower(-PIVOT_SPEED);
      }).until(
        pivotBckSwitch::get
      ).finallyDo(
      () -> {
          stopPivot();
      });
  }

  private void setPivotPower(double power) {
    pivot1Motor.set(power);
    pivot2Motor.set(power);
  }

  private void stopPivot() {
    setPivotPower(0.0);
  } 
}
