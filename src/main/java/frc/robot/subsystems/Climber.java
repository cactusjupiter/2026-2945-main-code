package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Helpers;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    // Variable definition
    private TalonFX climbRightMotor = new TalonFX(Constants.CLIMB_RIGHT_ID);
    private TalonFX climbLeftMotor = new TalonFX(Constants.CLIMB_LEFT_ID);
    private static final double CLIMB_SPEED = 0.1;
    private static final double CLIMB_DOWN_TIME = 3.0; // NOT downtime

    private DigitalInput climbLftLimit = new DigitalInput(Constants.CLIMB_LIMIT_LEFT);
    private DigitalInput climbRgtLimit = new DigitalInput(Constants.CLIMB_LIMIT_RIGHT);

  public Climber() {
    // for the right motor
    TalonFXConfiguration climberConfigRight = new TalonFXConfiguration();

    climberConfigRight.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    climberConfigRight.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // for the left motor
    TalonFXConfiguration climberConfigLeft = new TalonFXConfiguration();

    climberConfigLeft.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    climberConfigLeft.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    Helpers.applyConfig(climbRightMotor, climberConfigRight);
    Helpers.applyConfig(climbLeftMotor, climberConfigLeft);
  }

  public Command climbBothUpCommand() {
    // raise both motors, then stop when it's the right time
    return run(
      () -> {
        setRightClimberPower(CLIMB_SPEED);
        setLeftClimberPower(CLIMB_SPEED);
      }).finallyDo(
        () -> {
            stopRightClimber();
            stopLeftClimber();
        });
  }

  public Command climbBothDownCommand() {
    // lower both motors, then stop at the right time
    return run(
      () -> {
        setRightClimberPower(-CLIMB_SPEED);
        setLeftClimberPower(-CLIMB_SPEED);
      }).finallyDo(
        () -> {
            stopRightClimber();
            stopLeftClimber();
        });
  }

  public Command climbAutoCommand() {
    // raise climber, then pull the robot up
    return new SequentialCommandGroup(
      climbBothUpCommand(),
      climbBothDownCommand().withTimeout(CLIMB_DOWN_TIME)
    );
  }

  private void setLeftClimberPower(double power) {
    // if the climber is not actively at the top and trying to raise, set the left power.
    if (climberLeftAtTop() && power > 0.0) {
      climbLeftMotor.set(0.0);
    } else {
      climbLeftMotor.set(power);
    }
  }

  private void setRightClimberPower(double power) {
    // if the climber is not actively at the top and trying to raise, set the right power.
    if (climberRightAtTop() && power > 0.0) {
      climbRightMotor.set(0.0);
    } else {
      climbRightMotor.set(power);
    }
  }

  private void stopLeftClimber() {
    setLeftClimberPower(0.0);
  } 

  private void stopRightClimber() {
    setRightClimberPower(0.0);
  } 

  private boolean climberLeftAtTop() {
    // get and invert the left limit switch's output
    // the raw value of this is the opposite of what we need
    return !climbLftLimit.get();
  }

  private boolean climberRightAtTop() {
    // get and invert the right limit switch's output
    // the raw value of this is the opposite of what we need
    return !climbRgtLimit.get();
  }
}
