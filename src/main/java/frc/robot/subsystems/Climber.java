package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Helpers;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    // Variable definition
    private TalonFX climbRightMotor = new TalonFX(Constants.CLIMB_RIGHT_ID);
    private TalonFX climbLeftMotor = new TalonFX(Constants.CLIMB_LEFT_ID);
    //private TalonFX pivot1Motor = new TalonFX(Constants.PIVOT_1_ID);
    //private TalonFX pivot2Motor = new TalonFX(Constants.PIVOT_2_ID);
    private static final double CLIMB_SPEED = 0.05;
    private static final double CLIMB_DOWN_TIME = 3.0; // NOT downtime

    private DigitalInput climbLftLimit = new DigitalInput(Constants.CLIMB_LIMIT_LEFT);
    private DigitalInput climbRgtLimit = new DigitalInput(Constants.CLIMB_LIMIT_RIGHT);

  public Climber() {
    TalonFXConfiguration climberConfigRight = new TalonFXConfiguration();

    // TODO: figure out if this is counterclock of clock
    climberConfigRight.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    climberConfigRight.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    TalonFXConfiguration climberConfigLeft = new TalonFXConfiguration();

    // TODO: figure out if this is counterclock of clock
    climberConfigLeft.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    climberConfigLeft.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    Helpers.applyConfig(climbRightMotor, climberConfigRight);
    Helpers.applyConfig(climbLeftMotor, climberConfigLeft);

    /*TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

    // TODO: figure out if this is 
    ounterclock of clock
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    Helpers.applyConfig(pivot1Motor, pivotConfig);
    Helpers.applyConfig(pivot2Motor, pivotConfig);*/
  }

  public Command climbBothUpCommand() {
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
    return new SequentialCommandGroup(
      climbBothUpCommand(),
      climbBothDownCommand().withTimeout(CLIMB_DOWN_TIME)
    );
    
    /*run(
        () -> {
            setClimberPower(CLIMB_SPEED);
        }).until(
          climbTopLimit::get
        ).finallyDo(
        () -> {
            stopClimber();
        });*/
  }

  private void setLeftClimberPower(double power) {
    if (climberLeftAtTop()) {
      stopLeftClimber();
    } else {
      climbLeftMotor.set(power);
    }
  }

  private void setRightClimberPower(double power) {
    if (climberRightAtTop()) {
      stopRightClimber();
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
    return !climbLftLimit.get();
  }

  private boolean climberRightAtTop() {
    return !climbRgtLimit.get();
  }
}
