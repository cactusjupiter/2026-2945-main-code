package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Helpers;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    // Variable definition
    private TalonFX climb1Motor = new TalonFX(Constants.CLIMB_RIGHT_ID);
    private TalonFX climb2Motor = new TalonFX(Constants.CLIMB_LEFT_ID);
    private TalonFX pivot1Motor = new TalonFX(Constants.PIVOT_1_ID);
    private TalonFX pivot2Motor = new TalonFX(Constants.PIVOT_2_ID);
    private static final double CLIMB_SPEED = 0.8;

    private DigitalInput climbTopLimit = new DigitalInput(Constants.CLIMB_LIMIT_LEFT);
    private DigitalInput climbBtmLimit = new DigitalInput(Constants.CLIMB_LIMIT_RIGHT);

  public Climber() {
    TalonFXConfiguration climberConfig = new TalonFXConfiguration();

    // TODO: figure out if this is counterclock of clock
    climberConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    Helpers.applyConfig(climb1Motor, climberConfig);
    Helpers.applyConfig(climb2Motor, climberConfig);

    /*TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

    // TODO: figure out if this is 
    ounterclock of clock
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    Helpers.applyConfig(pivot1Motor, pivotConfig);
    Helpers.applyConfig(pivot2Motor, pivotConfig);*/
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

  public Command climbAutoCommand() {
    return new SequentialCommandGroup(
      climbUpCommand().until(this::climberAtTop),
      climbDownCommand().withTimeout(1.0)
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

  private void setClimberPower(double power) {
    climb1Motor.set(power);
    climb2Motor.set(power);
  }

  private void stopClimber() {
    setClimberPower(0.0);
  } 

  private boolean climberAtTop() {
    return !climbTopLimit.get();
  }

  private boolean climberAtBottom() {
    return !climbBtmLimit.get();
  }
}
