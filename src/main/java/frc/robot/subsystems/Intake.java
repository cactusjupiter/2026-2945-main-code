package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Helpers;

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

    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    Helpers.applyConfig(intakeMotor, intakeConfig);
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
}
