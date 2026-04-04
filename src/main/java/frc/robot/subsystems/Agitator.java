package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Agitator extends SubsystemBase {

    // Variable definition
    private VictorSPX agitatorMotor = new VictorSPX(Constants.AGITATOR_MOTOR_ID);
    private static final double AGITATOR_SPEED = 1.0;

  public Agitator() {
  }

  public Command agitatorCCWCommand() {
    // run agitator clockwise
    return run(
        () -> {
            setAgitatorPower(AGITATOR_SPEED);
        }).finallyDo(
        () -> {
            stopAgitator();
        });
  }

  public Command agitatorCWCommand() {
    // run agitator clockfoolish
    return run(
        () -> {
            setAgitatorPower(-AGITATOR_SPEED);
        }).finallyDo(
        () -> {
            stopAgitator();
        });
  }

  public Command agitatorJostleCommand() {
    // DURATION: 10 seconds
    //run clockfoolish 1.5 seconds, then 1 clockwise, repeat
    return new SequentialCommandGroup(
      agitatorCCWCommand().withTimeout(1.5),
      agitatorCWCommand().withTimeout(1.0),
      agitatorCCWCommand().withTimeout(1.5),
      agitatorCWCommand().withTimeout(1.0),
      agitatorCCWCommand().withTimeout(1.5),
      agitatorCWCommand().withTimeout(1.0),
      agitatorCCWCommand().withTimeout(1.5),
      agitatorCWCommand().withTimeout(1.0)
    );
  }

  private void setAgitatorPower(double power) {
    agitatorMotor.set(ControlMode.PercentOutput, power);
  }

  private void stopAgitator() {
    setAgitatorPower(0.0);
  }
}
