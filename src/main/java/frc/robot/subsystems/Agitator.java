package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Helpers;

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

  public Command agitatorCWCommand() {
    // agitator into the robot
    return run(
        () -> {
            setAgitatorPower(AGITATOR_SPEED);
        }).finallyDo(
        () -> {
            stopAgitator();
        });
  }

  public Command agitatorCCWCommand() {
    // reverse the motor to remove balls
    return run(
        () -> {
            setAgitatorPower(-AGITATOR_SPEED);
        }).finallyDo(
        () -> {
            stopAgitator();
        });
  }

  public Command agitatorDefaultCommand() {
    return new SequentialCommandGroup(
      agitatorCCWCommand().withTimeout(3.0),
      agitatorCWCommand().withTimeout(2.0)
    );
  }

  private void setAgitatorPower(double power) {
    agitatorMotor.set(ControlMode.PercentOutput, power);
  }

  private void stopAgitator() {
    setAgitatorPower(0.0);
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
