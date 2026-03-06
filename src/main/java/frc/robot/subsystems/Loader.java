package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Helpers;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Loader extends SubsystemBase {

    // desired loaderMotor power (%) and shootMotor velocity (rev/second)
    private static final double LOADER_POWER = 1; //muahahahahahahaha fast

    // initialize motors
    private TalonFX loaderMotor = new TalonFX(Constants.LOADER_MOTOR_ID);
    
  public Loader() {

    // configure the loaderMotor
    TalonFXConfiguration loaderMotorConfig = new TalonFXConfiguration();

    // convention: positive power pulls from agitator
    loaderMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // brake when in neutral
    loaderMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // apply loaderMotor config
    Helpers.applyConfig(loaderMotor, loaderMotorConfig);
  }

  // shoot to eject
  public Command loaderShootCommand() {
    return run(
        () -> {
          // run the loader at desired power
          setLoaderPower(LOADER_POWER);
        }).finallyDo(
        () -> {
          // stop both motors
          stopLoader();
        });
  }

  // reverse to un-jam shooter
  public Command loaderReverseCommand() {
    return run(
        () -> {
          // run the loader at desired power
          setLoaderPower(-LOADER_POWER);
        }).finallyDo(
        () -> {
          stopLoader();
        });
  }

  private void setLoaderPower(double power) {
    loaderMotor.set(power);
  }

  private void stopLoader() {
    setLoaderPower(0.0);
  } 
}
