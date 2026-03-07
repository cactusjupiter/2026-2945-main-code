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

public class Shooter extends SubsystemBase {

    // desired loaderMotor power (%) and shootMotor velocity (rev/second)
    private static final double SHOOTER_SPEED = 52.0;

    // initialize motors
    private TalonFX shootMotor = new TalonFX(Constants.SHOOT_MOTOR_ID);
    
    // initialize velocity based controller
    private static final VelocityVoltage shooterVelocityController = new VelocityVoltage(0).withSlot(0); 

    // stop the shooter with neutral control
    private NeutralOut shooterBreak = new NeutralOut();

  public Shooter() {
    // configure the shooterMotor
    TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();

    // convention: positive power ejects from shooter
    shooterMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // brake when in neutral
    shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // configure velocity control paramters
    // no I and D terms
    shooterMotorConfig.Slot0.kS = 0.1;  // pass in to cancel out back-emf of motor
    shooterMotorConfig.Slot0.kV = 0.12; // volts per rpm
    shooterMotorConfig.Slot0.kP = 0.11; // proportion
    shooterMotorConfig.Slot0.kI = 0;
    shooterMotorConfig.Slot0.kD = 0;

    // apply shooterMotor config
    Helpers.applyConfig(shootMotor, shooterMotorConfig);
  }

  // shoot to eject
  public Command shooterShootCommand() {
    return run(
        () -> {
          // run the shooter at desired velocity 
          setShooterSpeed(SHOOTER_SPEED);
        }).finallyDo(
        () -> {
          // stop shooot motors
          stopShooter();
        });
  }

  // reverse to un-jam shooter
  public Command shooterReverseCommand() {
    return run(
        () -> {
          // run the shooter at desired velocity 
          setShooterSpeed(-SHOOTER_SPEED);
        }).finallyDo(
        () -> {
          stopShooter();
        });
  }

  private void setShooterSpeed(double velocity) {
    shootMotor.setControl(shooterVelocityController.withVelocity(velocity));        
  }

  private void stopShooter() {
    shootMotor.setControl(shooterBreak);
  } 
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("SHOOT_MOTOR_POWER", shootMotor.get());
    // TODO: is this required?
    shootMotor.getVelocity().refresh();
    SmartDashboard.putNumber("SHOOT_MOTOR_VEL", shootMotor.getVelocity().getValue().magnitude());
  }
}
