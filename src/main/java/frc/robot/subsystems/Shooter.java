package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Helpers;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    // Variable definition
    private TalonFX shootMotor = new TalonFX(Constants.SHOOT_MOTOR_ID);
    private TalonFX hoodMotor = new TalonFX(Constants.HOOD_MOTOR_ID);
    private static final VelocityVoltage fireVelocityController = new VelocityVoltage(0).withSlot(0); // velocity based control

    private DigitalInput hoodOpenSwitch = new DigitalInput(Constants.HOOD_LIMIT_OPEN);
    private DigitalInput hoodClosedSwitch = new DigitalInput(Constants.HOOD_LIMIT_CLOSED);
    private NeutralOut shooterBreak = new NeutralOut();
    private boolean isHoodOpen = false;

    private static final double HOOD_SPEED = 0.8;
    private static final double FIRE_SPEED = 50;

  public Shooter() {

    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

    // TODO: figure out if this is counterclock of clock
    // open is positive
    hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    //initialize hood to closed
    CommandScheduler.getInstance().schedule(hoodCloseCommand());

    Helpers.applyConfig(hoodMotor, hoodConfig);

    // shooter
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

    // TODO: figure out if this is counterclock of clock
    shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // TODO: Figure out if coast or break
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    shooterConfig.Slot0.kS = 0.1; // pass in to cancel out back-emf of motor
    shooterConfig.Slot0.kV = 0.12; // volts per rpm
    shooterConfig.Slot0.kP = 0.11; // proportion
    shooterConfig.Slot0.kI = 0;
    shooterConfig.Slot0.kD = 0;

    Helpers.applyConfig(shootMotor, shooterConfig);
  }

  public Command shooterReverseCommand() {
    // intake into the robot
    return run(
        () -> {
            setShooterPower(FIRE_SPEED);
        }).finallyDo(
        () -> {
            stopShooter();
        });
  }

  public Command shooterShootCommand() {
    // reverse the motor to remove balls
    return run(
        () -> {
            setShooterPower(-FIRE_SPEED);
        }).finallyDo(
        () -> {
            stopShooter();
        });
  }

  public Command hoodCommand() {
    // close because is currently open
    if (isHoodOpen) {
      return hoodCloseCommand();
    } else { //open because is currently closed
      return hoodOpenCommand();
    }
  }

  private Command hoodCloseCommand() {
    return run(
      () -> {
          setHoodPower(-HOOD_SPEED);
      }).until(
        this::getHoodClosedSwitch
      ).finallyDo(
      () -> {
          stopHood();
      });
  }

  private Command hoodOpenCommand() {
    return run(
      () -> {
          setHoodPower(HOOD_SPEED);
      }).until(
        this::getHoodOpenSwitch
      ).finallyDo(
      () -> {
          stopHood();
      });
  }

  private boolean getHoodOpenSwitch() {
        return !hoodOpenSwitch.get();
  }

  private boolean getHoodClosedSwitch() {
        return !hoodClosedSwitch.get();
  }

  private void setShooterPower(double velocity) {
    shootMotor.setControl(fireVelocityController.withVelocity(velocity));        
  }

  private void stopShooter() {
    shootMotor.setControl(shooterBreak);
  } 

  private void setHoodPower(double power) {
    hoodMotor.set(power);
  }

  private void stopHood() {
    setHoodPower(0.0);
  } 
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("(FUNCLE) HOOD_LIMIT_CLOSED", getHoodClosedSwitch());
  
    SmartDashboard.putBoolean("(REEEEEEEEEEE) HOOD_LIMIT_OPEN", getHoodOpenSwitch());
  }
}
