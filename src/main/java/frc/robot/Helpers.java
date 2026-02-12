package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class Helpers {
    public static boolean applyConfig(TalonFX motor, TalonFXConfiguration config) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i ++) {
            status = motor.getConfigurator().apply(config);
            if (status.isOK()) {
                System.out.println("Configs applied successfully!");
                return true;
            }
        }
        System.err.println("Configs NOT applied succesfully!!! Try again, loser: " + status.toString());
        return false;
    }
}
