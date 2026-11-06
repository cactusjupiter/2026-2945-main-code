package frc.robot;

import frc.robot.subsystems.Shooter;

public class Limelight {

    public double tx;
    public double ty;
    public double ta;
    public boolean hasTarget;
    public double txnc;
    public double tync;

    public String limelightName = "limelight-algae";

    public void updateData(){
        // Basic targeting data
        tx = LimelightHelpers.getTX(limelightName);  // Horizontal offset from crosshair to target in degrees
        ty = LimelightHelpers.getTY(limelightName);  // Vertical offset from crosshair to target in degrees
        ta = LimelightHelpers.getTA(limelightName);  // Target area (0% to 100% of image)
        hasTarget = LimelightHelpers.getTV(limelightName); // Do you have a valid target?

        txnc = LimelightHelpers.getTXNC(limelightName);  // Horizontal offset from principal pixel/point to target in degrees
        tync = LimelightHelpers.getTYNC(limelightName);  // Vertical offset from principal pixel/point to target in degrees
    }

    public void positionChangeShooterSpeed(){

        if (Math.abs(ta) <= 5 )
        {
            Shooter.SHOOTER_SPEED = 50;
        }

        if (Math.abs(ta) > 5 && Math.abs(ta) <= 20)
        {
            Shooter.SHOOTER_SPEED = 30;
        }

        if (Math.abs(ta) > 20 && Math.abs(ta) <= 40)
        {
            Shooter.SHOOTER_SPEED = 20;
        }

        if (Math.abs(ta) > 40 && Math.abs(ta) <= 60)
        {
            Shooter.SHOOTER_SPEED = 10;
        }
    }
}