package frc.robot;

public class Constants {
    // IDs are set correctly
    public static final int INTAKE_MOTOR_ID = 12;
    public static final int AGITATOR_MOTOR_ID = 56;
    public static final int SHOOT_MOTOR_ID = 15; // SHOOTY MOTOR ID
    public static final int LOADER_MOTOR_ID = 21; // in da load MOTOR ID
    public static final int CLIMB_RIGHT_ID = 8;
    public static final int CLIMB_LEFT_ID = 10;
    public static final int CLIMB_LIMIT_RIGHT = 0;
    public static final int CLIMB_LIMIT_LEFT = 1;
    public static final int[] CAMERA_TAGS = {9,10, 25,26};
    public static final int CAMERA_MODE = 2;
    public static final String LIMELIGHT_NAME = "snake";

    // CONTROLLER AXES
    public static final int CONTROLLER_LY_AXIS = 1;
    public static final int CONTROLLER_RY_AXIS = 5;

    // Non id constants
    // Most of these are set in the respective subsystem classes
    public static final double AXIS_THRESHOLD = 0.5; 

    // TURN
    // - FRONT_RIGHT_TURN: 16
    // - BACK_RIGHT_TURN: 20
    // - BACK_LEFT_TURN: 18
    // - FRONT_LEFT_TURN: 24

    // CANCODER
    // - FRONT_RIGHT_CANCODER: 17
    // - FRONT_LEFT_CANCODER: 22
    // - BACK_LEFT_CANCODER: 29
    // - BACK_RIGHT_CANCODER: 19

    // DRIVE
    // - FRONT_RIGHT_DRIVE: 25
    // - BACK_LEFT_DRIVE: 23
    // - BACK_RIGHT_DRIVE: 40
    // - FRONT_LEFT_DRIVE: 39

    // PIGEON: 20

    // 19 total devices
}
