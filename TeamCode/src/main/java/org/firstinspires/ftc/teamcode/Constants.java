package org.firstinspires.ftc.teamcode;

public class Constants {
    // Drive Base Motor Names
    public static String FRONT_LEFT_DRIVE_NAME = "fl";
    public static String FRONT_RIGHT_DRIVE_NAME = "fr";
    public static String REAR_LEFT_DRIVE_NAME = "rl";
    public static String REAR_RIGHT_DRIVE_NAME = "rr";

    // Odometery Pod Names
    public static String VERTICAL_LEFT_NAME = FRONT_LEFT_DRIVE_NAME;
    public static String VERTICAL_RIGHT_NAME = FRONT_RIGHT_DRIVE_NAME;
    public static String HORIZONTAL_NAME = REAR_LEFT_DRIVE_NAME;

    // Shooter Motor Names
    public static double SHOOTER_VELOCITY = 210;

    public static String SHOOTER_1_NAME = "s1";
    public static String SHOOTER_2_NAME = "s2";

    public static String KICKER_NAME = "kicker";

    public static double KICKER_OPEN_POS = 0.21;
    public static double KICKER_KICK_POS = 0.03;

    // Intake Constants
    public static String INTAKE_TOP_NAME = "i1";
    public static String INTAKE_BOTTOM_NAME = "i2";

    public static String LEFT_WALL_NAME = "lw";
    public static String RIGHT_WALL_NAME = "rw";

    public static double LEFT_WALL_POS_OUT = 0.35;
    public static double RIGHT_WALL_POS_OUT = 0.85;

    public static double LEFT_WALL_POS_IN = 0.5;
    public static double RIGHT_WALL_POS_IN = 0.6;

    // Wobble Goal Constants
    public static String WOBBLE_GOAL_SERVO1_NAME = "wg1";
    public static String WOBBLE_GOAL_SERVO2_NAME = "wg2";

    public static double[] WOBBLE_GOAL_POSITION_VALUES = {0.1, 0.5, 0.8};

    public static String WOBBLE_GOAL_MANIPULATOR_SERVO = "wgm";
    public static double WOBBLE_GOAL_MANIPULATOR_SERVO_OPEN_POS = 0.8;
    public static double WOBBLE_GOAL_MANIPULATOR_SERVO_CLOSE_POS = 0;
}
