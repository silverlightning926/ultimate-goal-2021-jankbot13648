package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Constants {

    public static final String RING_DETECTION_CAMERA_NAME = "Webcam 2";
    public static final String GOAL_DETECTION_CAMERA_NAME = "Webcam 1";
    public static int CAMERA_RESOLUTION_WIDTH = 1280;
    public static int CAMERA_RESOLUTION_HEIGHT = 720;

    // Drive Base Motor Names
    public static final String FRONT_LEFT_DRIVE_NAME = "fl";
    public static final String FRONT_RIGHT_DRIVE_NAME = "fr";
    public static final String REAR_LEFT_DRIVE_NAME = "rl";
    public static final String REAR_RIGHT_DRIVE_NAME = "rr";

    // Odometery Pod Names
    public static final String VERTICAL_LEFT_NAME = FRONT_LEFT_DRIVE_NAME;
    public static final String VERTICAL_RIGHT_NAME = FRONT_RIGHT_DRIVE_NAME;
    public static final String HORIZONTAL_NAME = REAR_LEFT_DRIVE_NAME;

    public static double GOAL_X_COORD = 144;
    public static double GOAL_Y_COORD = -17;

    public static double AUTO_AIM_OFFSET = -5.75;

    // Shooter Motor Names
    public static double AUTONOMOUS_SHOOTER_SPEED = 197;
    public static double TELEOP_SHOOTER_SPEED = 198;

    public static double POWER_SHOT_VELOCITY = 171.5;

    public static final String SHOOTER_1_NAME = "s1";
    public static final String SHOOTER_2_NAME = "s2";

    public static int SHOOTER_DELAY = 140;
    public static int DROP_DELAY = 170;

    public static final String KICKER_NAME = "kicker";

    public static double KICKER_OPEN_POS = 0.13;
    public static double KICKER_KICK_POS = 0.3;

    public static PIDFCoefficients SHOOTER_PID_COEFFICIENTS = new PIDFCoefficients(100, 1, 1, 12.5);

    public static double POWER_SHOT_TURN = 7.875;

    // Intake Constants
    public static final String INTAKE_TOP_NAME = "i1";
    public static final String INTAKE_BOTTOM_NAME = "i2";

    public static final String LEFT_WALL_NAME = "lw";
    public static final String RIGHT_WALL_NAME = "rw";

    public static final String LEFT_FUNNEL_NAME = "lf";
    public static final String RIGHT_FUNNEL_NAME = "rf";

    public static double LEFT_FUNNEL_RELEASE_POS = 0.5;
    public static double RIGHT_FUNNEL_RELEASE_POS = 0.5;

    public static double LEFT_FUNNEL_CLOSE_POS = 0.4;
    public static double RIGHT_FUNNEL_CLOSE_POS = 0.6;

    public static double LEFT_WALL_POS_OUT = 0.18;
    public static double RIGHT_WALL_POS_OUT = 0.9;

    public static double LEFT_WALL_POS_IN = 0.42;
    public static double RIGHT_WALL_POS_IN = 0.5;

    // Wobble Goal Constants
    public static final String WOBBLE_GOAL_SERVO1_NAME = "wg1";
    public static final String WOBBLE_GOAL_SERVO2_NAME = "wg2";

    public static final String AUTO_WOBBLE_CLAW = "awg";
    public static double AUTO_WOBBLE_CLAW_OPEN_POS = 1;
    public static double AUTO_WOBBLE_CLAW_CLOSE_POS = 0;

    public static double[] WOBBLE_GOAL_POSITION_VALUES = {0.2, 0.4, 0.75};

    public static final String WOBBLE_GOAL_MANIPULATOR_SERVO = "wgm";
    public static double WOBBLE_GOAL_MANIPULATOR_SERVO_OPEN_POS = 0.38;
    public static double WOBBLE_GOAL_MANIPULATOR_SERVO_CLOSE_POS = 0.85;
}
