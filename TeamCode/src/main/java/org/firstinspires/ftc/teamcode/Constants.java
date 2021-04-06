package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Constants {

    public static String CAMERA_NAME = "Webcam 1";
    public static int CAMERA_RESOLUTION_WIDTH = 1280;
    public static int CAMERA_RESOLUTION_HEIGHT = 720;

    // Drive Base Motor Names
    public static String FRONT_LEFT_DRIVE_NAME = "fl";
    public static String FRONT_RIGHT_DRIVE_NAME = "fr";
    public static String REAR_LEFT_DRIVE_NAME = "rl";
    public static String REAR_RIGHT_DRIVE_NAME = "rr";

    // Odometery Pod Names
    public static String VERTICAL_LEFT_NAME = FRONT_LEFT_DRIVE_NAME;
    public static String VERTICAL_RIGHT_NAME = FRONT_RIGHT_DRIVE_NAME;
    public static String HORIZONTAL_NAME = REAR_LEFT_DRIVE_NAME;

    public static Vector2d GOAL_VECTOR2D = new Vector2d(132.989966, 0);

    // Shooter Motor Names
    public static double SHOOTER_VELOCITY = 195;
    public static double POWER_SHOT_VELOCITY = 217 * (8.0/10.0);

    public static String SHOOTER_1_NAME = "s1";
    public static String SHOOTER_2_NAME = "s2";

    public static String KICKER_NAME = "kicker";

    public static double KICKER_OPEN_POS = 0.35;
    public static double KICKER_KICK_POS = 0.05;

    public static PIDFCoefficients SHOOTER_PID_COEFFICIENTS = new PIDFCoefficients(10, 3, 0, 0);

    // Intake Constants
    public static String INTAKE_TOP_NAME = "i1";
    public static String INTAKE_BOTTOM_NAME = "i2";

    public static String LEFT_WALL_NAME = "lw";
    public static String RIGHT_WALL_NAME = "rw";

    public static String LEFT_FUNNEL_NAME = "lf";
    public static String RIGHT_FUNNEL_NAME = "rf";

    public static double LEFT_FUNNEL_RELEASE_POS = 0.5;
    public static double RIGHT_FUNNEL_RELEASE_POS = 0.5;

    public static double LEFT_WALL_POS_OUT = 0.75;
    public static double RIGHT_WALL_POS_OUT = 0.9;

    public static double LEFT_WALL_POS_IN = 0.45;
    public static double RIGHT_WALL_POS_IN = 0.6;

    // Wobble Goal Constants
    public static String WOBBLE_GOAL_SERVO1_NAME = "wg1";
    public static String WOBBLE_GOAL_SERVO2_NAME = "wg2";

    public static double[] WOBBLE_GOAL_POSITION_VALUES = {0.2, 0.5, 0.7};

    public static String WOBBLE_GOAL_MANIPULATOR_SERVO = "wgm";
    public static double WOBBLE_GOAL_MANIPULATOR_SERVO_OPEN_POS = 1.0;
    public static double WOBBLE_GOAL_MANIPULATOR_SERVO_CLOSE_POS = 0;
}
