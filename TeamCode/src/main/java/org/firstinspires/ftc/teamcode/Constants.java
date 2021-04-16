package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Constants {

    public static transient String CAMERA_NAME = "Webcam 1";
    public static int CAMERA_RESOLUTION_WIDTH = 1280;
    public static int CAMERA_RESOLUTION_HEIGHT = 720;

    // Drive Base Motor Names
    public static transient String FRONT_LEFT_DRIVE_NAME = "fl";
    public static transient String FRONT_RIGHT_DRIVE_NAME = "fr";
    public static transient String REAR_LEFT_DRIVE_NAME = "rl";
    public static transient String REAR_RIGHT_DRIVE_NAME = "rr";

    // Odometery Pod Names
    public static transient String VERTICAL_LEFT_NAME = FRONT_LEFT_DRIVE_NAME;
    public static transient String VERTICAL_RIGHT_NAME = FRONT_RIGHT_DRIVE_NAME;
    public static transient String HORIZONTAL_NAME = REAR_LEFT_DRIVE_NAME;

    public static Vector2d GOAL_VECTOR2D = new Vector2d(145, 0);

    // Shooter Motor Names
    public static double SHOOTER_VELOCITY = 195;
    public static double MIN_SHOOTER_VELOCITY = 195;
    public static double MAX_SHOOTER_VELOCITY = 208;

    public static double POWER_SHOT_VELOCITY = 217 * (8.0/10.0);

    public static transient String SHOOTER_1_NAME = "s1";
    public static transient String SHOOTER_2_NAME = "s2";

    public static int shooterDelay = 125;
    public static int dropDelay = 140;

    public static transient String KICKER_NAME = "kicker";

    public static double KICKER_OPEN_POS = 0.13;
    public static double KICKER_KICK_POS = 0.28;

    public static PIDFCoefficients SHOOTER_PID_COEFFICIENTS = new PIDFCoefficients(10, 1, 2, 9);

    // Intake Constants
    public static transient String INTAKE_TOP_NAME = "i1";
    public static transient String INTAKE_BOTTOM_NAME = "i2";

    public static transient String LEFT_WALL_NAME = "lw";
    public static transient String RIGHT_WALL_NAME = "rw";

    public static transient String LEFT_FUNNEL_NAME = "lf";
    public static transient String RIGHT_FUNNEL_NAME = "rf";

    public static double LEFT_FUNNEL_RELEASE_POS = 0.5;
    public static double RIGHT_FUNNEL_RELEASE_POS = 0.5;

    public static double LEFT_WALL_POS_OUT = 0.2;
    public static double RIGHT_WALL_POS_OUT = 0.9;

    public static double LEFT_WALL_POS_IN = 0.5;
    public static double RIGHT_WALL_POS_IN = 0.6;

    // Wobble Goal Constants
    public static transient String WOBBLE_GOAL_SERVO1_NAME = "wg1";
    public static transient String WOBBLE_GOAL_SERVO2_NAME = "wg2";

    public static transient String AUTO_WOBBLE_CLAW = "awg";
    public static double AUTO_WOBBLE_CLAW_OPEN_POS = 1;
    public static double AUTO_WOBBLE_CLAW_CLOSE_POS = 0;

    public static double[] WOBBLE_GOAL_POSITION_VALUES = {0.25, 0.5, 0.68};

    public static transient String WOBBLE_GOAL_MANIPULATOR_SERVO = "wgm";
    public static double WOBBLE_GOAL_MANIPULATOR_SERVO_OPEN_POS = 1.0;
    public static double WOBBLE_GOAL_MANIPULATOR_SERVO_CLOSE_POS = 0;
}
