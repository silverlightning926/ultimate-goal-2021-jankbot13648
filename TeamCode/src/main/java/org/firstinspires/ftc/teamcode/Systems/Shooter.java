package org.firstinspires.ftc.teamcode.Systems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class Shooter {

    DcMotorEx shooterMotor1;
    DcMotorEx shooterMotor2;

    Servo kicker;

    public Shooter(HardwareMap hardwareMap)
    {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_1_NAME);
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_2_NAME);

        kicker = hardwareMap.get(Servo.class, Constants.KICKER_NAME);

        shooterMotor1.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        shooterMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        shooterMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Constants.SHOOTER_PID_COEFFICIENTS);
        shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Constants.SHOOTER_PID_COEFFICIENTS);
    }

    public void autoSetShooter(double distanceFromLine)
    {
        double shooterSpeed = Math.min(Math.max((-(Math.pow(distanceFromLine, 4)/24)) + (Math.pow(distanceFromLine, 3)/4) + (Math.pow(distanceFromLine, 2)/24) + ((7*distanceFromLine)/4) + 195, Constants.MIN_SHOOTER_VELOCITY), Constants.MAX_SHOOTER_VELOCITY);
        //double shooterSpeed = Math.min(Math.max(184.688 + (10.1651 * (Math.pow(Math.E, 0.208726 * distanceFromLine))), Constants.MIN_SHOOTER_VELOCITY), Constants.MAX_SHOOTER_VELOCITY);

        shooterMotor1.setVelocity(shooterSpeed, AngleUnit.DEGREES);
        shooterMotor2.setVelocity(shooterSpeed, AngleUnit.DEGREES);
    }

    public void setShooter(double shooterSpeed)
    {
        shooterMotor1.setVelocity(shooterSpeed, AngleUnit.DEGREES);
        shooterMotor2.setVelocity(shooterSpeed, AngleUnit.DEGREES);
    }

    public void kick()
    {
        kicker.setPosition(Constants.KICKER_KICK_POS);
    }

    public void unKick()
    {
        kicker.setPosition(Constants.KICKER_OPEN_POS);
    }
}
