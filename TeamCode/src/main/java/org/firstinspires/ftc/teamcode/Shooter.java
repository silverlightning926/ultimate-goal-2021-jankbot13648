package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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

        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Constants.SHOOTER_PID_COEFFICIENTS);
        shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Constants.SHOOTER_PID_COEFFICIENTS);
    }

    public void SetShooter(double shooterSpeed)
    {
        shooterMotor1.setVelocity(shooterSpeed, AngleUnit.DEGREES);
        shooterMotor2.setVelocity(shooterSpeed, AngleUnit.DEGREES);
    }

    public void Kick()
    {
        kicker.setPosition(Constants.KICKER_KICK_POS);
    }

    public void Unkick()
    {
        kicker.setPosition(Constants.KICKER_OPEN_POS);
    }
}
