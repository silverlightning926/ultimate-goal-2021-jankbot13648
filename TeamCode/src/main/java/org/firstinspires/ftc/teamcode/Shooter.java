package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Shooter {

    DcMotorEx shooterMotor1;
    DcMotorEx shooterMotor2;

    Servo kicker;

    double currentTime = 0;

    public Shooter(HardwareMap hardwareMap)
    {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_1_NAME);
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_2_NAME);

        kicker = hardwareMap.get(Servo.class, Constants.KICKER_NAME);
        kicker.setPosition(Constants.KICKER_OPEN_POS);

        shooterMotor1.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        shooterMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void SetShooter()
    {
        shooterMotor1.setVelocity(Constants.SHOOTER_VELOCITY, AngleUnit.DEGREES);
        shooterMotor2.setVelocity(Constants.SHOOTER_VELOCITY, AngleUnit.DEGREES);;
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
