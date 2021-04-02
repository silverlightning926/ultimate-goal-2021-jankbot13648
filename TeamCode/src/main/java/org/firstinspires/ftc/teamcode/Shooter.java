package org.firstinspires.ftc.teamcode;

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
    }

    public void SetShooter()
    {
        shooterMotor1.setVelocity(Constants.SHOOTER_VELOCITY, AngleUnit.DEGREES);
        shooterMotor2.setVelocity(Constants.SHOOTER_VELOCITY, AngleUnit.DEGREES);;
    }

    public void Shoot(boolean shootButton)
    {
        if(shootButton)
        {
            for(int i = 0; i < 3; i++)
            {
                kicker.setPosition(Constants.KICKER_OPEN_POS);

                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                kicker.setPosition(Constants.KICKER_KICK_POS);

                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }
}
