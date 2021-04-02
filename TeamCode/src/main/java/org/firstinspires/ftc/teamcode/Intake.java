package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    public DcMotorEx intake1;
    public DcMotorEx intake2;

    public Servo leftWall;
    public Servo rightWall;

    public Intake(HardwareMap hardwareMap)
    {
        intake1 = hardwareMap.get(DcMotorEx.class, Constants.INTAKE_TOP_NAME);
        intake2 = hardwareMap.get(DcMotorEx.class, Constants.INTAKE_BOTTOM_NAME);

        leftWall = hardwareMap.get(Servo.class, Constants.LEFT_WALL_NAME);
        rightWall = hardwareMap.get(Servo.class, Constants.RIGHT_WALL_NAME);

        intake1.setDirection(DcMotorEx.Direction.FORWARD);
        intake2.setDirection(DcMotorEx.Direction.FORWARD);

        intake1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intake1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void SetIntake(double intakeSpeed, double reverseIntakeSpeed)
    {
        if(reverseIntakeSpeed < 0.05)
        {
            intake1.setPower(intakeSpeed);
            intake2.setPower(intakeSpeed);
        }

        else {
            intake1.setPower(-reverseIntakeSpeed);
            intake2.setPower(-reverseIntakeSpeed);
        }

        if(intakeSpeed != 0)
        {
            SetWallPosition(Constants.LEFT_WALL_POS_OUT, Constants.RIGHT_WALL_POS_OUT);
        }

        else{
            SetWallPosition(Constants.LEFT_WALL_POS_IN, Constants.RIGHT_WALL_POS_IN);
        }
    }

    public void SetWallPosition(double leftWallPos , double rightWallPos)
    {
        leftWall.setPosition(leftWallPos);
        rightWall.setPosition(rightWallPos);
    }
}
