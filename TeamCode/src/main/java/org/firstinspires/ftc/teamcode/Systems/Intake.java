package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Intake {

    public DcMotorEx intake1;
    public DcMotorEx intake2;

    public Servo leftWall;
    public Servo rightWall;

    public Servo leftFunnel;
    public Servo rightFunnel;

    boolean intakeHasBeenRun = false;

    public Intake(HardwareMap hardwareMap)
    {
        intake1 = hardwareMap.get(DcMotorEx.class, Constants.INTAKE_TOP_NAME);
        intake2 = hardwareMap.get(DcMotorEx.class, Constants.INTAKE_BOTTOM_NAME);

        leftWall = hardwareMap.get(Servo.class, Constants.LEFT_WALL_NAME);
        rightWall = hardwareMap.get(Servo.class, Constants.RIGHT_WALL_NAME);

        leftFunnel = hardwareMap.get(Servo.class, Constants.LEFT_FUNNEL_NAME);
        rightFunnel = hardwareMap.get(Servo.class, Constants.RIGHT_FUNNEL_NAME);

        intake1.setDirection(DcMotorEx.Direction.FORWARD);
        intake2.setDirection(DcMotorEx.Direction.FORWARD);

        intake1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intake1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setIntake(double intakeSpeed, double reverseIntakeSpeed)
    {
        if(reverseIntakeSpeed < 0.1 && intakeSpeed > 0.1)
        {
            intake1.setPower(intakeSpeed);
            intake2.setPower(intakeSpeed);
            setWallPosIn();
        }

        else {
            intake1.setPower(-reverseIntakeSpeed);
            intake2.setPower(-reverseIntakeSpeed);
        }
    }

    public void setIntake(double intakeSpeed, double reverseIntakeSpeed, boolean intakeWithoutWallsButton)
    {
        if(intakeSpeed > 0.1)
        {
            intake1.setPower(intakeSpeed);
            intake2.setPower(intakeSpeed);

            setWallPosDown();

            intakeHasBeenRun = true;
        }

        else if(intakeWithoutWallsButton)
        {
            setWallPosIn();

            intake1.setPower(1);
            intake2.setPower(1);

            intakeHasBeenRun = true;
        }

        else if (reverseIntakeSpeed > 0.1)
        {
            intake1.setPower(-reverseIntakeSpeed);
            intake2.setPower(-reverseIntakeSpeed);

            intakeHasBeenRun = true;
        }

        else if (intakeSpeed < 0.1 && intakeHasBeenRun)
        {
            setWallPosIn();
            intake1.setPower(0);
            intake2.setPower(0);

            intakeHasBeenRun = false;
        }

        else {
            intake1.setPower(0);
            intake2.setPower(0);
        }
    }


    public void setWallPosition(double leftWallPos, double rightWallPos)
    {
        leftWall.setPosition(leftWallPos);
        rightWall.setPosition(rightWallPos);
    }

    public void setWallPosDown()
    {
        leftWall.setPosition(Constants.LEFT_WALL_POS_OUT);
        rightWall.setPosition(Constants.RIGHT_WALL_POS_OUT);
    }

    public void setWallPosIn()
    {
        leftWall.setPosition(Constants.LEFT_WALL_POS_IN);
        rightWall.setPosition(Constants.RIGHT_WALL_POS_IN);
    }

    public void releaseFunnels()
    {
        leftFunnel.setPosition(Constants.LEFT_FUNNEL_RELEASE_POS);
        rightFunnel.setPosition(Constants.RIGHT_FUNNEL_RELEASE_POS);
    }
}
