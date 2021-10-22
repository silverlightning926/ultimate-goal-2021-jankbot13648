package org.firstinspires.ftc.teamcode.Systems.Intake;



import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Intake {
     public DcMotorEx intake1;
     boolean intakeisRunning;
    public Intake(HardwareMap hardwareMap)
    {
        intake1 = hardwareMap.get(DcMotorEx.class, Constants.INTAKE_MOTOR_NAME);


        intake1.setDirection(DcMotorEx.Direction.FORWARD);

        intake1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intake1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void setIntake(double intakeSpeed, double reverseIntakeSpeed)
    {
        if(reverseIntakeSpeed < 0.1 && intakeSpeed > 0.1)
        {
            intake1.setPower(intakeSpeed);
        }
        else
        {
            intake1.setPower(-reverseIntakeSpeed);
        }
    }
}

