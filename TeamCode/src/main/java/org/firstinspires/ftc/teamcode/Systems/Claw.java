package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Claw {
    public Servo claw;
    public Claw(HardwareMap hardwareMap)
    {
        Claw = hardwareMap.get(Servo.class, Constants.CLAW_SERVO_NAME);

    }
    public void openClaw()
    {
        Claw.setPosition(1);
    }
    public void closeClaw()
    {
        Claw.setPosition(0.5);
    }
}

