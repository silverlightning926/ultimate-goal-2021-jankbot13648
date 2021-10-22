package org.firstinspires.ftc.teamcode.Systems;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Extender {
    public Servo extender;
    public Extender(HardwareMap hardwareMap)
    {
        extender = hardwareMap.get(Servo.class, Constants.INTAKE_MOTOR_NAME);

    }
    public void extenderPresets(double position)
    {
      extender.setPosition(position);
    }
}

