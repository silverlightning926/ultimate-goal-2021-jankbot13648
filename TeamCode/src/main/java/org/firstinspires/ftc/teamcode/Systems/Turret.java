package org.firstinspires.ftc.teamcode.Systems;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Turret {
    public Servo turret;
    public Turret (HardwareMap hardwareMap)
    {
        turret = hardwareMap.get(Servo.class, Constants.EXTENDER_SERVO_NAME);

    }
    public void turretPresets(double position)
    {
        turret.setPosition(position);
    }
}

