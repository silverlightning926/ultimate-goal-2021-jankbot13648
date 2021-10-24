package org.firstinspires.ftc.teamcode.Systems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
public class DuckSpinner {
    public DcMotorEx spinner;
    public DuckSpinner(HardwareMap hardwareMap)
    {
        spinner = hardwareMap.get(DcMotorEx.class, Constants.QUACK_SPINNER_NAME);
        spinner.setDirection(DcMotorSimple.Direction.FORWARD);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setSpinner()
    {

        if( spinner.getCurrentPosition() > 0.5)
        {
            spinner.setPower(0.8);
        }
        else
        {
            spinner.setPower(1);
        }
    }
}
