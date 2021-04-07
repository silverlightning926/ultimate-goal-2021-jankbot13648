package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class WobbleGoal {

    Servo wobbleGoal1;
    Servo wobbleGoal2;

    Servo wobbleGoalManipulatorServo;

    public WobbleGoal(HardwareMap hardwareMap)
    {
        wobbleGoal1 = hardwareMap.get(Servo.class, Constants.WOBBLE_GOAL_SERVO1_NAME);
        wobbleGoal2 = hardwareMap.get(Servo.class, Constants.WOBBLE_GOAL_SERVO2_NAME);

        wobbleGoalManipulatorServo = hardwareMap.get(Servo.class, Constants.WOBBLE_GOAL_MANIPULATOR_SERVO);
    }

    public void MoveWobbleGoalPosition(boolean wobbleGoalUpButton, boolean wobbleGoalDownButton)
    {
        if(wobbleGoalUpButton)
        {
            wobbleGoal1.setPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[0]);
            wobbleGoal2.setPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[0]);
        }

        else if(wobbleGoalDownButton)
        {
            wobbleGoal1.setPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);
            wobbleGoal2.setPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);
        }
    }

    public void SetWobbleGoalPosition(double position)
    {
        wobbleGoal1.setPosition(position);
        wobbleGoal2.setPosition(position);
    }

    public void MoveWobbleGoalManipulator(boolean wobbleGoalOpenButton, boolean wobbleGoalCloseButton)
    {
        if(wobbleGoalOpenButton)
        {
            SetWobbleGoalManipulatorOpen();
        }

        else if(wobbleGoalCloseButton)
        {
            SetWobbleGoalManipulatorClose();
        }
    }

    public void SetWobbleGoalManipulatorOpen()
    {
        wobbleGoalManipulatorServo.setPosition(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_OPEN_POS);
    }

    public void SetWobbleGoalManipulatorClose()
    {
        wobbleGoalManipulatorServo.setPosition(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_CLOSE_POS);
    }
}
