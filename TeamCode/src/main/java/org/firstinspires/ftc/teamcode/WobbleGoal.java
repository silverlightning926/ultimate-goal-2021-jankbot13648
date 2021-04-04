package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

    public void MoveWobbleGoalPosition(boolean wobbleGoalUpButton, boolean wobbleGoalMiddleButton, boolean wobbleGoalDownButton)
    {
        if(wobbleGoalUpButton)
        {
            wobbleGoal1.setPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[0]);
            wobbleGoal2.setPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[0]);
        }

        else if (wobbleGoalMiddleButton)
        {
            wobbleGoal1.setPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);
            wobbleGoal2.setPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);
        }

        else if(wobbleGoalDownButton)
        {
            wobbleGoal1.setPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);
            wobbleGoal2.setPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);
        }
    }

    public void GoToWobbleGoalPosition(double position)
    {

        wobbleGoal1.setPosition(position);
        wobbleGoal2.setPosition(position);
    }

    public void WobbleGoalManipulatorHandler(boolean wobbleGoalOpenButton, boolean wobbleGoalCloseButton)
    {
        if(wobbleGoalOpenButton)
        {
            wobbleGoalManipulatorServo.setPosition(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_OPEN_POS);
        }

        else if(wobbleGoalCloseButton)
        {
            wobbleGoalManipulatorServo.setPosition(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_CLOSE_POS);
        }
    }

    public void GoToPosWobbleGoalManipulatorHandler(double position)
    {
        wobbleGoalManipulatorServo.setPosition(position);
    }
}
