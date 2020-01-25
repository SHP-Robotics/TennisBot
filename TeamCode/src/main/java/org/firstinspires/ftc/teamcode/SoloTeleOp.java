package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Created by Chun on 22 November 2019 for SHP Tennis.
 */

@TeleOp

public class SoloTeleOp extends BaseRobot {
    private boolean toggle = false;
    private float leftSide = 0;
    private float rightSide = 0;
    private float mecanum = 0;
    private float inR = 0;
    private float inL = 0;

    @Override
    public void init() {
        super.init();
        gamepad1.setJoystickDeadzone(0.1f);
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();

        //drive train
        //tankanum_drive(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.right_stick_x);
        leftSide = gamepad1.left_stick_y;
        rightSide = gamepad1.right_stick_y;
        mecanum = gamepad1.right_stick_x;
        inR = gamepad1.right_trigger;
        inL = gamepad1.left_trigger;
//        if (!toggle) {
//
//        } else {
//            leftSide = -gamepad1.right_stick_y;
//            rightSide = -gamepad1.left_stick_y;
//            mecanum = -gamepad1.right_stick_x;
//            inR = gamepad1.left_trigger;
//            inL = gamepad1.right_trigger;
//        }
        tankanum_drive(leftSide, rightSide, mecanum);

        //intake
        if (inR > 0.1)
            intakeOut(-inR);
        else if (inL > 0.1)
            intakeOut(inL);
        else
            intakeOut(0);

        //foundation
        if (gamepad1.a) {
            foundationServoLeft.setPosition(ConstantVariables.K_FOUNDATION_LEFT_DOWN);
            foundationServoRight.setPosition(ConstantVariables.K_FOUNDATION_RIGHT_DOWN);
        } else if (gamepad1.b) {
            foundationServoLeft.setPosition(ConstantVariables.K_FOUNDATION_LEFT_UP);
            foundationServoRight.setPosition(ConstantVariables.K_FOUNDATION_RIGHT_UP);
        }

//        if (gamepad1.dpad_up) {
//            toggle = false;
//        } else if (gamepad2.dpad_down) {
//            toggle = true;
//        }
    }
}