package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Created by Chun on 22 November 2019 for SHP Tennis.
 */

@TeleOp

public class DualTeleOp extends BaseRobot {
    private boolean toggle = false;
    private float leftSide = 0;
    private float rightSide = 0;
    private float mecanum = 0;

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
        if (!toggle) {
            leftSide = gamepad1.left_stick_y;
            rightSide = gamepad1.right_stick_y;
            mecanum = gamepad1.right_stick_x;
        } else {
            leftSide = -gamepad1.right_stick_y;
            rightSide = -gamepad1.left_stick_y;
            mecanum = -gamepad1.right_stick_x;
        }
        tankanum_drive(leftSide, rightSide, mecanum);

        //intake
        if (gamepad2.right_trigger > 0.1)
            intakeOut(-gamepad2.right_trigger);
        else if (gamepad2.left_trigger > 0.1)
            intakeOut(gamepad2.left_trigger);
        else
            intakeOut(0);

        //p1 intake
        if (gamepad1.right_trigger > 0.1)
            intakeOut(-gamepad1.right_trigger);
        else if (gamepad1.left_trigger > 0.1)
            intakeOut(gamepad1.left_trigger);
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

        if (gamepad2.a) {
            toggle = false;
        } else if (gamepad2.b) {
            toggle = true;
        }
    }
}