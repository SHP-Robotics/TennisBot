package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
/*
 * Created by Chun on 22 November 2019 for SHP Tennis.
 */

@TeleOp
@Disabled

public class OldIMUTestTeleOp extends BaseRobot {
    private boolean toggle = false;
    private float leftSide = 0;
    private float rightSide = 0;
    private float mecanum = 0;
    private float inR = 0;
    private float inL = 0;

    BNO055IMU imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;

    @Override
    public void init() {
        super.init();
        gamepad1.setJoystickDeadzone(0.1f);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();

        //correction = checkDirection();

        telemetry.addData("1 imu heading", lastAngles.firstAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("3 correction", correction);


        //drive train
        //tankanum_drive(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.right_stick_x);
        if (!toggle) {
            leftSide = gamepad1.left_stick_y;
            rightSide = gamepad1.right_stick_y;
            mecanum = gamepad1.right_stick_x;
            inR = gamepad1.right_trigger;
            inL = gamepad1.left_trigger;
        } else {
            leftSide = -gamepad1.right_stick_y;
            rightSide = -gamepad1.left_stick_y;
            mecanum = -gamepad1.right_stick_x;
            inR = gamepad1.left_trigger;
            inL = gamepad1.right_trigger;
        }
        tankanum_drive(leftSide, rightSide, mecanum);

        //intake
        if (inR > 0.1)
            intake(-inR);
        else if (inL > 0.1)
            intake(inL);
        else
            intake(0);

        //foundation
        if (gamepad1.a) {
            foundationServoLeft.setPosition(ConstantVariables.K_FOUNDATION_LEFT_DOWN);
            foundationServoRight.setPosition(ConstantVariables.K_FOUNDATION_RIGHT_DOWN);
        } else if (gamepad1.b) {
            foundationServoLeft.setPosition(ConstantVariables.K_FOUNDATION_LEFT_UP);
            foundationServoRight.setPosition(ConstantVariables.K_FOUNDATION_RIGHT_UP);
        }

        if (gamepad1.dpad_up) {
            toggle = false;
        } else if (gamepad2.dpad_down) {
            toggle = true;
        }
    }
}