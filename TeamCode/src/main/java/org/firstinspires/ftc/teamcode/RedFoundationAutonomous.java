package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * Created by Chun on 17 January 2020 for SHP Tennis.
 */

@Autonomous
//@Disabled

public class RedFoundationAutonomous extends BaseRobot {
    private int stage = 0;

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();

        switch (stage) {
            case 0:
                //drive backwards to foundation
                if (auto_drive(-1, 40)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 1:
                //mecanum left to center
                if (auto_mecanum(-1, 22)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 2:
                //drive back to touch foundation
                if (auto_drive(-0.5, 6)) {
                    reset_drive_encoders();
                    stage++;
                    timer.reset();
                }
                break;
            case 3:
                //drop servos
                if (timer.seconds() > 0.3) {
                    reset_drive_encoders();
                    stage++;
                } else {
                    foundationServoLeft.setPosition(ConstantVariables.K_FOUNDATION_LEFT_DOWN);
                    foundationServoRight.setPosition(ConstantVariables.K_FOUNDATION_RIGHT_DOWN);
                }
                break;
            case 4:
                //turn slightly right
                if (auto_turn(1,20)) {
                    reset_drive_encoders();
                    stage++;
                }
            case 5:
                //drive forward
                if (auto_drive(1, 25)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 6:
                //turn foundation right 90 degrees
                if (auto_turn(1, 360)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 7:
                //drive backwards to push foundation into wall
                if (auto_drive(-1, 25)) {
                    reset_drive_encoders();
                    timer.reset();
                    stage++;
                }
                break;
            case 8:
                //raise servos
                if (timer.seconds()>0.3) {
                    reset_drive_encoders();
                    stage++;
                } else {
                    foundationServoLeft.setPosition(ConstantVariables.K_FOUNDATION_LEFT_UP);
                    foundationServoRight.setPosition(ConstantVariables.K_FOUNDATION_RIGHT_UP);
                }
                break;
            case 9:
                //drive to under bridge
                if (auto_drive(1, 48)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            default:
                break;
        }
    }
}