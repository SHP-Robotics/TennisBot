package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/*
 * Created by Chun on 22 November 2019 for SHP Tennis.
 */

@Autonomous
@Disabled

public class BasicAutonomous extends BaseRobot {
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
                //drive forward
                if (auto_drive(1, 12)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 1:
                //drive backward
                if (auto_drive(-1, 12)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 2:
                //horizontal right
                if (auto_mecanum(1, 12)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 3:
                //horizontal left
                if (auto_mecanum(-1, 12)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 4:
                //turn right
                if (auto_turn(1, 360)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 5:
                //turn left
                if (auto_turn(-1, 360)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            default:
                break;
        }
    }
}