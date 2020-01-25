package org.firstinspires.ftc.teamcode;

/*
 * Created by Chun on 22 November 2019 for SHP Tennis.
 */

public class ConstantVariables {
    public static final double K_FOUNDATION_LEFT_UP = 0.0;
    public static final double K_FOUNDATION_LEFT_DOWN = 0.37;
    public static final double K_FOUNDATION_RIGHT_UP = 1.0;
    public static final double K_FOUNDATION_RIGHT_DOWN = 0.65;
    //wheel: 15 teeth, motor: 20 teeth
    public static final int K_CPR_DRIVE = 560; //1120
    public static final double K_DRIVE_WHEEL_DIA = 4;
    public static final double K_DRIVE_WIDTH = 16.5;
    public static final double K_DRIVE_HEIGHT = 11.5;
    public static final double K_DRIVE_DIA = Math.sqrt(Math.pow(K_DRIVE_WIDTH,2) + Math.pow(K_DRIVE_HEIGHT,2));


    public static final double K_DRIVE_WHEEL_CIRCUMFERENCE = K_DRIVE_WHEEL_DIA * Math.PI;
    public static final double K_CPIN_DRIVE = K_CPR_DRIVE / K_DRIVE_WHEEL_CIRCUMFERENCE;

    public static final double K_TURN_CIRCUMFERENCE = K_DRIVE_DIA * Math.PI;
    public static final double K_CPTURN_DRIVE = K_CPIN_DRIVE * K_TURN_CIRCUMFERENCE;
    public static final double K_CPDEG_DRIVE = K_CPTURN_DRIVE / 360;

    public static final double K_DRIVE_ERROR_P = 500; // higher = less sensitive
}