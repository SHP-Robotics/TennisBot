package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/*
 * Created by Chun on 17 January 2020 for SHP Tennis.
 */

@Autonomous
//@Disabled

public class RedTwoSkystoneFoundationAutonomous extends BaseRobot {
    private int stage = 0;
    private float distTraveled = 0;
    private int position = 0; //0 = left, 1 = middle, 2 = right

    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = -0.2f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 1.2f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;


    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private OpenCVDetector.StageSwitchingPipeline.Stage stageToRenderToViewport = OpenCVDetector.StageSwitchingPipeline.Stage.detection;
        private OpenCVDetector.StageSwitchingPipeline.Stage[] stages = OpenCVDetector.StageSwitchingPipeline.Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }
    }

    @Override
    public void init() {
        super.init();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);//remove this

        phoneCam.openCameraDevice();//open camera
        //phoneCam.setPipeline(new OpenCVDetector.StageSwitchingPipeline());//different stages
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    @Override
    public void start() {
        super.start();
        if (valLeft == 0)
            position = 0;
        else if (valMid == 0)
            position = 1;
        else if (valRight == 0)
            position = 2;
    }

    @Override
    public void loop() {
        super.loop();

//        telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
//        telemetry.addData("Height", rows);
//        telemetry.addData("Width", cols);

        switch (stage) {
//            case -1:
//                if (auto_turn_imu(-90, angles.firstAngle)) {
//                    reset_drive_encoders();
//                    stage = -2;
//                }
//                break;
            case 0:
                //drive forward to blocks
                if (auto_drive_imu(1, 30,0,angles.firstAngle)) {
                    outtake(0);
                    reset_drive_encoders();
                    stage++;
                } else
                    outtake(1);
                break;
            case 1:
                if (position == 0) {
                    if (auto_mecanum(-1, 25)) {
                        reset_drive_encoders();
                        stage++;
                    }
                } else if (position == 1) {
                    if (auto_mecanum(-1, 15)) {
                        reset_drive_encoders();
                        stage++;
                    }
                } else if (position == 2) {
                    if (auto_mecanum(-1, 5)) {
                        reset_drive_encoders();
                        stage++;
                    }
                }
                break;
            case 2:
                //turn to pick up skystone
                if (auto_turn_imu(25, angles.firstAngle)) {
                    intakeOut(1);
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 3:
                //drive forward to pick up skystone
                if (auto_drive_imu(0.4,25,25,angles.firstAngle)) {
                    intakeOut(0);
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 4:
                //back up to original spot
                if (auto_drive_imu(-1,18, 25, angles.firstAngle)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 5:
                //turn to face away from foundation side
                if (position == 0) {
                    if (auto_turn_imu(85, angles.firstAngle)) {
                        reset_drive_encoders();
                        stage++;
                    }
                } else {
                    if (auto_turn_imu(80, angles.firstAngle)) {
                        reset_drive_encoders();
                        stage++;
                    }
                }
                break;
            case 6:
                //drive backwards to foundation
                if (position == 0) {
                    if (auto_drive_imu(-1, 154,90,angles.firstAngle)) {
                        reset_drive_encoders();
                        stage++;
                    }
                } else if (position == 1) {
                    if (auto_drive_imu(-1, 145,90,angles.firstAngle)) {
                        reset_drive_encoders();
                        stage++;
                    }
                } else if (position == 2) {
                    if (auto_drive_imu(-1, 130,90,angles.firstAngle)) {
                        reset_drive_encoders();
                        stage++;
                    }
                }
                break;
            case 7:
                //turn away from foundation
                if (auto_turn_imu(170, angles.firstAngle)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 8:
                //drive backwards to touch foundation
                if (auto_drive_imu(-0.5, 28,170,angles.firstAngle)) {
                    intakeOut(1);
                    foundationServoLeft.setPosition(ConstantVariables.K_FOUNDATION_LEFT_DOWN);
                    foundationServoRight.setPosition(ConstantVariables.K_FOUNDATION_RIGHT_DOWN);
                    reset_drive_encoders();
                    timer.reset();
                    stage++;
                }
                break;
            case 9:
                //place skystone on foundation
                if (timer.seconds()>0.3) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 10:
                //turn slightly right
                if (auto_turn_imu(165, angles.firstAngle)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 11:
                //drive foundation forward
                if (auto_drive_imu(1, 35,165,angles.firstAngle)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 12:
                //turn to face blocks
                if (auto_turn_imu(80, angles.firstAngle)) {
                    intakeOut(0);
                    foundationServoLeft.setPosition(ConstantVariables.K_FOUNDATION_LEFT_UP);
                    foundationServoRight.setPosition(ConstantVariables.K_FOUNDATION_RIGHT_UP);
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 13:
                //drive towards blocks
                if (position == 0) {
                    if (auto_drive_imu(1, 102, 85, angles.firstAngle)) {
                        reset_drive_encoders();
                        stage++;
                    }
                } else if (position == 1) {
                    if (auto_drive_imu(1, 82, 85, angles.firstAngle)) {
                        reset_drive_encoders();
                        stage++;
                    }
                } else if (position == 2) {
                    if (auto_drive_imu(1, 68, 83, angles.firstAngle)) {
                        reset_drive_encoders();
                        stage++;
                    }
                }
                break;
            case 14:
                //turn to pick up second skystone
                if (auto_turn_imu(25, angles.firstAngle)) {
                    intakeOut(1);
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 15:
                //drive forward to pick up skystone
                if (auto_drive_imu(0.4,35, 25, angles.firstAngle)) {
                    intakeOut(0);
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 16:
                //back up to original spot
                if (auto_drive_imu(-1,25, 25, angles.firstAngle)) {
                    intake(0);
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 17:
                //turn to face away from foundation side
                if (auto_turn_imu(80, angles.firstAngle)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 18:
                //drive backwards to foundation and push into construction site
                if (position == 0) {
                    if (auto_drive_imu(-1, 110,90, angles.firstAngle)) {
                        intakeOut(1);
                        reset_drive_encoders();
                        timer.reset();
                        stage++;
                    }
                } else if (position == 1) {
                    if (auto_drive_imu(-1, 100,85, angles.firstAngle)) {
                        intakeOut(1);
                        reset_drive_encoders();
                        timer.reset();
                        stage++;
                    }
                } else if (position == 2) {
                    if (auto_drive_imu(-1, 89,82, angles.firstAngle)) {
                        intakeOut(1);
                        reset_drive_encoders();
                        timer.reset();
                        stage++;
                    }
                }
                break;
            case 19:
                //place second skystone on foundation
                if (timer.time()>1.5) {
                    intakeOut(0);
                    stage++;
                }
                break;
            case 20:
                //drive to under bridge
                if (auto_drive_imu(1, 50, 90, angles.firstAngle)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            default:
                break;
        }
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                });
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}