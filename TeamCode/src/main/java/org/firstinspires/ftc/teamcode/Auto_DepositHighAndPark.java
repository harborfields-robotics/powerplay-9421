package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CV.AprilTags.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class Auto_DepositHighAndPark extends LinearOpMode {

    static int postDepositWaitTime = 1000; // time between dropping cone and moving robot again (milliseconds)
    static double turnPower = 0.5;
    static double drivePower = 0.4;
    static double strafePower = 0.45;

    static int troubleshooterID = 11115;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 100;
    double fy = 100;
    double cx = 200;
    double cy = 120;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //Tag IDs of signal sleeve
    int leftID = 1;
    int middleID = 2;
    int rightID = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("BigBird is about to initialize", troubleshooterID);
        telemetry.update();
        Hardware BigBird = new Hardware(hardwareMap, telemetry);
        telemetry.addData("BigBird is initialized", troubleshooterID);
        telemetry.update();

        CycleConeThread cct = new CycleConeThread(BigBird,true, SlidesTarget.FRONT_GROUND.slides_position);

        BigBird.init();

        // CV junk starts here
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Log.d("Auto", "Got MonitorViewId: " + cameraMonitorViewId);
        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        Log.d("Auto", "Instantiated camera factory");
        telemetry.addLine("Got pipeline");
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        Log.d("Auto", "Instantiated pipeline");

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT); // Set camera dimensions here
                Log.d("Camera Open", "Opened Camera");
            }

            @Override
            public void onError(int errorCode)
            {
                Log.e("Camera Open", "Failed to open camera: " + errorCode);
            }
        });
        telemetry.addData("camera initialization complete", troubleshooterID);
        telemetry.update();

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("CV detection loop entered", troubleshooterID);
            Log.d("Auto", "Entered CV detection loop");
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == leftID || tag.id == middleID || tag.id == rightID) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
            if (isStarted() || isStopRequested()) {
                Log.d("Auto Init", "Started or stop requested.");
                break;
            }
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        telemetry.addData("CV loop exited", 11115);

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful
         * (CV junk ends here)
         */

        /*
         * Tiles are notated like on the coordinate grid.
         * - bottom left tile is [1, 1]
         * - bottom right tile is [6, 1]
         * - top left tile is [1, 6]
         * - top right tile is [6, 6]
         *
         * [x, y] - means center of tile
         * [x1, y1] b [x2, y2] means between two tiles
         */

        //Navigate: [2, 1] --> [2, 3] --> [2, 3] b [3, 3]
        cct.start();
        // [2, 1] --> [2, 3]
        BigBird.dt.driveForward(2.5, drivePower);
        Thread.sleep(100);
        // turn around
        BigBird.dt.turnLeft(180, turnPower);
        Thread.sleep(250);
        // [2, 3] --> [2, 3] b [3, 3]
        BigBird.dt.strafeLeft(.6, strafePower);
        Thread.sleep(200);
        BigBird.dt.driveBackward(.1, drivePower);
        // deposit cone
        Thread.sleep(2000);
        cct.waiting = false;
        Thread.sleep(postDepositWaitTime);
        BigBird.dt.driveForward(.1, drivePower);
        Thread.sleep(200);

        // Navigate: [2, 3] b [3, 3] --> signal zone
        // - Left zone (one dot): [2, 3] b [3, 3] --> [1, 3]
        if (tagOfInterest.id == leftID) {
            BigBird.dt.strafeRight(1.6, strafePower);
        }
        // - Middle zone (one dot): [2, 3] b [3, 3] --> [2, 3]
        else if (tagOfInterest.id == middleID) {
            BigBird.dt.strafeRight(0.6, strafePower);
        }
        // - Right zone (one dot): [2, 3] b [3, 3] --> [3, 3]
        else if (tagOfInterest.id == rightID) {
            BigBird.dt.strafeLeft(0.6, strafePower);
        }

    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    // unused as of 1/25/23
    public static class DriveThread extends Thread {

        Hardware BigBird = null;
        Drivetrain dt = null;
        DcMotor FL = null;
        DcMotor BL = null;
        DcMotor FR = null;
        DcMotor BR = null;

        public DriveThread(Hardware bb) {
            BigBird = bb;
            dt = bb.dt;
            FL = bb.dt.FL;
            BL = bb.dt.BL;
            FR = bb.dt.FR;
            BR = bb.dt.BR;
        }

        @Override
        public void run() {

        }

    }

    public static class CycleConeThread extends Thread {

        Hardware BigBird = null;
        SlidesTarget slidesPosition = SlidesTarget.FRONT_GROUND;
        volatile boolean waiting;
        int coneDropDelay = 0; // time between slides reaching target position and cone being dropped (milliseconds)
        static double slidesPowerUp = 0.8;
        static double slidesPowerDown = 0.6;
        int slidesResetPosition; // final slides position of the cycle, set in initialization

        public CycleConeThread(Hardware b, boolean w, int srp) {
            BigBird = b;
            waiting = w;
            slidesResetPosition = srp;
        }

        @Override
        public void run() {
            // move slides up, flip elbow and wrist
            try {BigBird.flipElbowAndWrist(true);} catch (InterruptedException ignored) {}
            BigBird.slides.setTargetPosition(SlidesTarget.BACK_HIGH.slides_position);
            while (BigBird.slides.getCurrentPosition() < BigBird.slides.getTargetPosition()) {
                BigBird.slides.setPower(slidesPowerUp);
            }
            BigBird.slides.setPower(0);

            while (waiting) {} // wait for cue from drive thread to drop cone
            BigBird.grabber.openClaw(); // drop cone
            try {Thread.sleep(postDepositWaitTime);} catch (InterruptedException ignored) {} // wait a bit for the cone to drop before proceeding

            // go back down and flip back to front
            try {BigBird.flipElbowAndWrist(false);} catch (InterruptedException ignored) {}
            BigBird.slides.setTargetPosition(0);
            while (BigBird.slides.getCurrentPosition() > 0) {
                BigBird.slides.setPower(slidesPowerDown);
            }
            BigBird.slides.setPower(0);
        }

        public void changeSlidesPosition() {
            boolean increases = BigBird.slides.getCurrentPosition() < slidesPosition.slides_position;
            BigBird.slides.setPower(0);
        }
    }
}
