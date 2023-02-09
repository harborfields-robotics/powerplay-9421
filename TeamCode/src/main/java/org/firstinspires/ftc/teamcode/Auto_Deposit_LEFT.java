package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CV.AprilTags.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class Auto_Deposit_LEFT extends LinearOpMode {

    static int postDepositWaitTime = 200; // time between dropping cone and moving robot again (milliseconds)
    static double turnPower = 0.35;
    static double drivePower = 0.25;
    static double strafePower = 0.6;
    static double topConePosition = 0.93;
    static int junctionOffsetAngle = 15;
    static int pauseTime = 100;

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

        Hardware BigBird = new Hardware(hardwareMap, telemetry);
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
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); // Set camera dimensions here
                Log.d("Camera Open", "Opened Camera");
            }

            @Override
            public void onError(int errorCode) {
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

        BigBird.dt.driveBackward(1.8, drivePower); // drive to deposit position
        Thread.sleep(pauseTime);

        BigBird.dt.turnLeft(30, turnPower); // turn back towards high junction
        Thread.sleep(pauseTime);

        BigBird.autoDeposit(SlidesTarget.BACK_HIGH.slides_position, SlidesTarget.BACK_HIGH.elbow_position, Hardware.elbow_max);
        Thread.sleep(pauseTime);

        BigBird.dt.turnRight(30, turnPower); // turn back to cardinal directions
        Thread.sleep(pauseTime);

        BigBird.dt.driveForward(0.5, drivePower); // drive to 2nd row to prepare parking
        Thread.sleep(pauseTime);

        // park in zone
        if (tagOfInterest.id == leftID) {
            BigBird.dt.strafeRight(1.1, strafePower);
        }
        else if (tagOfInterest == null || tagOfInterest.id == middleID) {
            // go to middle zone
        }
        else if (tagOfInterest.id == rightID) {
            BigBird.dt.strafeLeft(1.1, strafePower);
        }

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
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
}
