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
    static double turnPower = 0.25;
    static double drivePower = 0.25;
    static double strafePower = 0.6;
    static double topConePosition = 0.93;
    static int junctionOffsetAngle = 15;
    static int pauseTime = 100;

    static final int GLUTEN_FREE = 11115;

    ConeDetection coneDetection;

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware BigBird = new Hardware(hardwareMap, telemetry);
        BigBird.init();

        // CV junk starts here
        coneDetection = new ConeDetection(telemetry, BigBird, "Webcam1");
        coneDetection.openCamera();

        telemetry.addLine("Camera initialization complete");
        telemetry.update();

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            coneDetection.verboseLoop();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        telemetry.addLine("CV loop exited");

        BigBird.dt.strafeRight(0.2, strafePower);
        // TODO: fix strafing so it does not turn the robot
        BigBird.dt.turnLeft(DebugConstants.STRAFE_CORRECTION, turnPower); // correct for turn during strafe

        BigBird.dt.driveBackward(1.65, drivePower); // drive to deposit position
        Thread.sleep(pauseTime);

        BigBird.dt.turnRight(30, turnPower); // turn back towards high junction
        Thread.sleep(pauseTime);

        BigBird.autoDeposit(false, SlidesTarget.BACK_HIGH.slides_position - 250, SlidesTarget.BACK_HIGH.elbow_position, Hardware.elbow_max);
        Thread.sleep(pauseTime);

        BigBird.dt.turnLeft(30, turnPower); // turn back to cardinal directions
        Thread.sleep(pauseTime);

        BigBird.dt.driveForward(0.3, drivePower); // drive to 2nd row to prepare parking
        Thread.sleep(pauseTime);

        switch (coneDetection.getDirection()) {
            case ConeDetection.leftId:
                BigBird.dt.strafeRight(1.1, strafePower);
                break;
            case ConeDetection.middleId:
                break; // park in place
            case ConeDetection.rightId:
                BigBird.dt.strafeLeft(1.1, strafePower);
                break;
            default:
                telemetry.addData("Invalid direction", coneDetection.getDirection());
        }

        /* Update the telemetry */
        coneDetection.tagToTelemetry();
        telemetry.update();
    }
}