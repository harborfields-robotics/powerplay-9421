package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CV.AprilTags.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class ConeDetection
{
    public static final int CAMERA_WIDTH = 320, CAMERA_HEIGHT = 240;
    public static final double TAG_SIZE = 0.166; // METERS
    public static final double FEET_PER_METER = 3.28084;
    // PIXELS
    public static final double CAMERA_FX = 100, CAMERA_FY = 100;
    public static final double CAMERA_CX = 200, CAMERA_CY = 120;
    public static final int leftId = 1, middleId = 2, rightId = 3;

    public AprilTagDetectionPipeline pipeline;
    public OpenCvCamera camera;
    public String cameraName;
    public Telemetry telemetry;
    public Hardware hardware;
    public int cameraId;
    public boolean detected;
    public AprilTagDetection detection;

    public ConeDetection(Telemetry telemetry, Hardware hardware, String cameraName)
    {
        this.cameraName = cameraName;
        //this.telemetry = FtcDashboard.getInstance().getTelemetry();
        this.telemetry = telemetry;
        this.hardware = hardware;
    }

    public void openCamera()
    {
        cameraId = hardware.hwMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId",
                        "id",
                        hardware.hwMap.appContext.getPackageName());
        telemetry.addData("Camera ID", cameraId);
        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardware.hwMap.get(WebcamName.class, cameraName), cameraId);
        telemetry.addLine("Got pipeline");
        pipeline = new AprilTagDetectionPipeline(TAG_SIZE, CAMERA_FX, CAMERA_FY, CAMERA_CX, CAMERA_CY);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT); // Set camera dimensions here
                telemetry.addLine("Opened camera");
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("Failed to open camera: error " + errorCode);
            }
        });
    }

    public void closeCamera() { camera.closeCameraDevice(); }

    public ArrayList<AprilTagDetection> detect() { return pipeline.getLatestDetections(); }

    public boolean detectLoop()
    {
        ArrayList<AprilTagDetection> detections = detect();
        if (detections.size() == 0)
            return false;
        detected = true;
        detection = detections.get(detections.size() - 1);

        return true;
    }

    public boolean verboseLoop()
    {
        telemetry.addLine("CV detection loop entered");
        boolean ret = detectLoop();
        if (detected) {
            telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
            tagToTelemetry();
        } else {
            telemetry.addLine("Don't see tag of interest :(");

            if (detection == null) {
                telemetry.addLine("(The tag has never been seen)");
            } else {
                telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                tagToTelemetry();
            }
        }
        telemetry.update();
        return ret;
    }

    public void tagToTelemetry() { tagToTelemetry(detection); }
    public void tagToTelemetry(AprilTagDetection detection) { tagToTelemetry(detection, this.telemetry);}

    public static void tagToTelemetry(AprilTagDetection detection, Telemetry telemetry)
    {
        telemetry.addLine("Tag snapshot:\n");
        if (detection == null) {
            telemetry.addLine("Tag is null!");
            return;
        }

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public int getDirection()
    {
        if (detection == null)
            return middleId;
        return detection.id;
    }
}
