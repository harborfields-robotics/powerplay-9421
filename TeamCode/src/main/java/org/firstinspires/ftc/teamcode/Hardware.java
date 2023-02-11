package org.firstinspires.ftc.teamcode;

import android.transition.Slide;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CV.AprilTags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.CV.BarcodeUtil;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Util;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
// import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


public class Hardware {
    public HardwareMap hwMap = null;
    public Drivetrain dt = null;
    DcMotor slides = null;
    Servo elbow1 = null;
    Servo elbow2 = null;
    public Grabber grabber = null;
    public static SlidesTarget slidesPosition = null;
    public SampleMecanumDrive drive;
    public static double slidesAngle = Math.sqrt(2.0)/2;
    BarcodeUtil cvUtil;


    private Telemetry telemetry;

    public static final double elbow_min = 0.515;
    public static final double elbow_max = 1.00;

    public static final int troubleshooterID = 11115;

    public Hardware(HardwareMap ahwMap, Telemetry telemetry) {
        hwMap = ahwMap;
        this.telemetry = telemetry;
        dt = new Drivetrain(ahwMap, telemetry);
        drive = new SampleMecanumDrive(ahwMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("cvUtil is about to initialize", troubleshooterID);
        telemetry.update();
        //cvUtil = new BarcodeUtil(hwMap, "Webcam1", telemetry);

        //dt = new org.firstinspires.ftc.teamcode.Drivetrain(hwMap);

        slides = ahwMap.get(DcMotor.class, "slides"); // expansion hub port 0
        slides.setDirection(DcMotor.Direction.REVERSE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesPosition = SlidesTarget.FRONT_GROUND;

        elbow1 = ahwMap.get(Servo.class, "elbow1"); // control hub port 0
        elbow2 = ahwMap.get(Servo.class, "elbow2"); // control hub port 1
        elbow1.setDirection(Servo.Direction.FORWARD);
        elbow2.setDirection(Servo.Direction.REVERSE);

        grabber = new Grabber(ahwMap);
    }

    //Inits hardware for opmode
    public void init() throws InterruptedException {

        int slides_reset_position = 1000;

        grabber.rightGrabberFace();
        Thread.sleep(500);
        grabber.closeClaw();
        /*
        slides.setTargetPosition(slides_reset_position);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(.2);
        Thread.sleep(1500);
         */
        elbowMove(SlidesTarget.FRONT_GROUND.elbow_position);
        slides.setTargetPosition(0);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (slides.getCurrentPosition() > 0) {
            slides.setPower(.2);
        }
        telemetry.addData("claw: ", grabber.claw.getPosition());
        telemetry.addData("wrist: ", grabber.wrist.getPosition());
        telemetry.addData("slides: ", slides.getCurrentPosition());
        telemetry.addData("elbow: ", elbow1.getPosition());

        //cvUtil = new BarcodeUtil(hwMap, "Webcam1", telemetry);
        //colorFront = hardwareMap.get(NormalizedColorSensor.class, "color_front");
        //hwMap = ahwMap;
        //dt = new Drivetrain(hwMap);     // FL, FR, BL, BR
        // intake = new Intake(hwMap);  // intakeFront, intakeBack
        //cvUtil = new BarcodeUtil(hwMap,"Webcam1",telemetry);

        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //vel = new Pose2d(0,0,0);


    }

    public void stop() throws InterruptedException {

    }

    public void elbowMove(double position)
    {
        position = Util.clamp(position, 0, 1);
        elbow1.setPosition(position);
        elbow2.setPosition(position);
    }

    public void flipElbowAndWrist(boolean raiseClaw) throws InterruptedException {
        if (raiseClaw) {
            this.elbowMove(Hardware.elbow_min);
            Thread.sleep(300);
            this.grabber.flipGrabberFace();
        } else {
            if (slides.getCurrentPosition() > 700) {
                Thread.sleep(750);
            }
            this.elbowMove(Hardware.elbow_max);
            this.grabber.rightGrabberFace();
        }

    }

    public void flipElbowAndWrist(boolean raiseClaw, double newElbowPosition) throws InterruptedException {
        this.elbowMove(newElbowPosition);
        if (raiseClaw) {
            Thread.sleep(300);
            this.grabber.flipGrabberFace();
        }
        else {
            this.grabber.rightGrabberFace();
        }

    }

    public void autoDeposit(boolean stack, int slidesPosition, double elbowMaxPosition, double elbowFinalPosition) throws InterruptedException {
        flipElbowAndWrist(true, elbowMaxPosition); // flip elbow up to target position
        slides.setTargetPosition(slidesPosition); // extend slides to target position
        slides.setPower(1);
        if (stack) {
            dt.driveBackward(0.15, 0.5);
            dt.turnRight(20, 0.4);
            Thread.sleep(1500);
        }
        else {
            Thread.sleep(2000);
        }
        elbowMove(elbowMaxPosition - .055);
        Thread.sleep(500);
        grabber.openClaw(); // drop cone
        slides.setTargetPosition(0); // extend slides to target position
        if (stack) {
            dt.turnLeft(20, 0.4);
            Thread.sleep(500);
            flipElbowAndWrist(false, elbowFinalPosition);
            dt.driveForward(0.15, 0.3);
            Thread.sleep(500);
        }
        else {
            Thread.sleep(600);
            flipElbowAndWrist(false, elbowFinalPosition);
        }
    }

    public Drivetrain getDrivetrain() {
        return dt;
    }
}
