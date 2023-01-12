package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CV.BarcodeUtil;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
// import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


public class Hardware {
    HardwareMap hwMap = null;
    Drivetrain dt = null;
    DcMotor slides = null;
    Servo elbow1 = null;
    Servo elbow2 = null;
    Grabber grabber = null;
    public static SlidesTarget slidesPosition = null;
    public SampleMecanumDrive drive;
    BarcodeUtil cvUtil;


    private Telemetry telemetry;

    public Hardware(HardwareMap ahwMap, Telemetry telemetry) {
        hwMap = ahwMap;
        this.telemetry = telemetry;
        dt = new Drivetrain(ahwMap, telemetry);
        drive = new SampleMecanumDrive(ahwMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        cvUtil = new BarcodeUtil(hwMap, "Webcam1", telemetry);

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
        /*
        int slides_reset_position = 1000;

        grabber.claw.setPosition(Grabber.claw_closed);
        Thread.sleep(250);
        slides.setTargetPosition(slides_reset_position);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(1);
        Thread.sleep(500);
        elbow1.setPosition(Slides.bottom_position);
        elbow2.setPosition(Slides.bottom_position);
        Thread.sleep(500);
        slides.setTargetPosition(0);
        slides.setPower(.8);

        //cvUtil = new BarcodeUtil(hwMap, "Webcam1", telemetry);
        //colorFront = hardwareMap.get(NormalizedColorSensor.class, "color_front");
        //hwMap = ahwMap;
        //dt = new Drivetrain(hwMap);     // FL, FR, BL, BR
        // intake = new Intake(hwMap);  // intakeFront, intakeBack
        //cvUtil = new BarcodeUtil(hwMap,"Webcam1",telemetry);

        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //vel = new Pose2d(0,0,0);

         */


    }

    public Drivetrain getDrivetrain() {
        return dt;
    }

	// TODO: this class was empty and causing issues with compilation
    public class DriveThread extends Thread {}
}
