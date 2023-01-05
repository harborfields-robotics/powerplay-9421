package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
// import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


public class Hardware {
    HardwareMap hwMap = null;
    Drivetrain dt = null;
    DcMotor slides = null;
    Servo elbow1 = null;
    Servo elbow2 = null;
    SlidesPosition slides_position = null;
    public SampleMecanumDrive drive;

    private Telemetry telemetry;

    public Hardware(HardwareMap ahwMap, Telemetry telemetry) {
        hwMap = ahwMap;
        this.telemetry = telemetry;
        dt = new Drivetrain(ahwMap);
        drive = new SampleMecanumDrive(ahwMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //dt = new org.firstinspires.ftc.teamcode.Drivetrain(hwMap);

        slides = hwMap.get(DcMotor.class, "slides"); // expansion hub port 0
        slides.setDirection(DcMotor.Direction.FORWARD);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elbow1 = ahwMap.get(Servo.class, "elbow1"); // control hub port 0
        elbow2 = ahwMap.get(Servo.class, "elbow2"); // control hub port 1
        elbow1.setDirection(Servo.Direction.FORWARD);
        elbow2.setDirection(Servo.Direction.REVERSE);

        slides_position = SlidesPosition.FRONT_GROUND;
    }

    //Inits hardware for opmode
    public void init() throws InterruptedException {
        int slides_reset_position = 300;

        slides.setTargetPosition(slides_reset_position);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(.4);
        Thread.sleep(1500);
        elbow1.setPosition(1);
        elbow2.setPosition(1);
        Thread.sleep(1500);
        slides.setTargetPosition(0);
        slides.setPower(.2);

        //cvUtil = new BarcodeUtil(hwMap, "Webcam1", telemetry);
        //colorFront = hardwareMap.get(NormalizedColorSensor.class, "color_front");
        //hwMap = ahwMap;
        //dt = new Drivetrain(hwMap);     // FL, FR, BL, BR
        // intake = new Intake(hwMap);  // intakeFront, intakeBack
        //cvUtil = new BarcodeUtil(hwMap,"Webcam1",telemetry);

        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //vel = new Pose2d(0,0,0);


    }

    public Drivetrain getDrivetrain() {
        return dt;
    }
}
