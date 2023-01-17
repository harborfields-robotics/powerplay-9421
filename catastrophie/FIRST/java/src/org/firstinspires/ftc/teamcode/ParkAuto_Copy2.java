package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="Park Auto", group="Linear Opmode")

public class ParkAuto_Copy2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor bl = null;
    private DcMotor br = null;
    private DcMotor fl = null; 
    private DcMotor fr = null; 
    private DcMotor shooter1 = null; 
    private DcMotor shooter2 = null; 
    private Servo wobbleGrabber = null; 
    private Servo ringBlocker = null; 
    private Servo shooterAngle = null; 
    private Servo ringPusher = null; 

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        bl  = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl  = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        wobbleGrabber = hardwareMap.get(Servo.class, "wobbleGrabber");
        ringPusher = hardwareMap.get(Servo.class, "ringPusher"); 
        ringBlocker = hardwareMap.get(Servo.class, "ringBlocker"); 
        shooterAngle = hardwareMap.get(Servo.class, "shooterAngle"); 
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        
        wobbleGrabber.setPosition(0.3); 
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        sleep(22000); 
        bl.setPower(-0.4); 
        fl.setPower(-0.4); 
        br.setPower(-0.4); 
        fr.setPower(-0.4); 
        sleep(3100); 
        bl.setPower(0); 
        fl.setPower(0); 
        br.setPower(0); 
        fr.setPower(0); 
        sleep(500);
        shooterAngle.setPosition(0); 
        sleep(300); 
        wobbleGrabber.setPosition(0.5); 
        sleep(1000);
        bl.setPower(0); 
        fl.setPower(0); 
        br.setPower(0); 
        fr.setPower(0); 
    }
}
