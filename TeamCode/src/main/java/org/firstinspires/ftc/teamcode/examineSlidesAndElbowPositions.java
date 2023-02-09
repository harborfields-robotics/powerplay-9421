package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/* Class objective:
 * - manually test slides and elbow positions using encoder values
 */

@TeleOp
public class examineSlidesAndElbowPositions extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware BigBird = new Hardware(hardwareMap, telemetry);
        BigBird.init();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("slides", BigBird.slides.getCurrentPosition());
            telemetry.addData("elbow", BigBird.elbow1.getPosition());
            telemetry.update();
        }

    }
}
