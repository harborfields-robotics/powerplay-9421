package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;

@Autonomous
public class Auto_5CycleLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware BigBird = new Hardware(hardwareMap, telemetry);
        BigBird.init();
        waitForStart();

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

        // 1. Use camera on sleeve to find final parking position

        // 2. Navigate: start [2, 1] --> [3, 1] --> [3, 2] b [3, 3]
        // - move a little forward at the start to avoid knocking over stack
        // - move cone to drop position while driving

        // 3. Turn 90 degrees right (or left if we are still outtaking on the back) and place cone

        // 4. Navigate: [3, 3] --> [3, 1]
        // - reset intake to bottom for next cycle

        // 5. Raise slides slightly so grabber grabs top cone off stack
        // - raise slides to lift top cone up

        // 6. repeat steps 2-5 four more times but starting from [3, 1]
        // - lower grabber slightly each time a cone is grabbed from stack

        // 1/14 Caleb - This height isn't tested but seemed like a good estimate
        for (int grabber_height = 300; grabber_height >= 0; grabber_height -= 100) {
            
        }

        // 7. Navigate: [3, 2] b [3, 3] --> [3, 3] --> signal zone
        // - Left zone (one dot): [3, 3] --> [1, 3]
        // - Middle zone (one dot): [3, 3] --> [2, 3]
        // - Right zone (one dot): no movement needed

    }
}
