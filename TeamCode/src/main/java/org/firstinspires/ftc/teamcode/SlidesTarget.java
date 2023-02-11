package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.util.Util;

/*
 * PROBLEM:  elbow currently rotates upwards over the top of the robot instead of underneath the pivot point
 * SOLUTION: maybe swap servo directions so they rotate the opposite ways\
 *
 * PROBLEM:  elbow limit around the back of the robot is too high (doesn't go low enough for ground junction)
 * SOLUTION: use servo programmers to re-set servo limits
 *
 * Also, these servo values have not been tested, they are just estimations of where the elbow should go with the current limits (12/21/22)
 */

public enum SlidesTarget {

    //BACK_GROUND(0, Hardware.elbow_min, false, "back ground"),
    BACK_HIGH(2050, Hardware.elbow_min, false, "back high"),
    BACK_MIDDLE(400, Hardware.elbow_min, false, "back middle"),
    BACK_LOW(0, 0.42, false, "back low"),
    FRONT_GROUND(0, Hardware.elbow_max, true, "front ground"); // 1.0
  //  FRONT_LOW(500, Hardware.elbow_max, true, "front low"), // 0.65
    //FRONT_MIDDLE(1000, Hardware.elbow_max, true, "front middle"), // 0.5
//    FRONT_HIGH(1600, Hardware.elbow_max, true, "front high"); // 0.4

    public final int slides_position;
    public final double elbow_position;

    public final boolean front;
    public final String name;

    SlidesTarget(int s, double e, boolean f, String n) {
        slides_position = s; // x4 for gear ratio
        elbow_position  = Util.clamp(e, 0, 1);
        front = f;
        name = n;
    }
}
