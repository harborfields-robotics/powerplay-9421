package org.firstinspires.ftc.teamcode;

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
    //BACK_GROUND(0, 0.0, false, "back ground"),
    //BACK_LOW(0, 0.0, false, "back low"),
    BACK_MIDDLE(0, 0, false, "back middle"),
    BACK_HIGH(600, 0.15, false, "back high"),
    FRONT_GROUND(0, 1.0, true, "front ground"),
    FRONT_LOW(0, 0.65, true, "front low"),
    FRONT_MIDDLE(300, 0.5, true, "front middle"),
    FRONT_HIGH(600, 0.4, true, "front high");

    public final int slides_position;
    public final double elbow_position;
    public final boolean front;
    public final String name;

    SlidesTarget(int s, double e, boolean f, String n) {
        slides_position = s*4; // x4 for gear ratio
        elbow_position  = e;
        front = f;
        name = n;
    }
}
