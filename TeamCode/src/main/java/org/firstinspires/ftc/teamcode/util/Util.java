package org.firstinspires.ftc.teamcode.util;

import java.lang.Math;
public class Util
{
    public static double clamp(double x, double min, double max)
    {
        return Math.min(max, Math.max(x, min));
    }
}
