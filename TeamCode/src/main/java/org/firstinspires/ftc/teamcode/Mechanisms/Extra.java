package org.firstinspires.ftc.teamcode.Mechanisms;

public class Extra
{
    //Every GoBilda 5202 series motor has 28 TPR
    public static double TICKS_PER_REV = 28;


    // Convert 0 - 360 degree range angles to -180 - 180
    public static double WrapAngle(double angle)
    {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    // Ensure angles stay between 0 and 360
    public static double NormalizeAngle(double angle)
    {
        angle %= 2 * Math.PI;
        if (angle < 0) angle += 2 * Math.PI;
        return angle;
    }

    // Calculates actual target angle when fed target is wrapped, but feedback angle is not
    public static double ComputeTargetAngle(double desiredAngle, double currentAngle)
    {
        double error = WrapAngle(desiredAngle - currentAngle);
        return currentAngle + error;
    }

}
