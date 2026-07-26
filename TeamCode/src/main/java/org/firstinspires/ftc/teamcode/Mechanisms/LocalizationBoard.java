package org.firstinspires.ftc.teamcode.Mechanisms;

import static java.lang.Math.sqrt;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


public class LocalizationBoard
{
    private final double BEARING_OFFSET = 0;
    private final double CALIBRATION_CONSTANT = 40 * sqrt(3.62);

    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;

    private double prevFieldX = 0;
    private double prevFieldY = 0;
    private double prevHeading = 0;
    private double prevTagX = 0;
    private double prevTagY = 0;

    private boolean useCamera = false;


    public void init(HardwareMap hwMap, int id)
    {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(id);

        pinpoint = hwMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setOffsets(1.22, -3.74, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    public void start()
    {
        limelight.start();
    }

    public void stop()
    {
        limelight.stop();
        limelight.close();
    }

    public double GetAprilTag(String dimension)
    {
        useCamera = true;
        LLResult detection = null;

        double distance;
        double bearing;

        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();
        double curFieldX = pos.getX(DistanceUnit.INCH);
        double curFieldY = pos.getY(DistanceUnit.INCH);
        double curHeading = pos.getHeading(AngleUnit.RADIANS);

        // Determine whether camera sees tag or not
        if (limelight == null) useCamera = false;
        else
        {
            detection = limelight.getLatestResult();
            if (!detection.isValid()) useCamera = false;
        }

        // Use camera for positioning
        if (useCamera)
        {
            double ta = detection.getTa();
            distance = CALIBRATION_CONSTANT / sqrt(ta);

            double tx = detection.getTx();
            bearing = Math.toRadians(tx - BEARING_OFFSET);

            prevTagX = distance * Math.cos(bearing);
            prevTagY = distance * Math.sin(bearing);
        }
        // Use odometry for positioning
        else
        {
            // Calculate displacement in field plane first, then rotate in robot plane
            // so that rotations during movement don't mess up the values
            double dtFieldX = curFieldX - prevFieldX;
            double dtFieldY = curFieldY - prevFieldY;
            double prevHeadingCos = Math.cos(prevHeading);
            double prevHeadingSin = Math.sin(prevHeading);
            double dtRobotX = dtFieldX * prevHeadingCos + dtFieldY * prevHeadingSin;
            double dtRobotY = -dtFieldX * prevHeadingSin + dtFieldY * prevHeadingCos;
            // Tag coordinates in previous plane (aka what they would be if the robot hadn't rotated)
            double prevPlaneTagX = prevTagX - dtRobotX;
            double prevPlaneTagY = prevTagY - dtRobotY;

            double dtHeading = curHeading - prevHeading;
            double dtHeadingCos = Math.cos(dtHeading);
            double dtHeadingSin = Math.sin(dtHeading);
            // Tag coordinates rotated to match this frame's plane
            double tagX = prevPlaneTagX * dtHeadingCos - prevPlaneTagY * dtHeadingSin;
            double tagY = prevPlaneTagX * dtHeadingSin + prevPlaneTagY * dtHeadingCos;

            distance = Math.hypot(tagY, tagX);
            bearing = Math.atan2(tagY, tagX);

            prevTagX = tagX;
            prevTagY = tagY;
        }

        prevFieldX = curFieldX;
        prevFieldY = curFieldY;
        prevHeading = curHeading;

        switch (dimension)
        {
            case "range": return distance;
            case "bearing": return bearing;
            default: return 999999;
        }
    }

    //Returns all dimensions in an array
    public double[] GetAprilTag()
    {
        return new double[] {
                GetAprilTag("range"),
                GetAprilTag("bearing"),
        };
    }

    public boolean isUsingCamera()
    {
        return useCamera;
    }
    public double IMUHeading()
    {
        return Math.toDegrees(prevHeading);
    }

}