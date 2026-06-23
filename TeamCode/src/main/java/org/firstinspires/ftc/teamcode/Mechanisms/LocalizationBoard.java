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
    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;

    private final double BEARING_OFFSET = 0;
    private final double CALIBRATION_CONSTANT = 40 * sqrt(3.62);

    private double prevX = 0;
    private double prevY = 0;
    private double prevHeading = 0;
    private double prevTagX = 0;
    private double prevTagY = 0;

    private boolean cameraIsInvalid = false;


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
        boolean useCamera = true;
        LLResult detection = null;

        double distance;
        double bearing;

        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();
        double curX = pos.getX(DistanceUnit.INCH);
        double curY = pos.getY(DistanceUnit.INCH);
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
            cameraIsInvalid = false;

            double ta = detection.getTa();
            distance = CALIBRATION_CONSTANT / sqrt(ta);

            double tx = detection.getTx();
            bearing = tx - BEARING_OFFSET;

            prevTagX = distance * Math.cos(bearing);
            prevTagY = distance * Math.sin(bearing);

            //Convert from robot relative to field relative
        }
        // Use odometry for positioning
        else
        {
            double tagX = prevTagX - (curX - prevX);
            double tagY = prevTagY - (curY - prevY);

            distance = Math.sqrt(tagX*tagX + tagY*tagY);
            bearing = Math.atan2(tagX, tagY) - (curHeading - prevHeading);

            prevX = curX;
            prevY = curY;
            prevHeading = curHeading;
            prevTagX = tagX;
            prevTagY = tagY;
        }

        switch (dimension)
        {
            case "range": return distance;
            case "bearing": return bearing;
            default: return 999999;
        }
    }

    //Returns all dimensions in a list
    public double[] GetAprilTag()
    {
        if (limelight == null) {return new double[] {999999, 999999};}
        LLResult detection = limelight.getLatestResult();
        double[] info = {999999, 999999};

        if (detection.isValid())
        {
            double ta = detection.getTa();
            info[0] = (CALIBRATION_CONSTANT / sqrt(ta));
            info[1] = detection.getTx() - BEARING_OFFSET;
        }
        return info;
    }
}