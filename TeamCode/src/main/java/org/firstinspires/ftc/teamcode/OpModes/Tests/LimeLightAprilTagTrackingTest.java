package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static java.lang.Math.sqrt;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.Mechanisms.LocalizationBoard;


@TeleOp(group = "tests")
public class LimeLightAprilTagTrackingTest extends OpMode
{
    private final LocalizationBoard localizationBoard = new LocalizationBoard();

    private double[] tagDimensions;


    @Override
    public void init()
    {
        localizationBoard.init(hardwareMap, 7);
    }

    @Override
    public void start()
    {
        localizationBoard.start();
    }

    @Override
    public void loop()
    {
        tagDimensions = localizationBoard.GetAprilTag();

        telemetry.addData("USING CAMERA", localizationBoard.isUsingCamera());
        telemetry.addData("RANGE", tagDimensions[0]);
        telemetry.addData("BEARING", Math.toDegrees(tagDimensions[1]));
        telemetry.addData("IMU HEADING", localizationBoard.IMUHeading());
    }

    @Override
    public void stop()
    {
        localizationBoard.stop();
    }
}