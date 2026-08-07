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
    private final LocalizationBoard LocalizationBoard = new LocalizationBoard();

    private double[] tagDimensions;


    @Override
    public void init()
    {
        LocalizationBoard.init(hardwareMap, 7);
    }

    @Override
    public void start()
    {
        LocalizationBoard.start();
    }

    @Override
    public void loop()
    {
        tagDimensions = LocalizationBoard.GetAprilTag();

        telemetry.addData("USING CAMERA", LocalizationBoard.isUsingCamera());
        telemetry.addData("RANGE", tagDimensions[0]);
        telemetry.addData("BEARING", Math.toDegrees(tagDimensions[1]));
        telemetry.addData("IMU HEADING", LocalizationBoard.IMUHeading());
    }

    @Override
    public void stop()
    {
        LocalizationBoard.stop();
    }
}