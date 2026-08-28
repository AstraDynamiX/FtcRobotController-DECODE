package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.LocalizationBoard;


@TeleOp(group = "tests")
public class AprilTagTrackingTest extends OpMode
{
    private final LocalizationBoard LocalizationBoard = new LocalizationBoard();


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
        double[] tagDimensions = LocalizationBoard.GetAprilTag();

        telemetry.addData("USING CAMERA", LocalizationBoard.IsUsingCamera());
        telemetry.addData("RANGE", tagDimensions[0]);
        telemetry.addData("BEARING", Math.toDegrees(tagDimensions[1]));
        telemetry.addData("IMU HEADING", LocalizationBoard.GetIMUHeading());
    }

    @Override
    public void stop()
    {
        LocalizationBoard.stop();
    }

}