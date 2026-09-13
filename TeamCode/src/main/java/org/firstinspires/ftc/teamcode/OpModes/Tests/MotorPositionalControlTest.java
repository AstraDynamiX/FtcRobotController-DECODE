package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Mechanisms.Extra.ComputeTargetAngle;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Mechanisms.LocalizationBoard;

@TeleOp(group = "tests")
public class MotorPositionalControlTest extends OpMode
{
    private double TICKS_PER_REV;
    private final double GEAR_RATIO = 3;
    private final double LIMIT = Math.toRadians(87);

    private MotorEx motor;

    LocalizationBoard LocalizationBoard = new LocalizationBoard();

    private double desiredAngle = 0;
    private double bearing = 0;
    private double currentAngle = 0;
    private double currentAngleTicks = 0;
    private double targetAngle = 0;
    private double targetAngleTicks = 0;

    private boolean joystickControl = false;
    private boolean motorControl = false;


    @Override
    public void init()
    {
        LocalizationBoard.init(hardwareMap, 7);

        motor = new MotorEx(hardwareMap, "turret", Motor.GoBILDA.RPM_435);
        motor.setRunMode(MotorEx.RunMode.PositionControl);
        motor.setCachingTolerance(0.001);
        motor.setPositionCoefficient(0.1);
        motor.setPositionTolerance(5);

        motor.resetEncoder();
        TICKS_PER_REV = motor.getCPR();
    }

    @Override
    public void start()
    {
        LocalizationBoard.start();
    }


    @Override
    public void loop()
    {
        if (gamepad1.aWasPressed()) {joystickControl = !joystickControl;}
        if (gamepad1.bWasPressed()) {motorControl = !motorControl;}

        currentAngleTicks = motor.getCurrentPosition() / GEAR_RATIO;
        currentAngle = currentAngleTicks / TICKS_PER_REV * 2 * Math.PI;
        LocalizationBoard.SetCameraOrientation(currentAngle);

        // Control motor using joystick, where joystick points = where motor points
        // (release => motor keeps position)
        if (joystickControl && Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y) > 0.5)
        {desiredAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI/2;}
        else if (!joystickControl)
        {
            bearing = LocalizationBoard.GetAprilTag("bearing");
            desiredAngle = -bearing;
        }

        targetAngle = ComputeTargetAngle(desiredAngle, currentAngle);
        targetAngleTicks = targetAngle / (2 * Math.PI) * TICKS_PER_REV * GEAR_RATIO;

        motor.setTargetPosition((int) targetAngleTicks);
        if (targetAngle > -LIMIT && targetAngle < LIMIT && motorControl) {motor.set(0.12);}
        else {motor.set(0);}

        telemetry.addData("AUTO CONTROL", !joystickControl);
        telemetry.addData("MOTOR CONTROL", motorControl);
        telemetry.addData("", "");
        if (!joystickControl)
        {telemetry.addData("USING CAMERA", LocalizationBoard.IsUsingCamera());}
        telemetry.addData("MOTOR ANGLE", Math.toDegrees(currentAngle));
        telemetry.addData("MOTOR TICKS", currentAngleTicks);
        telemetry.addData("BEARING", Math.toDegrees(bearing));
        telemetry.addData("IMU HEADING", LocalizationBoard.GetIMUHeading());
        telemetry.addData("TARGET ANGLE:", Math.toDegrees(targetAngle));
    }

    @Override
    public void stop() {LocalizationBoard.stop();}

}

