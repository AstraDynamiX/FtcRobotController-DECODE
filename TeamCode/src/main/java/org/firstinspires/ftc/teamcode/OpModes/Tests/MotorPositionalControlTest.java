package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Mechanisms.Extra.ComputeTargetAngle;
import static org.firstinspires.ftc.teamcode.Mechanisms.Extra.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.Mechanisms.Extra.WrapAngle;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@TeleOp(group = "tests")
public class MotorPositionalControlTest extends OpMode
{
    private MotorEx motor;

    private double currentAngle = 0;
    private double currentAngleTicks = 0;
    private double targetAngle = 0;
    private double targetAngleTicks = 0;


    @Override
    public void init()
    {
        motor = new MotorEx(hardwareMap, "motor", Motor.GoBILDA.RPM_435);
        motor.setRunMode(MotorEx.RunMode.PositionControl);
        motor.setCachingTolerance(0.001);
        motor.setPositionCoefficient(0.15);
        motor.setPositionTolerance(5);

        motor.resetEncoder();
    }

    @Override
    public void loop()
    {
        // Control motor using joystick, where joystick points = where motor points
        // (release => motor keeps position)
        if (Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y) > 0.5)
        {
            double desiredAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
            currentAngleTicks = motor.getCurrentPosition();
            currentAngle = currentAngleTicks / TICKS_PER_REV * 2 * Math.PI;

            targetAngle = ComputeTargetAngle(desiredAngle, currentAngle);
            targetAngleTicks = targetAngle / (2 * Math.PI) * TICKS_PER_REV;
        }

        motor.setTargetPosition((int) targetAngleTicks);

        telemetry.addData("MOTOR ANGLE", Math.toDegrees(currentAngle));
        telemetry.addData("MOTOR TICKS", currentAngleTicks);
        telemetry.addData("TARGET ANGLE:", targetAngle);
    }

}

