package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Mechanisms.Extra.ComputeTargetAngle;
import static org.firstinspires.ftc.teamcode.Mechanisms.Extra.WrapAngle;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(group = "tests")
public class SwerveTest extends OpMode
{
    private CRServoEx crServo;
    private AbsoluteAnalogEncoder servoAngle;
    private MotorEx motor;

    private double desiredAngle = 0;
    private double currentAngle = 0;
    private double targetAngle = 0;
    private boolean inverted = false;


    @Override
    public void init()
    {
        servoAngle = new AbsoluteAnalogEncoder(hardwareMap, "lfAngle", 3.3, AngleUnit.RADIANS);

        crServo = new CRServoEx(hardwareMap, "lfPod", servoAngle, CRServoEx.RunMode.OptimizedPositionalControl);
        crServo.setPIDF(new PIDFCoefficients(0.2, 0.0, 0.1, 0.0001));
        crServo.setCachingTolerance(0.01);

        motor = new MotorEx(hardwareMap, "lfWheel");
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setCachingTolerance(0.01);
    }

    @Override
    public void loop()
    {
        currentAngle = servoAngle.getCurrentPosition();

        // Control angle of wheel using joystick, where joystick points = where wheel points
        // (release => wheel keeps position)
        if (Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y) > 0.5)
        {
            desiredAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 2;
            targetAngle = ComputeTargetAngle(desiredAngle, currentAngle);

            // Angle optimization - if error is larger than 90 degrees switch direction of motor
            // and turn to the diametrically opposite angle
            double angleError = WrapAngle(targetAngle - currentAngle);

            if (Math.abs(angleError) > Math.PI / 2)
            {
                targetAngle = WrapAngle(targetAngle + Math.PI);
                inverted = true;
            }
            else
            {inverted = false;}

            motor.setInverted(inverted);
        }

        crServo.set(targetAngle);
        
        motor.set(-gamepad1.right_stick_y * 0.9);

        telemetry.addData("SERVO ANGLE", currentAngle);
        telemetry.addData("TARGET ANGLE:", targetAngle);
        telemetry.addData("INVERTED", inverted);
    }

}
