package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

@TeleOp(group = "tests basic")
public class BasicMotorTest extends OpMode
{
    private final String[] MOTOR_NAMES = {"leftFlywheel", "rightFlywheel", "turret"}; // Names used in Control Hub config
    private final Motor.GoBILDA[] MOTOR_TYPES = {Motor.GoBILDA.BARE, Motor.GoBILDA.BARE, Motor.GoBILDA.RPM_435};

    private MotorEx[] motors = new MotorEx[MOTOR_NAMES.length];
    // Automatically updates values based on current inputs
    private final Supplier<Float>[] inputs = new Supplier[] {
            () -> -gamepad1.left_stick_y,
            () -> -gamepad1.right_stick_y,
            () -> gamepad1.left_trigger,
            () -> gamepad1.right_trigger
    };

    @Override
    public void init()
    {
        for (int i = 0; i < MOTOR_NAMES.length; i++)
        {
            // Check if name exists in config
            if (hardwareMap.get(MOTOR_NAMES[i]) != null)
            {
                motors[i] = initMotor(hardwareMap, MOTOR_NAMES[i], MOTOR_TYPES[i]);
                motors[i].resetEncoder();
            }
        }
    }

    private MotorEx initMotor(HardwareMap hwMap, String name, Motor.GoBILDA type)
    {
        MotorEx motor;
        motor = new MotorEx(hwMap, name, type);
        motor.setRunMode(MotorEx.RunMode.RawPower);
        return motor;
    }

    @Override
    public void loop()
    {
        for (int i = 0; i < motors.length; i++)
        {
            if (motors[i] != null)
            {
                motors[i].set(inputs[i].get() * 0.9);
                telemetry.addData(MOTOR_NAMES[i], motors[i].getCurrentPosition());
            }
        }
    }

}
