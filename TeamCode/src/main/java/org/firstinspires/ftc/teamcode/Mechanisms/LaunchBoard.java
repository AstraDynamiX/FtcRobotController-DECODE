package org.firstinspires.ftc.teamcode.Mechanisms;

import static org.firstinspires.ftc.teamcode.Mechanisms.Extra.WrapAngle;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import java.util.ArrayList;
import java.util.List;

@Configurable
public class LaunchBoard
{
    private final double FLYWHEEL_RADIUS = 1.89; // in

    public static double FLYWHEEL_KP = 11.2;
    public static double FLYWHEEL_KI = 14;
    public static double FLYWHEEL_MULTIPLIER = 1.17;


    LocalizationBoard LocalizationBoard = new LocalizationBoard();
    ElapsedTime stopperTimer = new ElapsedTime();

    private DcMotor intake;
    private MotorEx leftFlywheel;
    private MotorEx rightFlywheel;

    private Servo leftLed;
    private Servo rightLed;

    private LaunchState launchState = LaunchState.IDLE;

    private double distance = 0;
    private double launchAngle = 0;
    private double smallestLaunchSpeed = 999999;
    private double flywheelInput = 0;
    private double manualAdjusterAngle = 0.35;
    private double adjusterAngle = 0;


    public void init(HardwareMap hwMap, boolean redAlliance)
    {
        LocalizationBoard.init(hwMap, (redAlliance) ? 7 : 8);
        //Initialize motors and servos
        intake = hwMap.get(DcMotor.class, "intake");
        leftFlywheel = initFlywheelMotor(hwMap, true, "leftFlywheel");
        rightFlywheel = initFlywheelMotor(hwMap, false, "rightFlywheel");

        leftLed = hwMap.get(Servo.class, "leftLed");
        rightLed = hwMap.get(Servo.class, "rightLed");

        Idle();
    }

    private MotorEx initFlywheelMotor(HardwareMap hwMap, boolean inverted, String name)
    {
        MotorEx motor;
        motor = new MotorEx(hwMap, name, Motor.GoBILDA.BARE);
        motor.setRunMode(MotorEx.RunMode.VelocityControl);
        motor.setVeloCoefficients(FLYWHEEL_KP, FLYWHEEL_KI, 0);
        motor.setInverted(inverted);
        motor.setCachingTolerance(0.01);
        return motor;
    }

    public void start() {LocalizationBoard.start();}
    public void stop() {LocalizationBoard.stop();}


    private enum FlywheelMode
    {
        IDLE,
        AUTOMATIC,
        MANUAL,
    }

    public void LaunchAdjustment(FlywheelMode mode, double manualFlywheelSpeed)
    {
        if (smallestLaunchSpeed == 0) flywheelInput = 0;
        else
        {
            double launchSpeedTps = smallestLaunchSpeed * leftFlywheel.getCPR() / (2 * 3.1415 * FLYWHEEL_RADIUS);
            flywheelInput = (FLYWHEEL_MULTIPLIER * launchSpeedTps);
        }

        leftFlywheel.setVelocity(flywheelInput);
        rightFlywheel.setVelocity(flywheelInput);

        switch (mode)
        {

            case MANUAL:

                leftFlywheel.setVeloCoefficients(FLYWHEEL_KP, FLYWHEEL_KI, 0);
                rightFlywheel.setVeloCoefficients(FLYWHEEL_KP, FLYWHEEL_KI, 0);

                smallestLaunchSpeed = manualFlywheelSpeed;
                adjusterAngle = manualAdjusterAngle;

                // Turn on LED's when flywheel is near the needed speed
                double flywheelGradient = leftFlywheel.getVelocity() / flywheelInput;
                if (flywheelGradient < 1.05 && flywheelGradient > 0.95)
                {
                    leftLed.setPosition(0.7);
                    rightLed.setPosition(0.7);
                }
                else
                {
                    leftLed.setPosition(0);
                    rightLed.setPosition(0);
                }

                break;

            case IDLE:
            default:

                // Remove PID when slowing down because slowing down quickly is unnecessary
                // and it drains battery, since motors brake instead of floating
                leftFlywheel.setVeloCoefficients(0, 0, 0);
                rightFlywheel.setVeloCoefficients(0, 0, 0);
                smallestLaunchSpeed = 0;

                leftLed.setPosition(0);
                rightLed.setPosition(0);
                break;
        }
    }

    public void LaunchAdjustment(FlywheelMode mode) {LaunchAdjustment(mode, 1);}

    // Getters
    public double getDistance() {return distance;}
    public double getLaunchAngle() {return Math.toDegrees(launchAngle);}
    public double getFlywheelInput() {return flywheelInput;}
    public double getFlywheelSpeed() {return rightFlywheel.getVelocity();}

    // ------ Intake state machine ------

    enum LaunchState
    {
        IDLE,
        INTAKE,
        OUTTAKE,
        REV,
        SHOOT,
    }

    public void Intake()
    {
        launchState = LaunchState.INTAKE;
    }

    public void Outtake() {launchState = LaunchState.OUTTAKE;}

    public void Rev()
    {
        launchState = LaunchState.REV;
        smallestLaunchSpeed = 999999; // Start revving preemptively
    }

    public void Shoot()
    {
        launchState = LaunchState.SHOOT;
        stopperTimer.reset();
    }

    public void Idle() {launchState = LaunchState.IDLE;}


    public void UpdateLaunch(boolean camAdjustment, double manualFlywheelSpeed)
    {
        switch (launchState)
        {
            case INTAKE:

                intake.setPower(-0.95);
                LaunchAdjustment(FlywheelMode.IDLE);

                break;

            case OUTTAKE:

                intake.setPower(0.95);
                LaunchAdjustment(FlywheelMode.IDLE);

                break;

            case REV:

                intake.setPower(0.05);

                if (camAdjustment)
                {LaunchAdjustment(FlywheelMode.AUTOMATIC);}
                else {LaunchAdjustment(FlywheelMode.MANUAL, manualFlywheelSpeed);}

                break;

            case SHOOT:

                // Wait for stopper to fully lift before starting transfer
                if (stopperTimer.milliseconds() < 150) return;

                intake.setPower(-0.9);

                if (camAdjustment)
                {LaunchAdjustment(FlywheelMode.AUTOMATIC);}
                else {LaunchAdjustment(FlywheelMode.MANUAL, manualFlywheelSpeed);}

                break;

            case IDLE:
            default:

                intake.setPower(0);
                LaunchAdjustment(FlywheelMode.IDLE);
                break;
        }
    }

}