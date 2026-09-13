package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.pedropathing.math.MathFunctions.findNormalizingScaling;

import com.pedropathing.Drivetrain;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import java.util.Arrays;
import java.util.List;

/**
 * This is the CoaxialSwerve class, a child class of Drivetrain. This class takes in inputs Vectors for driving, heading
 * correction, and translational/centripetal correction and returns an array with wheel powers.
 * @author Remus Serediuc - 24964 AstraDynamiX
 * @version 1.0, 9/12/2026
 */
public class CoaxialSwerve extends Drivetrain
{
    public CoaxialSwerveConstants constants;
    private final DcMotorEx leftFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightFront;
    private final DcMotorEx rightRear;
    private final CRServoEx leftFrontServo;
    private final CRServoEx leftRearServo;
    private final CRServoEx rightFrontServo;
    private final CRServoEx rightRearServo;
    private final AbsoluteAnalogEncoder leftFrontServoEncoder;
    private final AbsoluteAnalogEncoder leftRearServoEncoder;
    private final AbsoluteAnalogEncoder rightFrontServoEncoder;
    private final AbsoluteAnalogEncoder rightRearServoEncoder;
    private final List<DcMotorEx> motors;
    private final List<CRServoEx> servos;
    private final List<AbsoluteAnalogEncoder> encoders;
    private final VoltageSensor voltageSensor;
    private double motorCachingThreshold;
    private boolean useBrakeModeInTeleOp;
    private double staticFrictionCoefficient;

    /**
     * This creates a new CoaxialSwerve, which takes in various movement vectors and outputs
     * the wheel drive powers necessary to move in the intended direction, given the true movement
     * vector for the front left mecanum wheel.
     *
     * @param hardwareMap      this is the HardwareMap object that contains the motors and other hardware
     * @param coaxialSwerveConstants this is the CoaxialSwerveConstants object that contains the names of the motors and directions etc.
     */
    public CoaxialSwerve(HardwareMap hardwareMap, CoaxialSwerveConstants coaxialSwerveConstants)
    {
        constants = coaxialSwerveConstants;

        this.maxPowerScaling = coaxialSwerveConstants.maxPower;
        this.motorCachingThreshold = coaxialSwerveConstants.motorCachingThreshold;
        this.useBrakeModeInTeleOp = coaxialSwerveConstants.useBrakeModeInTeleOp;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        leftFront = hardwareMap.get(DcMotorEx.class, coaxialSwerveConstants.leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, coaxialSwerveConstants.leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, coaxialSwerveConstants.rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, coaxialSwerveConstants.rightFrontMotorName);

        leftFrontServo = hardwareMap.get(CRServoEx.class, coaxialSwerveConstants.leftFrontServoName);
        leftRearServo = hardwareMap.get(CRServoEx.class, coaxialSwerveConstants.leftRearServoName);
        rightRearServo = hardwareMap.get(CRServoEx.class, coaxialSwerveConstants.rightRearServoName);
        rightFrontServo = hardwareMap.get(CRServoEx.class, coaxialSwerveConstants.rightFrontServoName);

        leftFrontServoEncoder = hardwareMap.get(AbsoluteAnalogEncoder.class, coaxialSwerveConstants.leftFrontServoEncoderName);
        leftRearServoEncoder = hardwareMap.get(AbsoluteAnalogEncoder.class, coaxialSwerveConstants.leftRearServoEncoderName);
        rightRearServoEncoder = hardwareMap.get(AbsoluteAnalogEncoder.class, coaxialSwerveConstants.rightRearServoEncoderName);
        rightFrontServoEncoder = hardwareMap.get(AbsoluteAnalogEncoder.class, coaxialSwerveConstants.rightFrontServoEncoderName);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);
        servos = Arrays.asList(leftFrontServo, leftRearServo, rightFrontServo, rightRearServo);
        encoders = Arrays.asList(leftFrontServoEncoder, leftRearServoEncoder, rightFrontServoEncoder, rightRearServoEncoder);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (int i = 0; i < 4; i++)
        {
            servos.get(i).setAbsoluteEncoder(encoders.get(i));
            servos.get(i).setRunMode(CRServoEx.RunMode.OptimizedPositionalControl);
        }

        setMotorsToFloat();
        breakFollowing();
    }

    public void updateConstants()
    {
        leftFront.setDirection(constants.leftFrontMotorDirection);
        leftRear.setDirection(constants.leftRearMotorDirection);
        rightFront.setDirection(constants.rightFrontMotorDirection);
        rightRear.setDirection(constants.rightRearMotorDirection);
        this.motorCachingThreshold = constants.motorCachingThreshold;
        this.useBrakeModeInTeleOp = constants.useBrakeModeInTeleOp;
        this.voltageCompensation = constants.useVoltageCompensation;
        this.nominalVoltage = constants.nominalVoltage;
        this.staticFrictionCoefficient = constants.staticFrictionCoefficient;
    }

    /**
     * This takes in vectors for corrective power, heading power, and pathing power and outputs
     * an Array of four doubles, one for each wheel's motor power.
     * <p>
     * IMPORTANT NOTE: all vector inputs are clamped between 0 and 1 inclusive in magnitude.
     *
     * @param correctivePower this Vector includes the centrifugal force scaling Vector as well as a
     *                        translational power Vector to correct onto the Bezier curve the Follower
     *                        is following.
     * @param headingPower    this Vector points in the direction of the robot's current heading, and
     *                        the magnitude tells the robot how much it should turn and in which
     *                        direction.
     * @param pathingPower    this Vector points in the direction the robot needs to go to continue along
     *                        the Path.
     * @param robotHeading    this is the current heading of the robot, which is used to calculate how
     *                        much power to allocate to each wheel.
     * @return this returns an Array of doubles with a length of 4, which contains the wheel powers.
     */
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading)
    {
        // clamps down the magnitudes of the input vectors
        if (correctivePower.getMagnitude() > maxPowerScaling)
            correctivePower.setMagnitude(maxPowerScaling);
        if (headingPower.getMagnitude() > maxPowerScaling)
            headingPower.setMagnitude(maxPowerScaling);
        if (pathingPower.getMagnitude() > maxPowerScaling)
            pathingPower.setMagnitude(maxPowerScaling);

        // the powers for the wheel vectors
        double[] wheelPowers = new double[4];
        double[] wheelAngles = new double[4];

        Vector[] modulePositions = {
                new Vector(constants.trackWidth / 2, constants.wheelBase / 2),
                new Vector(constants.trackWidth / 2, -constants.wheelBase / 2),
                new Vector(-constants.trackWidth / 2, constants.wheelBase / 2),
                new Vector(-constants.trackWidth / 2, -constants.wheelBase / 2)
        };

        // recover signed turn rate from headingPower (parallel to heading by contract)
        double headingUnitX = Math.cos(robotHeading);
        double headingUnitY = Math.sin(robotHeading);
        double turnPower = headingPower.getXComponent() * headingUnitX + headingPower.getYComponent() * headingUnitY;

        // rotate module positions into field frame, get rotation contribution per module
        Vector[] rotationContribution = new Vector[4];
        for (int i = 0; i < 4; i++) {
            Vector posFieldFrame = modulePositions[i].copy();
            posFieldFrame.rotateVector(robotHeading);
            rotationContribution[i] = new Vector(-posFieldFrame.getYComponent(), posFieldFrame.getXComponent()).times(turnPower);
        }

        // stage 1: correctivePower + rotation, scale turnPower down if any module overflows
        Vector[] noPathing = new Vector[4];
        for (int i = 0; i < 4; i++) noPathing[i] = correctivePower.plus(rotationContribution[i]);

        boolean overflow = false;
        for (Vector v : noPathing) if (v.getMagnitude() > maxPowerScaling) overflow = true;

        if (overflow)
        {
            double headingScalingFactor = Double.MAX_VALUE;
            for (int i = 0; i < 4; i++)
                headingScalingFactor = Math.min(headingScalingFactor,
                        findNormalizingScaling(correctivePower, rotationContribution[i], maxPowerScaling));
            turnPower *= headingScalingFactor;
            for (int i = 0; i < 4; i++) {
                rotationContribution[i] = rotationContribution[i].times(headingScalingFactor);
                noPathing[i] = correctivePower.plus(rotationContribution[i]);
            }
        }

        // stage 2: add pathingPower, scale it down if any module overflows
        Vector[] target = new Vector[4];
        for (int i = 0; i < 4; i++) target[i] = noPathing[i].plus(pathingPower);

        overflow = false;
        for (Vector v : target) if (v.getMagnitude() > maxPowerScaling) overflow = true;

        if (overflow) {
            double pathingScalingFactor = Double.MAX_VALUE;
            for (int i = 0; i < 4; i++)
                pathingScalingFactor = Math.min(pathingScalingFactor,
                        findNormalizingScaling(noPathing[i], pathingPower, maxPowerScaling));
            for (int i = 0; i < 4; i++)
                target[i] = noPathing[i].plus(pathingPower.times(pathingScalingFactor));
        }

        // convert each module's field-frame target into robot-frame speed + angle
        for (int i = 0; i < 4; i++) {
            Vector robotFrame = target[i].copy();
            robotFrame.rotateVector(-robotHeading);
            wheelPowers[i] = robotFrame.getMagnitude();
            wheelAngles[i] = Math.atan2(robotFrame.getYComponent(), robotFrame.getXComponent());
        }

        // Final compensation and scaledown
        if (voltageCompensation) {
            double voltageNormalized = getVoltageNormalized();
            for (int i = 0; i < 4; i++) wheelPowers[i] *= voltageNormalized;
        }

        double wheelPowerMax = 0;
        for (double p : wheelPowers) wheelPowerMax = Math.max(wheelPowerMax, Math.abs(p));
        if (wheelPowerMax > maxPowerScaling) {
            for (int i = 0; i < 4; i++) wheelPowers[i] = (wheelPowers[i] / wheelPowerMax) * maxPowerScaling;
        }

        double[] wheelPowersAndAngles = new double[8];
        for (int i = 0; i < 4; i++)
        {
            wheelPowersAndAngles[i] = wheelPowers[i];
            wheelPowersAndAngles[i+4] = wheelAngles[i];
        }
        return wheelPowersAndAngles;
    }

    /**
     * This sets the motors to the zero power behavior of brake.
     */
    private void setMotorsToBrake() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * This sets the motors to the zero power behavior of float.
     */
    private void setMotorsToFloat() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void breakFollowing() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
        setMotorsToFloat();
    }

    public void runDrive(double[] drivePowersAndAngles)
    {
        for (int i = 0; i < 4; i++)
        {
            if (Math.abs(motors.get(i).getPower() - drivePowersAndAngles[i]) > motorCachingThreshold) {
                motors.get(i).setPower(drivePowersAndAngles[i]);
                servos.get(i).set(drivePowersAndAngles[i+4]);
            }
        }
    }

    @Override
    public void startTeleopDrive() {
        if (useBrakeModeInTeleOp) {
            setMotorsToBrake();
        }
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        if (brakeMode) {
            setMotorsToBrake();
        } else {
            setMotorsToFloat();
        }
    }

    public void getAndRunDrivePowers(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        runDrive(calculateDrive(correctivePower, headingPower, pathingPower, robotHeading));
    }

    public double xVelocity() {
        return constants.xVelocity;
    }

    public double yVelocity() {
        return constants.yVelocity;
    }

    public void setXVelocity(double xMovement) { constants.setXVelocity(xMovement); }
    public void setYVelocity(double yMovement) { constants.setYVelocity(yMovement); }

    public double getStaticFrictionCoefficient() {
        return staticFrictionCoefficient;
    }

    @Override
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    private double getVoltageNormalized() {
        double voltage = getVoltage();
        return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient));
    }

    public String debugString() {
        return "Mecanum{" +
                " leftFront=" + leftFront +
                ", leftRear=" + leftRear +
                ", rightFront=" + rightFront +
                ", rightRear=" + rightRear +
                ", motors=" + motors +
                ", motorCachingThreshold=" + motorCachingThreshold +
                ", useBrakeModeInTeleOp=" + useBrakeModeInTeleOp +
                '}';
    }

    public List<DcMotorEx> getMotors() {
        return motors;
    }
}
