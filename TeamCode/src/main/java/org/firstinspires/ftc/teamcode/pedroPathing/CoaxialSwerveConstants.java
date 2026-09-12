package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class CoaxialSwerveConstants
{
    /** The Forward Velocity of the Robot - Different for each robot
     *  Default Value: 81.34056 */
    public  double xVelocity = 81.34056;

    /** The Lateral Velocity of the Robot - Different for each robot
     *  Default Value: 65.43028 */
    public  double yVelocity = 65.43028;

    public  double maxPower = 1;
    public  String leftFrontMotorName = "leftFront";
    public  String leftRearMotorName = "leftRear";
    public  String rightFrontMotorName = "rightFront";
    public  String rightRearMotorName = "rightRear";
    public  String leftFrontServoName = "leftFront";
    public  String leftRearServoName = "leftRear";
    public  String rightFrontServoName = "rightFront";
    public  String rightRearServoName = "rightRear";
    public  String leftFrontServoEncoderName = "leftFront";
    public  String leftRearServoEncoderName = "leftRear";
    public  String rightFrontServoEncoderName = "rightFront";
    public  String rightRearServoEncoderName = "rightRear";
    public  DcMotorSimple.Direction leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
    public  DcMotorSimple.Direction leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
    public  DcMotorSimple.Direction rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    public  DcMotorSimple.Direction rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
    public  double wheelBase = 15;
    public  double trackWidth = 15;
    public  double motorCachingThreshold = 0.01;
    public  boolean useBrakeModeInTeleOp = false;
    public  boolean useVoltageCompensation = false;
    public  double nominalVoltage = 12.0;
    public  double staticFrictionCoefficient = 0.1;

    public CoaxialSwerveConstants() {
        defaults();
    }

    public CoaxialSwerveConstants xVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
        return this;
    }

    public CoaxialSwerveConstants yVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
        return this;
    }

    public CoaxialSwerveConstants maxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    public CoaxialSwerveConstants leftFrontMotorName(String leftFrontMotorName) {
        this.leftFrontMotorName = leftFrontMotorName;
        return this;
    }

    public CoaxialSwerveConstants leftRearMotorName(String leftRearMotorName) {
        this.leftRearMotorName = leftRearMotorName;
        return this;
    }

    public CoaxialSwerveConstants rightFrontMotorName(String rightFrontMotorName) {
        this.rightFrontMotorName = rightFrontMotorName;
        return this;
    }

    public CoaxialSwerveConstants rightRearMotorName(String rightRearMotorName) {
        this.rightRearMotorName = rightRearMotorName;
        return this;
    }

    public CoaxialSwerveConstants leftFrontServoName(String leftFrontServoName) {
        this.leftFrontServoName = leftFrontServoName;
        return this;
    }

    public CoaxialSwerveConstants leftRearServoName(String leftRearServoName) {
        this.leftRearServoName = leftRearServoName;
        return this;
    }

    public CoaxialSwerveConstants rightFrontServoName(String rightFrontServoName) {
        this.rightFrontServoName = rightFrontServoName;
        return this;
    }

    public CoaxialSwerveConstants rightRearServoName(String rightRearServoName) {
        this.rightRearServoName = rightRearServoName;
        return this;
    }

    public CoaxialSwerveConstants leftFrontServoEncoderName(String leftFrontServoEncoderName) {
        this.leftFrontServoEncoderName = leftFrontServoEncoderName;
        return this;
    }

    public CoaxialSwerveConstants leftRearServoEncoderName(String leftRearServoEncoderName) {
        this.leftRearServoEncoderName = leftRearServoEncoderName;
        return this;
    }

    public CoaxialSwerveConstants rightFrontServoEncoderName(String rightFrontServoEncoderName) {
        this.rightFrontServoEncoderName = rightFrontServoEncoderName;
        return this;
    }

    public CoaxialSwerveConstants rightRearServoEncoderName(String rightRearServoEncoderName) {
        this.rightRearServoEncoderName = rightRearServoEncoderName;
        return this;
    }

    public CoaxialSwerveConstants leftFrontMotorDirection(DcMotorSimple.Direction leftFrontMotorDirection) {
        this.leftFrontMotorDirection = leftFrontMotorDirection;
        return this;
    }

    public CoaxialSwerveConstants leftRearMotorDirection(DcMotorSimple.Direction leftRearMotorDirection) {
        this.leftRearMotorDirection = leftRearMotorDirection;
        return this;
    }

    public CoaxialSwerveConstants rightFrontMotorDirection(DcMotorSimple.Direction rightFrontMotorDirection) {
        this.rightFrontMotorDirection = rightFrontMotorDirection;
        return this;
    }

    public CoaxialSwerveConstants rightRearMotorDirection(DcMotorSimple.Direction rightRearMotorDirection) {
        this.rightRearMotorDirection = rightRearMotorDirection;
        return this;
    }

    public CoaxialSwerveConstants wheelBase(double wheelBase) {
        this.wheelBase = wheelBase;
        return this;
    }

    public CoaxialSwerveConstants trackWidth(double trackWidth) {
        this.trackWidth = trackWidth;
        return this;
    }

    public CoaxialSwerveConstants motorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
        return this;
    }

    public CoaxialSwerveConstants useBrakeModeInTeleOp(boolean useBrakeModeInTeleOp) {
        this.useBrakeModeInTeleOp = useBrakeModeInTeleOp;
        return this;
    }

    public CoaxialSwerveConstants useVoltageCompensation(boolean useVoltageCompensation) {
        this.useVoltageCompensation = useVoltageCompensation;
        return this;
    }

    public CoaxialSwerveConstants nominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public CoaxialSwerveConstants staticFrictionCoefficient(double staticFrictionCoefficient) {
        this.staticFrictionCoefficient = staticFrictionCoefficient;
        return this;
    }

    public double getXVelocity() {
        return xVelocity;
    }

    public void setXVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
    }

    public double getYVelocity() {
        return yVelocity;
    }

    public void setYVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
    }

    public double getMaxPower() {
        return maxPower;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public String getLeftFrontMotorName() {
        return leftFrontMotorName;
    }

    public void setLeftFrontMotorName(String leftFrontMotorName) {
        this.leftFrontMotorName = leftFrontMotorName;
    }

    public String getLeftRearMotorName() {
        return leftRearMotorName;
    }

    public void setLeftRearMotorName(String leftRearMotorName) {
        this.leftRearMotorName = leftRearMotorName;
    }

    public String getRightFrontMotorName() {
        return rightFrontMotorName;
    }

    public void setRightFrontMotorName(String rightFrontMotorName) {
        this.rightFrontMotorName = rightFrontMotorName;
    }

    public String getRightRearMotorName() {
        return rightRearMotorName;
    }

    public void setRightRearMotorName(String rightRearMotorName) {
        this.rightRearMotorName = rightRearMotorName;
    }

    public String getLeftFrontServoName() {
        return leftFrontServoName;
    }

    public void setLeftFrontServoName(String leftFrontServoName) {
        this.leftFrontServoName = leftFrontServoName;
    }

    public String getLeftRearServoName() {
        return leftRearServoName;
    }

    public void setLeftRearServoName(String leftRearServoName) {
        this.leftRearServoName = leftRearServoName;
    }

    public String getRightFrontServoName() {
        return rightFrontServoName;
    }

    public void setRightFrontServoName(String rightFrontServoName) {
        this.rightFrontServoName = rightFrontServoName;
    }

    public String getRightRearServoName() {
        return rightRearServoName;
    }

    public void setRightRearServoName(String rightRearServoName) {
        this.rightRearServoName = rightRearServoName;
    }

    public String getLeftFrontServoEncoderName() {
        return leftFrontServoEncoderName;
    }

    public void setLeftFrontServoEncoderName(String leftFrontServoEncoderName) {
        this.leftFrontServoEncoderName = leftFrontServoEncoderName;
    }

    public String getLeftRearServoEncoderName() {
        return leftRearServoEncoderName;
    }

    public void setLeftRearServoEncoderName(String leftRearServoEncoderName) {
        this.leftRearServoEncoderName = leftRearServoEncoderName;
    }

    public String getRightFrontServoEncoderName() {
        return rightFrontServoEncoderName;
    }

    public void setRightFrontServoEncoderName(String rightFrontServoEncoderName) {
        this.rightFrontServoEncoderName = rightFrontServoEncoderName;
    }

    public String getRightRearServoEncoderName() {
        return rightRearServoEncoderName;
    }

    public void setRightRearServoEncoderName(String rightRearServoEncoderName) {
        this.rightRearServoEncoderName = rightRearServoEncoderName;
    }

    public DcMotorSimple.Direction getLeftFrontMotorDirection() {
        return leftFrontMotorDirection;
    }

    public void setLeftFrontMotorDirection(DcMotorSimple.Direction leftFrontMotorDirection) {
        this.leftFrontMotorDirection = leftFrontMotorDirection;
    }

    public DcMotorSimple.Direction getLeftRearMotorDirection() {
        return leftRearMotorDirection;
    }

    public void setLeftRearMotorDirection(DcMotorSimple.Direction leftRearMotorDirection) {
        this.leftRearMotorDirection = leftRearMotorDirection;
    }

    public DcMotorSimple.Direction getRightFrontMotorDirection() {
        return rightFrontMotorDirection;
    }

    public void setRightFrontMotorDirection(DcMotorSimple.Direction rightFrontMotorDirection) {
        this.rightFrontMotorDirection = rightFrontMotorDirection;
    }

    public DcMotorSimple.Direction getRightRearMotorDirection() {
        return rightRearMotorDirection;
    }

    public void setRightRearMotorDirection(DcMotorSimple.Direction rightRearMotorDirection) {
        this.rightRearMotorDirection = rightRearMotorDirection;
    }

    public double getWheelBase() {
        return wheelBase;
    }

    public void setWheelBase(double wheelBase) {
        this.wheelBase = wheelBase;
    }

    public double getTrackWidth() {
        return trackWidth;
    }

    public void setTrackWidth(double trackWidth) {
        this.trackWidth = trackWidth;
    }

    public double getMotorCachingThreshold() {
        return motorCachingThreshold;
    }

    public void setMotorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
    }

    public boolean isUseBrakeModeInTeleOp() {
        return useBrakeModeInTeleOp;
    }

    public void setUseBrakeModeInTeleOp(boolean useBrakeModeInTeleOp) {
        this.useBrakeModeInTeleOp = useBrakeModeInTeleOp;
    }

    /**
     * This method sets the default values for the CoaxialSwerveConstants class.
     * It is called in the constructor of the CoaxialSwerveConstants class.
     */
    public void defaults() {
        xVelocity = 81.34056;
        yVelocity = 65.43028;
        maxPower = 1;
        leftFrontMotorName = "leftFront";
        leftRearMotorName = "leftRear";
        rightFrontMotorName = "rightFront";
        rightRearMotorName = "rightRear";
        leftFrontServoName = "leftFront";
        leftRearServoName = "leftRear";
        rightFrontServoName = "rightFront";
        rightRearServoName = "rightRear";
        leftFrontServoEncoderName = "leftFront";
        leftRearServoEncoderName = "leftRear";
        rightFrontServoEncoderName = "rightFront";
        rightRearServoEncoderName = "rightRear";
        leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        wheelBase = 15;
        trackWidth = 15;
        motorCachingThreshold = 0.01;
        useBrakeModeInTeleOp = false;
        useVoltageCompensation = false;
        nominalVoltage = 12.0;
        staticFrictionCoefficient = 0.1;
    }
}
