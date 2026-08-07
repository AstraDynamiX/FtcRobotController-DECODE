package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.LaunchBoard;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

class AutoStep
{
    PathChain path;
    Runnable action;
    double speed;
    double delay;
    double actionDelay;

    public AutoStep(PathChain path, Runnable action, double speed, double delay, double actionDelay)
    {
        this.path = path;
        this.action = action;
        this.speed = speed;
        this.delay = delay;
        this.actionDelay = actionDelay;
    }
}

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous")
public class Autonomous extends OpMode
{
    private final double SHOOT_DELAY = 900; // Wait for turret to adjust
    private final double SHOOT_TIME = 2600; // Wait for shooting before starting path
    private final double INTAKE_SPEED = 0.5;

    private final double FIELD_LENGTH = 144; // in pedroPathing coordinates

    LaunchBoard LaunchBoard = new LaunchBoard();

    private Follower follower;

    ElapsedTime opModeTimer = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();


    //Blue close
    private final Pose startClose = new Pose(25.083, 131.242, Math.toRadians(143.5));
    private final Pose shootClose = new Pose(54, 90, Math.toRadians(135));
    private final Pose intake1StartClose = new Pose(52.297, 87.270, Math.toRadians(180));
    private final Pose intake1EndClose = new Pose(19.299, 87.270, Math.toRadians(180));
    private final Pose intake2StartClose = new Pose(50.081, 63.703, Math.toRadians(180));
    private final Pose intake2EndClose = new Pose(17.702, 63.594, Math.toRadians(180));
    private final Pose intake3StartClose = new Pose(55.865, 39.243, Math.toRadians(180));
    private final Pose intake3EndClose = new Pose(20.541, 39.243, Math.toRadians(180));
    private final Pose endClose = new Pose(24.919, 79.978, Math.toRadians(-135));

    //Blue far
    private final Pose startFar = new Pose(56,8, Math.toRadians(90));
    private final Pose shootFar = new Pose(57.730,13.514, Math.toRadians(110));
    private final Pose intake1StartFar = new Pose(55.865, 37.243, Math.toRadians(180));
    private final Pose intake1EndFar = new Pose(20.541, 37.243, Math.toRadians(180));
    private final Pose intake2StartFar = new Pose(20.919, 44.838, Math.toRadians(-95));
    private final Pose intake2EndFar = new Pose(18, 17.459, Math.toRadians(-95));
    private final Pose endFar = new Pose(38.108, 26.297, Math.toRadians(-150));


    private Pose currentPose;

    List<AutoStep> steps = new ArrayList<>();
    int stepIndex = 0;
    private boolean stepStarted = false;
    private boolean pathStarted = false;
    private boolean actionStarted = false;

    //Trajectory selection booleans
    boolean redAlliance = false;
    boolean closeTrajectory = true;
    boolean confirmed = false;
    boolean turretAdjustment = false;


    @Override
    public void init()
    {
        opModeTimer.reset();
        follower = Constants.createFollower(hardwareMap);

        steps.clear();
        stepIndex = 0;
        stepStarted = false;
        pathStarted = false;
        actionStarted = false;
    }

    @Override
    public void init_loop()
    {
        if (confirmed)
        {
            telemetry.addData("CONFIRMED", "");
            telemetry.addData("ALLIANCE", (redAlliance) ? "red" : "blue");
            telemetry.addData("TRAJECTORY", (closeTrajectory) ? "close" : "far");
        }
        else
        {
            if (gamepad1.bWasPressed())
            {redAlliance = !redAlliance;}

            if (gamepad1.xWasPressed())
            {closeTrajectory = !closeTrajectory;}

            if (gamepad1.a)
            {
                confirmed = true;
                LaunchBoard.init(hardwareMap, redAlliance);
            }

            telemetry.addData("", "() - change alliance, [] - change trajectory, X - confirm");
            telemetry.addData("ALLIANCE", (redAlliance) ? "red" : "blue");
            telemetry.addData("TRAJECTORY", (closeTrajectory) ? "close" : "far");

        }
    }

    @Override
    public void start()
    {
        opModeTimer.reset();
        LaunchBoard.start();

        if (closeTrajectory)
        {
            StartRoutine(startClose);

            AddRoutineStep(shootClose, () -> LaunchBoard.Rev(), 1,  0, 0);
            AddRoutineStep(intake1StartClose, () -> LaunchBoard.Shoot(), 1, SHOOT_TIME, SHOOT_DELAY);
            AddRoutineStep(intake1EndClose, () -> LaunchBoard.Intake(), INTAKE_SPEED, 0, 0);
            AddRoutineStep(shootClose, () -> LaunchBoard.Rev(), 1, 0, 0);
            AddRoutineStep(intake2StartClose, () -> LaunchBoard.Shoot(), 1, SHOOT_TIME, SHOOT_DELAY);
            AddRoutineStep(intake2EndClose, () -> LaunchBoard.Intake(), INTAKE_SPEED, 0, 0);
            AddRoutineStep(shootClose, () -> LaunchBoard.Rev(), 1, 0, 0);
            AddRoutineStep(startClose, () -> LaunchBoard.Shoot(), 1, SHOOT_TIME, SHOOT_DELAY);
            /*AddRoutineStep(intake3StartClose, () -> LaunchBoard.Shoot(), 1, SHOOT_TIME, SHOOT_DELAY);
            AddRoutineStep( intake3EndClose, () -> LaunchBoard.Intake(), INTAKE_SPEED, 0, 0);
            AddRoutineStep(shootClose, () -> LaunchBoard.Rev(), 1, 0, 0);
            AddRoutineStep(endClose, () -> LaunchBoard.Shoot(), 1, SHOOT_TIME, SHOOT_DELAY);*/
        }
        else
        {
            StartRoutine(startFar);

            AddRoutineStep(shootFar, () -> LaunchBoard.Rev(), 1, 0, 0);
            AddRoutineStep(intake1StartFar, () -> LaunchBoard.Shoot(), 1, SHOOT_TIME, SHOOT_DELAY);
            AddRoutineStep(intake1EndFar, () -> LaunchBoard.Intake(), INTAKE_SPEED, 0, 0);
            AddRoutineStep(shootFar, () -> LaunchBoard.Rev(), 1, 0, 0);
            AddRoutineStep(intake2StartFar, () -> LaunchBoard.Shoot(), 1, SHOOT_TIME, SHOOT_DELAY);
            AddRoutineStep(intake2EndFar, () -> LaunchBoard.Intake(), INTAKE_SPEED, 0, 0);
            AddRoutineStep(shootFar, () -> LaunchBoard.Rev(), 0.8, 0, 0);
            AddRoutineStep(endFar, () -> LaunchBoard.Shoot(), 1, SHOOT_TIME, SHOOT_DELAY);
        }
    }

    @Override
    public void loop()
    {
        follower.update();
        RunScheduler();
        LaunchBoard.UpdateLaunch(true, 1);

        if (turretAdjustment) LaunchBoard.TurretMovement();
        else LaunchBoard.TurretLockPosition(0);

        if (opModeTimer.seconds() > 30) {requestOpModeStop();}
    }

    @Override
    public void stop()
    {
        LaunchBoard.stop();
    }


    private Pose MirrorPose(Pose pose, boolean vertical)
    {
        if (vertical)
        {
            return new Pose(
                    FIELD_LENGTH - pose.getX(),
                    pose.getY(),
                    NormalizeAngle(-pose.getHeading())
            );
        }
        else
        {
            return new Pose(
                    pose.getX(),
                    FIELD_LENGTH - pose.getY(),
                    NormalizeAngle(Math.PI - pose.getHeading())
            );
        }
    }


    private void AddStep(Pose start, Pose end, Runnable action, double speed, double delay, double actionDelay)
    {
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();

        steps.add(new AutoStep(path, action, speed, delay, actionDelay));
    }


    private Pose AlliancePose(Pose bluePose)
    {
        return redAlliance ? bluePose : MirrorPose(bluePose, true);
    }

    /** "Routine" means path chain that follows this structure:
     *  - the end pose of a step is the start pose of the next step
     *  - only the poses from one alliance (side of the field) is used
     */
    private void StartRoutine(Pose start)
    {
        currentPose = AlliancePose(start);
        follower.setPose(currentPose);
    }

    private void AddRoutineStep(Pose end, Runnable action, double speed, double delay, double actionDelay)
    {
        Pose nextPose = AlliancePose(end);
        AddStep(currentPose, nextPose, action, speed, delay, actionDelay);
        currentPose = nextPose;
    }


    private void RunScheduler() {

        if (stepIndex >= steps.size())
        {
            LaunchBoard.Idle();
            return;
        }
        AutoStep step = steps.get(stepIndex);

        //Reset timer after each new step
        if (!stepStarted)
        {
            timer.reset();
            stepStarted = true;
        }

        if (!pathStarted && timer.milliseconds() > step.delay)
        {
            pathStarted = true;
            follower.setMaxPower(step.speed);
            follower.followPath(step.path, true);
        }

        //Run action after specified time
        if (!actionStarted && timer.milliseconds() > step.actionDelay)
        {
            if (step.action != null) step.action.run();
            actionStarted = true;
        }

        //Run next path after the previous one is finished
        if (!follower.isBusy() && pathStarted && actionStarted)
        {
            stepIndex++;
            stepStarted = false;
            pathStarted = false;
            actionStarted = false;
        }
    }


    private static double NormalizeAngle(double angle)
    {
        angle %= 2 * Math.PI;
        if (angle < 0) angle += 2 * Math.PI;
        return angle;
    }
}
