package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.legacy.Temp_Hardware;

@Autonomous(name="NovaAuto", group="Yay...")
public class NovaAuto extends OpMode {
    Temp_Hardware robot = new Temp_Hardware();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(48, 9, Math.toRadians(90));
    private final Pose scorePose = new Pose(84,108, Math.toRadians(45));
    private final Pose pickUp1Pose = new Pose(132, 108, Math.toRadians(0));
    private final Pose pickUp2Pose = new Pose(132, 84, Math.toRadians(0));
    private final Pose pickUp3Pose = new Pose(132, 60, Math.toRadians(0));
    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    public void buildPaths() {
            scorePreload = new Path(new BezierLine(startPose, scorePose));
            scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

            grabPickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickUp1Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickUp1Pose.getHeading())
                    .build();

            scorePickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(pickUp1Pose, scorePose))
                    .setLinearHeadingInterpolation(pickUp1Pose.getHeading(), scorePose.getHeading())
                    .build();

            grabPickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickUp2Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickUp2Pose.getHeading())
                    .build();

            scorePickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(pickUp2Pose, scorePose))
                    .setLinearHeadingInterpolation(pickUp2Pose.getHeading(), scorePose.getHeading())
                    .build();

            grabPickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickUp3Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickUp3Pose.getHeading())
                    .build();

            scorePickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(pickUp3Pose, scorePose))
                    .setLinearHeadingInterpolation(pickUp3Pose.getHeading(), scorePose.getHeading())
                    .build();

            
}

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(grabPickup1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    setPathState(3);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(grabPickup3, true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void init() {
        robot.init(hardwareMap);
        robot.initIMU();
        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
}
    @Override
    public void start(){
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
