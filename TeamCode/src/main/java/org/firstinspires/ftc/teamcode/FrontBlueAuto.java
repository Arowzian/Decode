package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;


@Autonomous(name="FrontBlueAuto", group="Yay...")
public class FrontBlueAuto extends OpMode {
    double deliveryPower;
    double intakePower;
    Decode_Hardware robot = new Decode_Hardware();
    private Follower follower;
    long storedTime;
    double servoPosition = 0;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(24, 120, Math.toRadians(135));
    private final Pose scorePose = new Pose(58,79, Math.toRadians(135));
    private final Pose pickUp1Pose = new Pose(19, 83, Math.toRadians(180));
    private final Pose startPickUp2Pose = new Pose(58, 55, Math.toRadians(180));
    private final Pose endPickUp2Pose = new Pose(19, 55, Math.toRadians(180));
    private final Pose startPickUp3Pose = new Pose(58, 48, Math.toRadians(180));
    private final Pose endPickUp3Pose = new Pose(19, 48, Math.toRadians(180));
    private final Pose offWhiteLinePose = new Pose(58, 108, Math.toRadians(90));
    private Path scorePreload;
    private Path grabPickup1, scorePickup1, startGrabPickup2, endGrabPickup2, scorePickup2, startGrabPickup3, endGrabPickup3, scorePickup3, getOffWhiteLine;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        scorePreload.setHeadingConstraint(Math.toRadians(10));
        scorePreload.setTranslationalConstraint(1);

        grabPickup1 = new Path(new BezierLine(scorePose, pickUp1Pose));
        grabPickup1.setHeadingInterpolation(HeadingInterpolator.constant(pickUp1Pose.getHeading()));

        scorePickup1 = new Path(new BezierLine(pickUp1Pose, scorePose));
        scorePickup1.setLinearHeadingInterpolation(pickUp1Pose.getHeading(), scorePose.getHeading());

        startGrabPickup2 = new Path(new BezierLine(scorePose, startPickUp2Pose));
        startGrabPickup2.setLinearHeadingInterpolation(scorePose.getHeading(), startPickUp2Pose.getHeading());

        endGrabPickup2 = new Path(new BezierLine(startPickUp2Pose, endPickUp2Pose));
        endGrabPickup2.setLinearHeadingInterpolation(startPickUp2Pose.getHeading(), endPickUp2Pose.getHeading());

        scorePickup2 = new Path(new BezierLine(endPickUp2Pose, scorePose));
        scorePickup2.setLinearHeadingInterpolation(endPickUp2Pose.getHeading(), scorePose.getHeading());

        startGrabPickup3 = new Path(new BezierLine(scorePose, startPickUp3Pose));
        startGrabPickup3.setLinearHeadingInterpolation(scorePose.getHeading(), startPickUp3Pose.getHeading());

        endGrabPickup3 = new Path(new BezierLine(startPickUp3Pose, endPickUp3Pose));
        endGrabPickup3.setLinearHeadingInterpolation(startPickUp3Pose.getHeading(), endPickUp3Pose.getHeading());

        scorePickup3 = new Path(new BezierLine(endPickUp3Pose, scorePose));
        scorePickup3.setLinearHeadingInterpolation(endPickUp3Pose.getHeading(), scorePose.getHeading());

        getOffWhiteLine = new Path(new BezierLine(scorePose, offWhiteLinePose));
        getOffWhiteLine.setLinearHeadingInterpolation(scorePose.getHeading(), offWhiteLinePose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Moves to score pose
                follower.followPath(scorePreload);
                telemetry.addData("Busy", follower.isBusy());
                if (follower.isBusy() && follower.getPathCompletion() > 0.95) {
                    storedTime = System.currentTimeMillis();
                    setPathState(1);
                }
                break;
            case 1: // Shoots preload
                deliveryPower = 0.65;
                if (System.currentTimeMillis() - storedTime >= 2000) {
                    robot.indexMotor.setPower(0.6);
                }
                if (System.currentTimeMillis() - storedTime >= 3500) {
                    robot.indexMotor.setPower(0);
                    deliveryPower = 0;
                    setPathState(2);
                }
                break;
            case 2: // Picks up the farthest row of artifacts
                follower.followPath(grabPickup1, true);
                intakePower = 0.6;
                robot.indexMotor.setPower(0.45);
                deliveryPower = -0.4;
                robot.indexServo.setPosition(0.6);
                if ((follower.isBusy() && follower.getPathCompletion() > 0.95) || pathTimer.getElapsedTimeSeconds() > 5) {
                    storedTime = System.currentTimeMillis();
                    setPathState(3);
                }
                break;
            case 3: // Travels to score pose with Pickup One + preps deli motor
                follower.followPath(scorePickup1, true);
//                deliveryPower = 0.67;
                robot.indexServo.setPosition(0);
                robot.indexMotor.setPower(0);
                if (follower.isBusy() && follower.getPathCompletion() > 0.95) {
                    storedTime = System.currentTimeMillis();
                    setPathState(4);
                }
                break;
            case 4: // Scores Pickup One
                deliveryPower = 0.65;
                if (System.currentTimeMillis() - storedTime >= 2000) {
                    robot.indexMotor.setPower(0.6);
                }
                if (System.currentTimeMillis() - storedTime >= 3500) {
                    robot.indexMotor.setPower(0);
                    deliveryPower = 0;
                    setPathState(5);
                }
                break;
            case 5: // Moves down to y-level for pickup 2 (middle row of artifacts)
                follower.followPath(startGrabPickup2, true);
                if (follower.isBusy() && follower.getPathCompletion() > 0.95) {
                    setPathState(6);
                }
                break;
            case 6: // Loads Pickup 2 (middle row of artifacts)
                follower.followPath(endGrabPickup2, true);
                intakePower = 0.8;
                robot.indexMotor.setPower(0.8);
                deliveryPower = -0.4;
                if (!follower.isBusy()) {
                    storedTime = System.currentTimeMillis();
                    setPathState(7);
                }
                break;
            case 7: // Travels to score position
                follower.followPath(scorePickup2, true);
                deliveryPower = 0.65;
                if (follower.isBusy() && follower.getPathCompletion() > 0.95) {
                    storedTime = System.currentTimeMillis();
                    setPathState(8);
                }
                break;
            case 8: // Scores Pickup 2
                deliveryPower = 0.65;
                if (System.currentTimeMillis() - storedTime >= 2000) {
                    robot.indexMotor.setPower(0.8);
                }
                if (System.currentTimeMillis() - storedTime >= 3500) {
                    robot.indexMotor.setPower(0);
                    deliveryPower = 0;
                    setPathState(9);
                }
                break;
            case 9: // Travels to the y level for Pickup 3
                follower.followPath(startGrabPickup3, true);
                if (follower.isBusy() && follower.getPathCompletion() > 0.95) {
                    setPathState(10);
                }
                break;
            case 10: // Intakes Pickup 3
                follower.followPath(endGrabPickup3, true);
                intakePower = 0.8;
                robot.indexMotor.setPower(0.8);
                deliveryPower = -0.2;
                if (follower.isBusy() && follower.getPathCompletion() > 0.95) {
                    storedTime = System.currentTimeMillis();
                    setPathState(11);
                }
                break;
            case 11: // Travels to score pose + preps deli motor
                follower.followPath(scorePickup3, true);
                deliveryPower = 0.65;
                if (follower.isBusy() && follower.getPathCompletion() > 0.95) {
                    storedTime = System.currentTimeMillis();
                    setPathState(12);
                }
                break;
            case 12: // Shoots Pickup 3
                deliveryPower = 0.65;
                if (System.currentTimeMillis() - storedTime >= 2000) {
                    robot.indexMotor.setPower(0.8);
                }
                if (System.currentTimeMillis() - storedTime >= 3500) {
                    robot.indexMotor.setPower(0);
                    deliveryPower = 0;
                    setPathState(13);
                }
                break;
            case 13: // Moves off the white line
                follower.followPath(getOffWhiteLine);
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        opmodeTimer.resetTimer();
    }

    public void init() {
        deliveryPower = 0;

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        robot.init(hardwareMap);
        robot.initIMU();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

//        telemetry.addData("Heading Error", follower.getHeadingError());
//        telemetry.addData("Translational Error", follower.getTranslationalError());

        robot.leftDeliveryMotor.setPower(deliveryPower);
        robot.rightDeliveryMotor.setPower(deliveryPower);

        robot.intakeMotor.setPower(intakePower);

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("PathBe", follower.getPathCompletion());
        telemetry.addData("FL", robot.FL.getPower());
        telemetry.addData("FR", robot.FR.getPower());
        telemetry.addData("BL", robot.BL.getPower());
        telemetry.addData("BR", robot.BR.getPower());
        telemetry.update();
    }
    @Override
    public void start(){
        opmodeTimer.resetTimer();
        setPathState(0);
        pathTimer.resetTimer();
        servoPosition = -1;
    }
}
