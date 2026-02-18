package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;


@Autonomous(name="BackBlueAuto", group="Yay...")
public class BackBlueAuto extends OpMode {
    double deliveryPower;
    double intakePower;
    Decode_Hardware robot = new Decode_Hardware();
    private Follower follower;
    long storedTime;
    double servoPosition = 0;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(48, 9, Math.toRadians(90));
    private final Pose scorePose = new Pose(58,20, Math.toRadians(114));
    private final Pose setUpPickUp1 = new Pose(54, 36, Math.toRadians(180));
    private final Pose pickUp1Pose = new Pose(19, 36, Math.toRadians(180));
    private final Pose startPickUp2Pose = new Pose(44, 60, Math.toRadians(180));
    private final Pose pickUp2Pose = new Pose(19, 60, Math.toRadians(180));
    private Path scorePreload;

    private PathChain chain;
    private Path grabPickup1, scorePickup1, startGrabPickup2, scoreToSetup1, grabPickup2, scorePickup2;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        scorePreload.setHeadingConstraint(Math.toRadians(10));
        scorePreload.setTranslationalConstraint(1);

        scoreToSetup1 = new Path(new BezierLine(scorePose, setUpPickUp1));
        scoreToSetup1.setHeadingInterpolation(HeadingInterpolator.constant(setUpPickUp1.getHeading()));

        grabPickup1 = new Path(new BezierLine(setUpPickUp1, pickUp1Pose));
        grabPickup1.setHeadingInterpolation(HeadingInterpolator.constant(pickUp1Pose.getHeading()));

        chain = follower.pathBuilder()
                .addPath(grabPickup1)
                .build();

        scorePickup1 = new Path(new BezierLine(pickUp1Pose, scorePose));
//        scorePickup1.setLinearHeadingInterpolation(pickUp1Pose.getHeading(), scorePose.getHeading());
        scorePickup1.setHeadingInterpolation(HeadingInterpolator.linear(pickUp1Pose.getHeading(), scorePose.getHeading()));

        startGrabPickup2 = new Path(new BezierLine(scorePose, startPickUp2Pose));
        startGrabPickup2.setLinearHeadingInterpolation(scorePose.getHeading(), startPickUp2Pose.getHeading());

        grabPickup2 = new Path(new BezierLine(startPickUp2Pose, pickUp2Pose));
        startGrabPickup2.setLinearHeadingInterpolation(startPickUp2Pose.getHeading(), pickUp2Pose.getHeading());

        scorePickup2 = new Path(new BezierLine(pickUp2Pose, scorePose));
        scorePickup2.setLinearHeadingInterpolation(pickUp2Pose.getHeading(), scorePose.getHeading());

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Moves to score pose
                follower.followPath(scorePreload);
                telemetry.addData("Busy", follower.isBusy());
                if (follower.isBusy() && follower.getPathCompletion() > 0.98) {
                    storedTime = System.currentTimeMillis();
                    setPathState(1);
                }
                break;
            case 1: // Shoots preload
                deliveryPower = 0.9;
                robot.indexServo.setPosition(0);
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
                follower.followPath(scoreToSetup1, false);
//                intakePower = 0.6;
//                robot.indexMotor.setPower(0.45);
//                deliveryPower = -0.4;
//                robot.indexServo.setPosition(0.6);
                if ((follower.isBusy() && follower.getPathCompletion() > 0.98) || pathTimer.getElapsedTimeSeconds() > 5) {
                    storedTime = System.currentTimeMillis();
                    setPathState(3);
                }
                break;
            case 3: // Picks up the farthest row of artifacts
                follower.followPath(chain,0.5 , false);
                intakePower = 0.6;
                robot.indexMotor.setPower(0.7);
//                deliveryPower = -0.4;
                robot.indexServo.setPosition(0.6);
                if ((follower.isBusy() && follower.getPathCompletion() > 0.98) || pathTimer.getElapsedTimeSeconds() > 5) {
                    storedTime = System.currentTimeMillis();
                    Pose storedPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
                    follower = Constants.createFollower(hardwareMap);
                    follower.setStartingPose(storedPose);
                    setPathState(4);

                }
                break;
            case 4: // Travels to score pose with Pickup One + preps deli motor
                follower.followPath(scorePickup1, true);
                telemetry.addData("Goal", follower.getHeadingGoal(1));
                telemetry.addData("Goal", follower.getCurrentPath().getPose(1).getHeading());

//                deliveryPower = 0.67;
//                robot.indexServo.setPosition(0);
                if (follower.isBusy() && follower.getPathCompletion() > 0.98) {
                    storedTime = System.currentTimeMillis();
                    setPathState(5);
                }
                break;
            case 5: // Scores Pickup One
                robot.indexServo.setPosition(-1);
                deliveryPower = 0.9;
                if (System.currentTimeMillis() - storedTime >= 0 && System.currentTimeMillis() - storedTime < 500) {
                    robot.indexMotor.setPower(-0.3);
                } else if(System.currentTimeMillis() - storedTime < 2000){
                    robot.indexMotor.setPower(0);
                }
                if (System.currentTimeMillis() - storedTime >= 2000) {
                    robot.indexMotor.setPower(0.6);
                }
                if (System.currentTimeMillis() - storedTime >= 3500) {
                    robot.indexMotor.setPower(0);
                    deliveryPower = 0;
                    setPathState(6);
                }
                break;
            case 6: // Moves down to y-level for pickup 2 (middle row of artifacts)
                follower.followPath(startGrabPickup2, true);
                if (follower.isBusy() && follower.getPathCompletion() > 0.95) {
                    setPathState(7);
                }
                break;
            case 7: // Picks up the middle row of artifacts
                follower.followPath(grabPickup2, false);
                intakePower = 0.6;
                robot.indexMotor.setPower(0.7);
//                deliveryPower = -0.4;
                robot.indexServo.setPosition(0.6);
                if ((follower.isBusy() && follower.getPathCompletion() > 0.98) || pathTimer.getElapsedTimeSeconds() > 5) {
                    storedTime = System.currentTimeMillis();
                    Pose storedPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
                    follower = Constants.createFollower(hardwareMap);
                    follower.setStartingPose(storedPose);
                    setPathState(8);

                }
                break;
            case 8: // Travels to score pose with Pickup Two + preps deli motor
                follower.followPath(scorePickup2, true);

                if (follower.isBusy() && follower.getPathCompletion() > 0.98) {
                    storedTime = System.currentTimeMillis();
                    setPathState(9);
                }
                break;
            case 9: // Scores Pickup One
                robot.indexServo.setPosition(-1);
                deliveryPower = 0.9;
                robot.indexMotor.setPower(0);
                if (System.currentTimeMillis() - storedTime >= 0 && System.currentTimeMillis() - storedTime < 500) {
                    robot.indexMotor.setPower(-0.3);
                } else if(System.currentTimeMillis() - storedTime < 2000){
                    robot.indexMotor.setPower(0);
                }
                if (System.currentTimeMillis() - storedTime >= 2000) {
                    robot.indexMotor.setPower(0.6);
                }
                if (System.currentTimeMillis() - storedTime >= 3500) {
                    robot.indexMotor.setPower(0);
                    deliveryPower = 0;
                    setPathState(6);
                }
                break;
            default:
                deliveryPower = 0;
                servoPosition = 0;
                intakePower = 0;
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
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
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
        servoPosition = 0;
    }
}
