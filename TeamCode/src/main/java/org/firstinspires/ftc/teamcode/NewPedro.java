package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@Autonomous(name="NewPedro", group="Pedro") //RT ARM UP LT DOWN
public class NewPedro extends OpMode {

    private Follower follower;
    private Timer pathTimer, actonTimer, opmodeTimer;
    Temp_Hardware robot = new Temp_Hardware();
    Pose startPose = new Pose(0, 0, Math.toRadians(0));

    Pose shootPose1 = new Pose(0, 0, Math.toRadians(0));

    @Override
    public void init() {
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
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        robot.pinpoint.update();
        follower.update();
        follower.followPath(testPath);
//        follower.followPath(testPath);
        telemetry.addData("Bruh: ", follower.isBusy());
        telemetry.addData("Bruh2: ", follower.getPose());
    }

    Path testPath;
    public void buildPaths(){
        testPath = new Path(new BezierLine(startPose, shootPose1));
        testPath.setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading());
    }
}
