package org.firstinspires.ftc.teamcode.legacy;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@Disabled
@Autonomous(name="AutoBackBlue", group="Pedro") //RT ARM UP LT DOWN
public class AutoBackBlue extends OpMode {

    private Follower follower;
    private Timer pathTimer, actonTimer, opmodeTimer;

    Temp_Hardware robot = new Temp_Hardware();

    // POSES

    Pose startPose = new Pose(48, 8.5, Math.toRadians(90));

    Pose shootPose1 = new Pose(56, 81, Math.toRadians(135));

    Pose getOut = new Pose(56, 110, Math.toRadians(90));

    private int pathState;

    long storedTime;
    int shootCount = 0;

    double deliPower = 0;
    double servoPosition = 0;

    private VoltageSensor voltageSensor;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        robot.init(hardwareMap);
        robot.initIMU();
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    double currentVoltage;
    @Override
    public void loop() {
        follower.update();
        currentVoltage = voltageSensor.getVoltage();

        autonomousPathUpdate();

        robot.deli.setPower(deliPower);
        robot.leftServo.setPower(servoPosition);

        telemetry.addData("Path State:", pathState);
        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.addData("Heading: ", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathState = 0;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate(){
        switch(pathState){
            case 0:
                if(!follower.isBusy()){
                    follower.followPath(shootingPath);
                    pathTimer.resetTimer();
                }
                deliPower = 0;
                servoPosition = 0;

                pathState++;
                break;
            case 1:
                if(!follower.isBusy()){
                    storedTime = System.currentTimeMillis();
                    pathState++;
                }
                break;
            case 2:
                deliPower = (10.5 / currentVoltage) * 0.8;
                if(System.currentTimeMillis() - storedTime >= 2500){
                    servoPosition = -1;
                }
                if(System.currentTimeMillis()-storedTime>=3000){
                    servoPosition = 0;
                    pathState = 1;
                    shootCount++;
                    if(shootCount > 2){
                        pathState = 3;
                        follower.followPath(getOutOfThere);
                        pathTimer.resetTimer();
                    }
                }
                break;
            case 3:

                break;
        }
    }

    Path shootingPath;
    Path getOutOfThere;
    public void buildPaths(){
        shootingPath = new Path(new BezierLine(startPose, shootPose1));
        shootingPath.setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading());
        getOutOfThere = new Path(new BezierLine(shootPose1, getOut));
        getOutOfThere.setLinearHeadingInterpolation(shootPose1.getHeading(), getOut.getHeading());
    }
}
