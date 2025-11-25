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

@Autonomous(name="AutoFrontRed", group="Pedro") //RT ARM UP LT DOWN
public class AutoFrontRed extends OpMode {


    Temp_Hardware robot = new Temp_Hardware();

    Pose startPose = new Pose(60, 0, Math.toRadians(0));

    Pose shootPose1 = new Pose(0, 0, Math.toRadians(0));
    Pose getOut = new Pose(24, -24, Math.toRadians(45));

    int shootCount = 0;


    private int pathState;

    long storedTime;

    double deliPower = 0;
    double servoPosition = 0;

    private VoltageSensor voltageSensor;

    private Follower follower;

    private Timer pathTimer, actonTimer, opmodeTimer;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.initIMU();
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        shootCount = 0;


        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

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
                deliPower = -0.825 * (12.0 / currentVoltage);
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
