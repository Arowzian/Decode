package org.firstinspires.ftc.teamcode.legacy;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@Autonomous(name="Auto, Bad", group="Pedro") //RT ARM UP LT DOWN
@Disabled
public class Auto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actonTimer, opmodeTimer;

    Temp_Hardware robot = new Temp_Hardware();

    // POSES

    Pose startPose = new Pose(96, 8.5, Math.toRadians(90));

    Pose shootPose1 = new Pose(90, 82, Math.toRadians(45));

    private int pathState;

    long coolTimer;

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
    @Override
    public void loop() {

        double currentVoltage = voltageSensor.getVoltage();
        telemetry.addData("Voltage: ", currentVoltage);

        follower.update();

        if(!follower.isBusy()){
            autonomousPathUpdate();
        }

        if(pathState > 1 && pathState < 5){

            deliPower = -0.735 * (12.0 / currentVoltage);

            if(System.currentTimeMillis() - coolTimer > 2000){
                coolTimer = System.currentTimeMillis();
                servoPosition = 0;
                pathState++;

            } else if(System.currentTimeMillis() - coolTimer > 1500){
                servoPosition = -1;
            }
        } else if(pathState > 4){
            deliPower = 0;
            servoPosition = 0;
        }

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
                follower.followPath(shootingPath);
                pathState++;
                pathTimer.resetTimer();
                deliPower = 0;
                servoPosition = 0;
                break;
            case 1:
                pathState++;
                coolTimer = System.currentTimeMillis();
                break;
            case 2:
            case 3:
                break;
            case 4:
                deliPower = 0;
                servoPosition = 0;
                break;
        }
    }

    Path shootingPath;
    public void buildPaths(){
        shootingPath = new Path(new BezierLine(startPose, shootPose1));
        shootingPath.setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading());
    }
}
