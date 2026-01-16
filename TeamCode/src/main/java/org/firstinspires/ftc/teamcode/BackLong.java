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

@Autonomous(name="BackLong", group="Pedro") //RT ARM UP LT DOWN
public class BackLong extends OpMode {


    Temp_Hardware robot = new Temp_Hardware();

    Pose startPose = new Pose(0, 0, Math.toRadians(0));
    Pose getOut = new Pose(24, 0, Math.toRadians(0));

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
        robot.pinpoint.resetPosAndIMU();


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
                storedTime = System.currentTimeMillis();
                pathState++;
                break;
            case 1:
                deliPower = (11.3 / currentVoltage);
                if(System.currentTimeMillis() - storedTime >= 2500){
                    servoPosition = -1;
                }
                if(System.currentTimeMillis()-storedTime>=3000){
                    servoPosition = 0;
                    pathState = 0;
                    shootCount++;
                    if(shootCount > 2){
                        pathState = 2;
                        follower.followPath(getOutOfThere);
                        pathTimer.resetTimer();
                    }
                }
                break;
            case 2:
                deliPower = 0;
                servoPosition = 0;
                break;
        }
    }

    Path shootingPath;

    Path getOutOfThere;
    public void buildPaths(){
        getOutOfThere = new Path(new BezierLine(startPose, getOut));
        getOutOfThere.setLinearHeadingInterpolation(startPose.getHeading(), getOut.getHeading());
    }


}
