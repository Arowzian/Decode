package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.files.DataLogger;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.IOException;
import java.util.List;
import java.util.logging.Logger;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Pushbot") //RT ARM UP LT DOWN
public class TeleOp extends OpMode {

    // PANELS IP: http://192.168.43.1:8001/

    Logger dataLogger = Logger.getLogger("Burg");

    double robotPowerAddition;

    boolean dpadUpDown;

    double bearing = 0;
    boolean dpadDownDown;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private VoltageSensor voltageSensor;
    double tgt = 0;

    int previousDeliveryEncoder = 0;

    boolean detected = false;

    int lDelEncoder = 0;
    int rDelEncoder = 0;

    long prevTime = 0;
    double servoPower = 0;

    double distance = 0;
    long timeElapsed = 0;

    boolean resetingOrientation = false;



    Decode_Hardware robot = new Decode_Hardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.initIMU();
        previousDeliveryEncoder = 0;
        configurePinpoint();
        resetingOrientation = false;
        bearing = 0;
        timeElapsed = 0;

//        initAprilTag();

        tgt = 0;
        distance = 0;

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");


        servoPower = 0;
        rDelEncoder = 0;
        lDelEncoder = 0;
        prevTime = 0;

        detected = false;

        robotPowerAddition = 0;
    }

    @Override
    public void start() {
        robot.pinpoint.resetPosAndIMU();
    }

    @Override
    public void loop() {

        if(gamepad1.start){
            resetOrientation();
            resetingOrientation = true;
        } else {
            resetingOrientation = false;
        }

        robot.setButtonsToggled(gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y);

        double currentVoltage = voltageSensor.getVoltage();

        robot.pinpoint.update();

        //////////////////////
        /// ROBOT MOVEMENT ///
        //////////////////////

//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

//        for (AprilTagDetection detection : currentDetections) {
//            if (distance == 0){
//                distance = detection.ftcPose.range;
////                robot.pinpoint.setPosX(Math.abs(distance),DistanceUnit.INCH);
//                robot.pinpoint.setHeading(90,AngleUnit.DEGREES);
//                timeElapsed = System.nanoTime();
//                detected = true;
//            }
//        }


        double heading = robot.pinpoint.getHeading(AngleUnit.RADIANS);


        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        double rx = gamepad1.right_stick_x;

        if(Math.abs(rx) > 0) {
            tgt = heading;
        } else if(!resetingOrientation){
            rx = -robot.autoYawTrim(1.5,0, 0.0005, heading, tgt + bearing);
//            if(detected && distance != 0 && timeElapsed - System.nanoTime() > 1000000000){
//                tgt = Math.atan2(robot.pinpoint.getPosY(DistanceUnit.INCH) + distance, -robot.pinpoint.getPosX(DistanceUnit.INCH));
//            }
//            telemetry.addData("X", robot.pinpoint.getPosX(DistanceUnit.INCH));
//            rx = -robot.autoYawTrim(1.5, 0, 0.0005, heading, tgt);
        }



        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;


        robot.FL.setPower(frontLeftPower);
        robot.BL.setPower(backLeftPower);
        robot.BR.setPower(backRightPower);
        robot.FR.setPower(frontRightPower);

        ////////////////////
        /// ROBOT INTAKE ///
        ////////////////////
        double intakePower = 0;

        servoPower = 0.6;
        if(gamepad1.right_bumper){
            intakePower = 1;
        } else if(gamepad1.left_bumper){
            intakePower = -1;
        }


        robot.intakeMotor.setPower(intakePower);

        ///////////////////
        /// INDEX MOTOR ///
        ///////////////////

        double indexPower = (gamepad1.right_trigger - gamepad1.left_trigger) * 0.8;

        robot.indexMotor.setPower(indexPower);

        ///////////////////////
        /// DELIVERY MOTORS ///
        ///////////////////////

        previousDeliveryEncoder = robot.leftDeliveryMotor.getCurrentPosition();

        if(!dpadDownDown && gamepad1.dpad_down){
            dpadDownDown = true;
            robotPowerAddition -= 0.01;
        }
        if(dpadDownDown && !gamepad1.dpad_down){
            dpadDownDown = false;
        }

        if(!dpadUpDown && gamepad1.dpad_up){
            dpadUpDown = true;
            robotPowerAddition += 0.025;
        }
        if(dpadUpDown && !gamepad1.dpad_up){
            dpadUpDown = false;
        }

        telemetry.addData("RobotPowerAddition", robotPowerAddition);

        double deliveryPower = 0;
        if(robot.buttonsToggled[0]){
            deliveryPower = 0.9 + robotPowerAddition;
            servoPower = 0;
//
        }
        if(robot.buttonsToggled[1]){
            deliveryPower = 0.75 + robotPowerAddition;
            servoPower = 0;
        }
        if(robot.buttonsToggled[2]){
            deliveryPower = -0.1;
        }

//        robot.leftDeliveryMotor.setPower(deliveryPower * (10.5 / currentVoltage));
//        robot.rightDeliveryMotor.setPower(deliveryPower * (10.5 / currentVoltage));

        robot.leftDeliveryMotor.setPower(deliveryPower);
        robot.rightDeliveryMotor.setPower(deliveryPower);
        robot.indexServo.setPosition(servoPower);

        long deltaTime = System.currentTimeMillis() - prevTime;

        prevTime = System.currentTimeMillis();

        dataLogger.info("6458BurgbotsInfo: " + (double)(robot.leftDeliveryMotor.getCurrentPosition() - lDelEncoder) / deltaTime + ", 4!End!");

        lDelEncoder = robot.leftDeliveryMotor.getCurrentPosition();
        rDelEncoder = robot.rightDeliveryMotor.getCurrentPosition();

    }

    public void resetOrientation(){
        robot.pinpoint.resetPosAndIMU();
        tgt = 0;
    }

//    private void initAprilTag() {
//
//        // Create the AprilTag processor.
//        aprilTag = new AprilTagProcessor.Builder()
//                .build();
//
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//
//        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//
//        builder.addProcessor(aprilTag);
//        visionPortal = builder.build();
//    }

    public void configurePinpoint(){
        robot.pinpoint.setOffsets(0, 6, DistanceUnit.INCH); //these are tuned for 3110-0002-0001 Product Insight #1
        robot.pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        robot.pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        robot.pinpoint.resetPosAndIMU();
    }

}
