package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Disabled
@TeleOp(name="TeleOp", group="Pushbot") //RT ARM UP LT DOWN
public class OldTeleOp extends OpMode {

    Temp_Hardware robot = new Temp_Hardware();

    private VoltageSensor voltageSensor;

    boolean slowDeli = false;
    double tgt = 0;

    boolean aPressed = false;
    boolean bPressed = false;

    boolean slowShoot = false;
    boolean fastShoot = false;

    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    boolean theyCalledHimBob = false;

    @Override
    public void init() {
        theyCalledHimBob = true;
        robot.init(hardwareMap);
        robot.initIMU();
        tgt = 0;
        telemetry.addData("Calibration:", robot.imu.getCalibrationStatus());

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        slowDeli = false;
        fastShoot = false;
        slowShoot = false;
        bPressed = false;
        aPressed = false;
        initAprilTag();
    }

    long servoTime = 0;
    double leftServoPower = 0;
//    PIDHelper rotationPID = new PIDHelper(1,0.0005);

    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        }

        builder.addProcessor(aprilTag);

        visionPortal = builder.build();

    }


    @Override
    public void start() {
        configurePinpoint();
    }

    @Override
    public void loop() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        double deliPower = 0;

        double currentVoltage = voltageSensor.getVoltage();

        telemetry.addData("Calibration:", robot.imu.getCalibrationStatus());


        long time = System.currentTimeMillis();
        double heading = robot.imu.getAngularOrientation().firstAngle * (Math.PI / 180);
        telemetry.addData("L'Orientation", robot.imu.getAngularOrientation().firstAngle);
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        // Everything to do with rotation!!!

//        rotationPID.setCurrentPosition(heading);

        if(gamepad1.dpad_down){
            theyCalledHimBob = true;
        }



//        if(currentDetections.size() > 0 && currentDetections.get(0) != null){
//            double fojaewodwjiod = currentDetections.get(0).ftcPose.x;
//            double akwdoiaw = currentDetections.get(0).ftcPose.y;
//            telemetry.addData("Bruh: ", Math.atan2(fojaewodwjiod, akwdoiaw));
//            tgt = heading - Math.toRadians(currentDetections.get(0).ftcPose.yaw - 12);
//        }

        double rx = gamepad1.right_stick_x;

        if(Math.abs(rx) > 0) {
//            rotationPID.setSetpoint(heading);
            tgt = heading;
        } else {
//            rx = -rotationPID.calcPID();
            rx = -robot.autoYawTrim(1,0, 0.0005, heading, tgt);
        }

        telemetry.addData("RX: ", rx);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        robot.FL.setPower(frontLeftPower);
        robot.BL.setPower(backLeftPower);
        robot.BR.setPower(backRightPower);
        robot.FR.setPower(frontRightPower);

        telemetry.addData("FL: ", frontLeftPower);
        telemetry.addData("FR: ", frontRightPower);
        telemetry.addData("BL: ", backLeftPower);
        telemetry.addData("BR: ", backRightPower);

        // * (12.0 / currentVoltage);

        if(gamepad1.right_bumper){
            servoTime = time;
            leftServoPower= -0.3;
        }

        if(time - servoTime > 400){
            leftServoPower = .4;
        }

        robot.leftServo.setPower(leftServoPower);

        robot.pinpoint.update();
        Pose2D pose2D = robot.pinpoint.getPosition();

        telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));

        if(gamepad1.start){
            resetOrientation();
        }

//        if(gamepad1.y && firstYPressed){
//            firstYPressed = false;
//        }
//        if(gamepad1.y && !firstYPressed){
//            firstYPressed = true;
//        }
//        if(!gamepad1.y && firstYPressed){
//            firstYPressed = false;
//        }

//        if(firstYPressed){
//            slowDeli = !slowDeli;
//        }

//        if(gamepad1.right_bumper){
//            slowDeli = true;
//        } else {
//            slowDeli = false;
//        }

        if(!bPressed && gamepad1.b){
            bPressed = true;
            slowShoot = !slowShoot;
        }

        if(!aPressed && gamepad1.a){
            aPressed = true;
            fastShoot = !fastShoot;
        }

        if(!gamepad1.a){
            aPressed = false;
        }

        if(!gamepad1.b){
            bPressed = false;
        }

        if(slowShoot){
            deliPower = (10.5 / currentVoltage) * 0.85;
        }
        if(fastShoot){
            deliPower = (10.5 / currentVoltage);
        }
        if (gamepad1.left_trigger>0){
            deliPower=gamepad1.left_trigger;
        }
        robot.deli.setPower(deliPower);
    }

    public void resetOrientation(){
        robot.initIMU();
        tgt = 0;
    }

    // PID TIME!!!

    /*
    We're only doing P term for now!!!
     */

    public void configurePinpoint(){
        robot.pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        robot.pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        robot.pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        robot.pinpoint.resetPosAndIMU();
    }

}
