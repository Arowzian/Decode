package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.helpers.PIDHelper;

@TeleOp(name="Deli Testing", group="Pushbot") //RT ARM UP LT DOWN
public class TempTest extends OpMode {

    Temp_Hardware robot = new Temp_Hardware();

    private VoltageSensor voltageSensor;

    double tgt = 0;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.initIMU();
        tgt = 0;
        telemetry.addData("Calibration:", robot.imu.getCalibrationStatus());

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

    }

    long servoTime = 0;
    double leftServoPower = 0;
//    PIDHelper rotationPID = new PIDHelper(1,0.0005);


    @Override
    public void start() {
        configurePinpoint();
    }

    @Override
    public void loop() {
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

        deliPower = gamepad1.left_trigger - gamepad1.right_trigger * (12.0 / currentVoltage);
        // * (12.0 / currentVoltage);

        if(gamepad1.right_bumper){
            servoTime = time;
            leftServoPower= -1;
        }
        if(time - servoTime > 500){
            leftServoPower = 0;
        }

        robot.leftServo.setPower(leftServoPower);

        robot.pinpoint.update();
        Pose2D pose2D = robot.pinpoint.getPosition();

        telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));

        if(gamepad1.a){
            resetOrientation();
        }
        if (gamepad1.y){
            deliPower = 0.85;
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
