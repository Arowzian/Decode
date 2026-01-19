package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Pushbot") //RT ARM UP LT DOWN
public class TeleOp extends OpMode {

    private VoltageSensor voltageSensor;
    double tgt = 0;

    Decode_Hardware robot = new Decode_Hardware();
    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.initIMU();

        configurePinpoint();

        tgt = 0;

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }

    @Override
    public void loop() {

        robot.setButtonsToggled(gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y);

        double currentVoltage = voltageSensor.getVoltage();

        robot.pinpoint.update();

        //////////////////////
        /// ROBOT MOVEMENT ///
        //////////////////////

        double heading = robot.pinpoint.getHeading(AngleUnit.RADIANS);

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        double rx = gamepad1.right_stick_x;

        if(Math.abs(rx) > 0) {
            tgt = heading;
        } else {
            rx = -robot.autoYawTrim(1,0, 0.0005, heading, tgt);
        }

        telemetry.addData("RX", rx);
        telemetry.addData("Heading", heading);
        telemetry.addData("Target", tgt);

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

        if(gamepad1.right_bumper){
            intakePower = 1;
        } else if(gamepad1.left_bumper){
            intakePower = -1;
        }

        robot.intakeMotor.setPower(intakePower);

        ///////////////////
        /// INDEX MOTOR ///
        ///////////////////

        double indexPower = gamepad1.right_trigger - gamepad1.left_trigger;

        robot.indexMotor.setPower(indexPower);

        ///////////////////////
        /// DELIVERY MOTORS ///
        ///////////////////////

        double deliveryPower = 0;
        if(robot.buttonsToggled[0]){
            deliveryPower = -0.7;
        }
        if(robot.buttonsToggled[1]){
            deliveryPower = -0.5;
        }
        if(robot.buttonsToggled[2]){
            deliveryPower = 0.2;
        }

        robot.leftDeliveryMotor.setPower(deliveryPower);
        robot.rightDeliveryMotor.setPower(deliveryPower);



    }

    public void configurePinpoint(){
        robot.pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        robot.pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        robot.pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        robot.pinpoint.resetPosAndIMU();
    }


}
