package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.logging.Logger;

@TeleOp(name="OpMolaMola", group="Pushbot")
public class OperationMolaMola extends OpMode {

    Decode_Hardware robot = new Decode_Hardware();

    long prevTime = 0;

    Logger dataLogger = Logger.getLogger("MMData");

    int lDelEncoder = 0;
    int rDelEncoder = 0;
    double servoPower = 0;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.initIMU();

        prevTime = 0;

        lDelEncoder = 0;
        rDelEncoder = 0;

        servoPower = 0;

    }

    @Override
    public void start() {
        prevTime = System.nanoTime();
    }

    @Override
    public void loop() {

        robot.setButtonsToggled(gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y);

        double deliveryPower = 0;
        servoPower = 0.6;

        if(robot.buttonsToggled[1]){
            deliveryPower = 0.75;
            servoPower = 0;
        }

        robot.leftDeliveryMotor.setPower(deliveryPower);
        robot.rightDeliveryMotor.setPower(deliveryPower);

        robot.indexServo.setPosition(servoPower);

        robot.indexMotor.setPower((gamepad1.right_trigger - gamepad1.left_trigger) * 0.8);

        long deltaTime = System.nanoTime() - prevTime;
        prevTime = System.nanoTime();

        deltaTime /= 1000000;

        telemetry.addData("Delta Time", deltaTime);

        dataLogger.info("6458BurgbotsInfo: " + (double)(robot.leftDeliveryMotor.getCurrentPosition() - lDelEncoder) / deltaTime + ", 4!End!");

        lDelEncoder = robot.leftDeliveryMotor.getCurrentPosition();
        rDelEncoder = robot.rightDeliveryMotor.getCurrentPosition();

    }
}
