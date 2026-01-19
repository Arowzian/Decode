package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Disabled
@TeleOp(name="Motor Testing", group="Pushbot") //RT ARM UP LT DOWN
public class MotorTesting extends OpMode {

    Decode_Hardware robot = new Decode_Hardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.initIMU();
    }

    @Override
    public void loop() {

        double flPower = 0; // bl
        double frPower = 0; // fr
        double blPower = 0; // fl
        double brPower = 0; // br

        robot.pinpoint.update();
        telemetry.addData("Pinpoint X: ", robot.pinpoint.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Y: ", robot.pinpoint.getPosition().getY(DistanceUnit.INCH));

        if(gamepad1.a){
            flPower = 1;
        }
        if(gamepad1.b){
            frPower = 1;
        }
        if(gamepad1.y){
            blPower = 1;
        }
        if(gamepad1.x){
            brPower = 1;
        }

        robot.FL.setPower(flPower);
        robot.FR.setPower(frPower);
        robot.BL.setPower(blPower);
        robot.BR.setPower(brPower);

    }
}