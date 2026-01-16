package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="AHHHH", group="HELP") //RT ARM UP LT DOWN
public class TemporarySolution extends OpMode {

    Decode_Hardware robot = new Decode_Hardware();
    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.initIMU();
    }

    @Override
    public void loop() {

        double intakePower = 0;
        if(gamepad1.a){
            intakePower = 1;
        } else if(gamepad1.b){
            intakePower = -1;
        }
        robot.intakeMotor.setPower(intakePower);

        double lDelMotorPower = (gamepad1.right_trigger - gamepad1.left_trigger) * 0.7;
        double rDelMotorPower = (gamepad1.right_trigger - gamepad1.left_trigger) * 0.7;

        robot.leftDeliveryMotor.setPower(lDelMotorPower);
        robot.rightDeliveryMotor.setPower(rDelMotorPower);

        double indexPower = 0;

        if(gamepad1.y){
            indexPower = 1;
        } else if(gamepad1.x){
            indexPower = -1;
        }

        robot.indexMotor.setPower(indexPower);






    }
}
