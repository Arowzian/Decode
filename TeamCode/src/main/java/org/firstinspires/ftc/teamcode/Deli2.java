package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Deli2", group="Pushbot") //RT ARM UP LT DOWN
public class Deli2 extends OpMode {
    Temp_Hardware robot = new Temp_Hardware();
    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.initIMU();
    }

    @Override
    public void loop() {
        robot.deli.setPower((gamepad1.left_trigger - gamepad1.right_trigger));
        robot.deli2.setPower((gamepad1.left_trigger - gamepad1.right_trigger));
    }
    /*
    if (gamepad1.
     */
}
