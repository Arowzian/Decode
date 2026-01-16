package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="LearnOpMode", group="GroupName")
public class LearnOpMode extends OpMode {
    Temp_Hardware robot = new Temp_Hardware();
    @Override
    public void init() {
       robot.init(hardwareMap);
       robot.initIMU();
    }

    @Override
    public void loop() {
       robot.FL.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x);
       robot.FR.setPower(-gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x);
       robot.BL.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x);
       robot.BR.setPower(-gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x);

    }
}
