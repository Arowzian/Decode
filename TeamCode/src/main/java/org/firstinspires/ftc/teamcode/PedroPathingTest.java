//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//
//@TeleOp(name="PedroPathingTest", group="Pushbot") //RT ARM UP LT DOWN
//public class PedroPathingTest extends OpMode {
//
//    Temp_Hardware robot = new Temp_Hardware();
//
//    @Override
//    public void init() {
//        robot.init(hardwareMap);
//        robot.initIMU();
//    }
//
//    @Override
//    public void loop() {
//        telemetry.addData("Pinpoint: ", robot.pinpoint);
//        telemetry.addData("Pinpoint X: ", robot.pinpoint.getPosX());
//        telemetry.addData("Pinpoint Y: ", robot.pinpoint.getPosY(Distance));
//    }
//
//}
