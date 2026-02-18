package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.files.DataLogger;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.IOException;
import java.util.List;

@TeleOp(name="Webcam", group="Pushbot") //RT ARM UP LT DOWN
public class ComputerEye extends OpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private double distance;

    long timeElapsed;

    DataLogger dataLogger = new DataLogger("positionalData");


    Decode_Hardware robot = new Decode_Hardware();

    public ComputerEye() throws IOException {
    }

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.initIMU();

        distance = 0;
        configurePinpoint();
        initAprilTag();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        robot.pinpoint.update();
        try {
            dataLogger.addDataLine("6458BurgbotsInfo: " + robot.pinpoint.getPosX(DistanceUnit.INCH) + ", 4!End!");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (distance == 0){
                distance = detection.ftcPose.range;
//                robot.pinpoint.setPosX(Math.abs(distance),DistanceUnit.INCH);
                robot.pinpoint.setHeading(90,AngleUnit.DEGREES);
                timeElapsed = System.nanoTime();
            }
        }   // end for() loop



        if(timeElapsed > 1000000000) {

            double tgt = Math.atan2(robot.pinpoint.getPosY(DistanceUnit.INCH) + distance, -robot.pinpoint.getPosX(DistanceUnit.INCH));
            double heading = robot.pinpoint.getHeading(AngleUnit.RADIANS);
            double rx = -robot.autoYawTrim(1.5, 0, 0.0005, heading, tgt);

            telemetry.addData("Heading", heading);
            telemetry.addData("Target", tgt);
            telemetry.addData("X", robot.pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("Y", robot.pinpoint.getPosY(DistanceUnit.INCH));

            // double rx = -robot.autoYawTrim(1.5,0, 0.0005, heading, Math.toRadians(target));


            double frontLeftPower = (rx);
            double backLeftPower = (rx);
            double frontRightPower = (-rx);
            double backRightPower = (-rx);

            robot.FL.setPower(frontLeftPower);
            robot.BL.setPower(backLeftPower);
            robot.BR.setPower(backRightPower);
            robot.FR.setPower(frontRightPower);

        }

    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    public void configurePinpoint(){
        robot.pinpoint.setOffsets(0, 6, DistanceUnit.INCH);
        robot.pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        robot.pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        robot.pinpoint.resetPosAndIMU();
        robot.pinpoint.setHeading(90, AngleUnit.DEGREES);
    }
}
