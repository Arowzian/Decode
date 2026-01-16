package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="AprilTagProcesser", group="Pushbot") //RT ARM UP LT DOWN
public class AprilTagProcesser extends OpMode {
    Temp_Hardware robot = new Temp_Hardware();
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;

    boolean yar = false;

    Pose2D difference;
    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.initIMU();
        initAprilTag();
        difference = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES, 0);
        yar = false;
    }

    @Override
    public void start() {
        configurePinpoint();
    }

    @Override
    public void loop() {
        double heading = robot.imu.getAngularOrientation().firstAngle;
        double rx;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        robot.pinpoint.update();
        Pose2D pose2D = robot.pinpoint.getPosition();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if(detection.id == 23) {
                telemetry.addLine("Purple Purple Green");
            } else if (detection.id == 22) {
                telemetry.addLine("Purple Green Purple");
            } else if(detection.id == 21){
                telemetry.addLine("Green Purple Purple");
            } else if(detection.id == 24 && !yar){
                telemetry.addData("X: ",detection.ftcPose.x);
                telemetry.addData("Y: ",detection.ftcPose.y);
                telemetry.addData("Z: ",detection.ftcPose.z);

                robot.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, detection.ftcPose.x, detection.ftcPose.y, AngleUnit.DEGREES, robot.pinpoint.getHeading(AngleUnit.DEGREES)));
//                difference = robot.pinpoint.getPosition();
                difference = new Pose2D(DistanceUnit.INCH, detection.ftcPose.x, detection.ftcPose.y, AngleUnit.DEGREES, robot.pinpoint.getHeading(AngleUnit.DEGREES));
                yar = !yar;
            }
        }

        telemetry.addData("Pinpoint X", robot.pinpoint.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Y", robot.pinpoint.getPosition().getY(DistanceUnit.INCH));

        double distanceX = - robot.pinpoint.getPosition().getX(DistanceUnit.INCH);
        double distanceY = 2 * difference.getY(DistanceUnit.INCH) - robot.pinpoint.getPosition().getY(DistanceUnit.INCH);
        telemetry.addData("Difference" , difference.getY(DistanceUnit.INCH));

        telemetry.addData("Distance from AprilTag, X:", distanceX);
        telemetry.addData("Distance from AprilTag, Y:", distanceY);

        telemetry.addData("Angle Between: ", Math.toDegrees(Math.atan2(distanceY, distanceX)) - 90);

        telemetry.addData("Heading", heading);
        telemetry.addData("trig: ", Math.toDegrees(Math.atan2(distanceY, distanceX)) - 90);
        double tgt = heading - Math.toDegrees(Math.atan2(distanceY, distanceX)) - 90;


        rx = -robot.autoYawTrim(.05, 0,0.0005, heading, tgt);

        double frontLeftPower = -gamepad1.left_stick_y + gamepad1.left_stick_x + rx;
        double backLeftPower = -gamepad1.left_stick_y - gamepad1.left_stick_x + rx;
        double frontRightPower = -gamepad1.left_stick_y - gamepad1.left_stick_x - rx;
        double backRightPower = -gamepad1.left_stick_y + gamepad1.left_stick_x - rx;

        telemetry.addData("frontLeftPower", frontLeftPower);
        telemetry.addData("backLeftPower", backLeftPower);
        telemetry.addData("frontLeftPower", frontRightPower);
        telemetry.addData("backRightPower", backRightPower);

        robot.FL.setPower(frontLeftPower);
        robot.BL.setPower(backLeftPower);
        robot.BR.setPower(backRightPower);
        robot.FR.setPower(frontRightPower);

    }
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

    }

    public void configurePinpoint(){
        robot.pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        robot.pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        robot.pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        robot.pinpoint.resetPosAndIMU();
    }
}
