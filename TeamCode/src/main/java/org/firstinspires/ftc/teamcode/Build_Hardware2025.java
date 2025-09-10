package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.helpers.PIDHelper;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


public class Build_Hardware_2025 {

    public DcMotor FL;
    public DcMotor FR;
    public DcMotor BL;
    public DcMotor BR;

    public DcMotor liftR;
    public DcMotor liftL;
    public DcMotor intake;

    public CRServo conveyor;

    public Servo flipper;
    public Servo claw;

    public Servo drone;

    public Servo spike;


    public ColorSensor colorSensor;

    public ColorSensor color2;

    HardwareMap hardwareMap;

    BNO055IMU imu;

    double previousOutput;


    double previous_time;
    double previous_timeLift;
    double previous_timeEncoder;

    double I = 0.0;
    double IEncoder = 0.0;
    double ILift = 0.0;

    double previous_error;
    double previous_errorLift;
    double previous_errorEncoder;

    double globalAngle;

    int finalEncoder;

    Orientation lastAngles = new Orientation();

    PIDHelper autonPID = new PIDHelper(0.05, 0.0003, 0);


    double prevTime = 0;
    int prevFLPosition = 0;
    int prevFRPosition = 0;
    int prevBLPosition = 0;
    int prevBRPosition = 0;


    //TODO These are the init methods

    // initializes the imu and necessary pieces
    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    // initializes the hardware map with all motors, servos, and sensors
    public void init(HardwareMap i) {
        hardwareMap = i;

        FL = initMotor("FL", true);    // (Competition robot (Port 0) = true) (Melvin the 3rd (Port 2) = false)
        FR = initMotor("FR", false);   // (Competition robot (Port 1) = false) (Melvin the 3rd (Port 3) = true)
        BL = initMotor("BL", true);    // (Competition robot (Port 2) = true) (Melvin the 3rd (Port 1) = false)
        BR = initMotor("BR", false); // (Competition robot (Port 3) = false) (Melvin the 3rd (Port 0) = true)
        liftR = initMotor("liftR", false);
        liftL = initMotor("liftL", false);
        intake = initMotor("intake", true);
        flipper = i.get(Servo.class, "flipper");
        claw = i.get(Servo.class, "claw");
        conveyor = i.get(CRServo.class, "conveyor");
        spike = i.get(Servo.class, "spike");

        drone = i.get(Servo.class, "drone");



        colorSensor = i.get(ColorSensor.class, "color");


    }

    // method used to initialize motors and set their behaviors
    public DcMotor initMotor(String name, boolean isReverse) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        if (isReverse) {
            motor.setDirection(DcMotor.Direction.REVERSE);
        } else {
            motor.setDirection(DcMotor.Direction.FORWARD);
        }

        motor.setPower(0.0);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        return motor;
    }


    //TODO These are get and check methods that return values

    // returns the angles of the imu
    public Orientation checkOrientation() {
        Orientation angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        this.imu.getPosition();
        return angles;
    }

    // returns the hardware map
    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    //TODO These are methods that deal with limits

    //changes angle to radians 0 < x < 2pi
    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

    // makes sure that -1 < power < 1
    public double limitToPoles(double input, double max, double min) {
        if (input > max) {
            return max;
        } else {
            return Math.max(input, min);
        }
    }

    //TODO These methods help the robot to drive in different ways

    // drives the robot using simple mecanum equations
    public void driveMecanum(double ly, double lx, double rx) {

        double fl = ly - lx - rx;
        double fr = ly + lx + rx;
        double bl = ly + lx - rx;
        double br = ly - lx + rx;

        FL.setPower(fl);
        FR.setPower(fr);
        BL.setPower(bl);
        BR.setPower(br);
    }

    // drives the robot with respect to the field
    public void fieldCentricMecanum(double ly, double lx, double rx, double heading) {
        double rotX = lx * Math.cos(-heading) - ly * Math.sin(-heading);
        double rotY = lx * Math.sin(-heading) + ly * Math.cos(-heading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double FLPower = (rotY + rotX + rx) / denominator;
        double BLPower = (rotY - rotX + rx) / denominator;
        double FRPower = (rotY - rotX - rx) / denominator;
        double BRPower = (rotY + rotX - rx) / denominator;

        FL.setPower(FLPower);
        BL.setPower(BLPower);
        FR.setPower(FRPower);
        BR.setPower(BRPower);
    }

    // resets all wheel encoders and changes modes
    public void resetEncoders() {
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    // Refresh all motors & encoders

    public void refresh(){
        setAllPower(0);
        resetEncoders();
    }


}
