package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.helpers.PIDHelper;


public class Build_Hardware_2025 {

    public DcMotor FL;
    public DcMotor FR;
    public DcMotor BL;
    public DcMotor BR;
    public Servo flap;

    public DcMotor rotatingArmMotor;
    public DcMotor extendingMotor;

    public CRServo intakeServo;

    public Servo clawServo;

    HardwareMap hardwareMap;

    BNO055IMU imu;

    public TouchSensor slideSW;

    double previousOutput;


    double previous_time;
    double previous_timeLift;
    double previous_timeEncoder;
    double IEncoder = 0.0;
    double ILift = 0.0;

    double previous_error;
    double previous_errorLift;
    double previous_errorEncoder;

    double globalAngle;

    int finalEncoder;

    Orientation lastAngles = new Orientation();

    PIDHelper autonPIDX;
    PIDHelper autonPIDY;
//    PIDHelper armPID = new PIDHelper(0.01, 0.003, 2);


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
    public void init (HardwareMap i){
        hardwareMap = i;

        FL = initMotor("FL", false);
        FR = initMotor("FR", false);
        BL = initMotor("BL", true);
        BR = initMotor("BR", false);
        rotatingArmMotor = initMotor("rotatingArmMotor", false); // 2024
        extendingMotor = initMotor("extendingMotor", false); // 2024

        intakeServo = i.get(CRServo.class, "intakeServo"); // 2024
        clawServo = i.get(Servo.class, "clawServo"); // 2024
        flap = i.get(Servo.class,"flap");


        slideSW = i.get(TouchSensor.class, "slideSW");

//        autonPIDX.setOutputRange(0, 0.4);
//        autonPIDY.setOutputRange(0, 0.4);

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

    public void setMotorDirections(DcMotor FL, DcMotor FR, DcMotor BL, DcMotor BR){
        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
    }


    //TODO These are get and check methods that return values

    // returns the hardware map
    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

//    public double getPIDValues(int pid){
//        if(pid == 0){
////            return autonPIDX.performPID(hardwareMap.get(SparkFunOTOS.class, "sensor_otos").getPosition().x);
//        } else if(pid == 1){
////            return autonPIDY.performPID(hardwareMap.get(SparkFunOTOS.class, "sensor_otos").getPosition().y);
//        } else {
//            return -1;
//        }
//
//    }


    //TODO These are methods that help with turning

    public boolean autonTurn(double target){
        /*if(Math.abs(angleWrap(Math.toRadians(checkOrientation().thirdAngle)) - target) < 0.5){
            return true;
        }
        else{
            driveMecanum(0,0, autoYawTrim(0.25,0.01,0.01, Math.toRadians(checkOrientation().thirdAngle), target));
            return false;
        }*/

        SparkFunOTOS.Pose2D pos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos").getPosition();

        if(target > 0){
            if(pos.h < target - 5){
                driveMecanum(0, 0, -0.4);
            }
            else return true;
        }
        if(target < 0){
            if(pos.h > target + 5){
                driveMecanum(0, 0, 0.4);
            }
            else return true;
        }
        return false;
    }
    // figures out the power at which the robot should turn
    public double autoYawTrim(double p, double i, double d, double current, double tgt) {
        double current_time = System.currentTimeMillis();
        double current_error = angleWrap(tgt - current);

        double P = p * current_error;
        double I = i * (current_error * (current_time - previous_time));

        double D = d * (current_error - previous_error) / (current_time - previous_time);

        double output = P + I + D;

        previous_error = current_error;
        previous_time = current_time;

        return limitToPoles(output, 0.6, -0.6);
    }

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

    // exits the turn if the power being given is too low
    private boolean cutoffTurn(double rx) {
        double cutoffPower = 0.04;

        if (rx > 0) {
            return (rx < cutoffPower);
        } else {
            return (rx > -cutoffPower);
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
        BR.setPower(-br);
    }


    // drives the robot using fancy trig wizardry and field centric code

    public void testFieldCentric(double x, double y, double rx, double heading){

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading); // Trig wizardry to have field centric movement
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        rotX *= -0.8;
        rotY *= -0.7;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1); // Trig wizardry to have field centric movement

        FL.setPower(limitToPoles((rotY + rotX + rx) / denominator, 0.85, -0.85));
        BL.setPower(limitToPoles((rotY - rotX + rx) / denominator, 0.85, -0.85));
        FR.setPower(limitToPoles(-(rotY - rotX - rx) / denominator, 0.85, -0.85));
        BR.setPower(limitToPoles((rotY + rotX - rx) / denominator, 0.85, -0.85));
    }



    public void autoDrive(double targetH, double heading, double x, double y){
//        double resultX = autonPIDX.performPID(x);
//        double resultY = autonPIDY.performPID(y);

//        driveMecanumFieldCentric(resultX, resultY, autoYawTrim(1.3,0.007,0.04, Math.toRadians(heading), Math.toRadians(targetH)), Math.toRadians(heading));
//        testFieldCentric(-resultX, -resultY, autoYawTrim(1.3,0.007,0.04, Math.toRadians(heading), Math.toRadians(targetH)), Math.toRadians(heading));
//        testFieldCentric(0, 0.5, autoYawTrim(1.3,0.007,0.04, Math.toRadians(heading), Math.toRadians(targetH)), Math.toRadians(heading));
    }

//    public void setPIDXTarget(double target){
//        autonPIDX.setSetpoint(target);
//    }

//    public void setPIDYTarget(double target){
//        autonPIDY.setSetpoint(target);
//    }


    // sets all wheel power to inputted power
    public void setAllPower(double power) {
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);
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


    //TODO These methods all help with moving the rotational arm


    public void driveArm(int target){
//        rotatingArmMotor.setPower(armPID.performPID(target));
    }

    public double getVoltage(){
        return hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

}
