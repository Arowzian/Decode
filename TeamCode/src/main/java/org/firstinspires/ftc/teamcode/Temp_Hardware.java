package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.helpers.PIDHelper;


public class Temp_Hardware {

    public DcMotor FL;
    public DcMotor FR;
    public DcMotor BL;
    public DcMotor BR;

    public DcMotor deli;

    public CRServo rightServo;
    public CRServo leftServo;

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

    GoBildaPinpointDriver pinpoint;


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

        FL = initMotor("FL", true);    // (Port 0) = false)
        FR = initMotor("FR", false);   // (Port 1) = true)
        BL = initMotor("BL", true);    // (Port 2) = false)
        BR = initMotor("BR", false); // (Port 3) = true)
        deli = initMotor("deli", false); // (Port 3) = true)


//        rightServo = i.get(CRServo.class, "leftServo"); // 2024
        leftServo = i.get(CRServo.class, "rightServo"); // 2024

        pinpoint = i.get(GoBildaPinpointDriver.class, "pinpoint");


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


    public double autoYawTrim(double p, double i, double d, double current, double tgt) {
        double current_error = angleWrap(tgt - current);

        double P = p * current_error;

        double D = d * (current_error - previous_error);

        double output = P + D;

        previous_error = current_error;
        return output;
    }

    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

//    double stupidHeading;

//    public double FML(double fakeahHeading){
//
//        // Fake 160
//        // Stored 160
//        // Real 0
//
//        // Fake 170
//        // Stored 160
//        // Real 10
//
//        // Fake -170
//        // Real 30
//        // Stored 130
//        // Real 30
//
//        if(fakeahHeading > 0){
//            return fakeahHeading - stupidHeading;
//        }
//
//        //AOWJFOHAWFOUAWFOJAWFJIEAWFW
//
//
//
//
//
//        return fixedHeading;
//    }

    // returns the hardware map
    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

}
