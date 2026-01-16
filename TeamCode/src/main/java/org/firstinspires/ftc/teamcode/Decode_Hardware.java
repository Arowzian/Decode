package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Decode_Hardware {

    ///////////////////////////
    /// INTEGRATED HARDWARE ///
    ///////////////////////////

    // Hardware Map //

    HardwareMap hardwareMap;

    // IMU //

    BNO055IMU imu;

    //////////////
    /// MOTORS ///
    //////////////

    // Drivetrain Motors //

//    public DcMotor FL;
//    public DcMotor FR;
//    public DcMotor BL;
//    public DcMotor BR;

    // Intake Motor //

    public DcMotor intakeMotor;

    // Index Motor //

    public DcMotor indexMotor;

    // Delivery Motors //

    public DcMotor leftDeliveryMotor;
    public DcMotor rightDeliveryMotor;

    ///////////////////
    /// I2C DEVICES ///
    ///////////////////

    // Pinpoint //

//    GoBildaPinpointDriver pinpoint;

    //////////////////////
    /// INITIALIZATION ///
    //////////////////////

    /// IMU

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

    /// MISC INIT

    public void init (HardwareMap i){
        hardwareMap = i;

//        FL = initMotor("FL", true);    // (Port 0) = false)
//        FR = initMotor("FR", false);   // (Port 1) = true)
//        BL = initMotor("BL", true);    // (Port 2) = false)
//        BR = initMotor("BR", false); // (Port 3) = true)
        intakeMotor = initMotor("intakeMotor", false);
        indexMotor = initMotor("indexMotor", false);
        leftDeliveryMotor = initMotor("leftDeliveryMotor", true);
        rightDeliveryMotor = initMotor("rightDeliveryMotor", false);

//        pinpoint = i.get(GoBildaPinpointDriver.class, "pinpoint");

        previous_error = 0;
    }

    /// MOTOR INIT

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

    /////////////////////
    /// MOTOR HELPERS ///
    /////////////////////


    /// ROTATIONAL PID

    double previous_error = 0;
    public double autoYawTrim(double p, double i, double d, double current, double tgt) {
        double current_error = angleWrap(tgt - current);

        double P = p * current_error;

        double D = d * (current_error - previous_error);

        double output = P + D;

        previous_error = current_error;
        return output;
    }

    /////////////////////
    /// ANGLE HELPERS ///
    /////////////////////

    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }
}
