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

    public DcMotor FL;
    public DcMotor FR;
    public DcMotor BL;
    public DcMotor BR;

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

    GoBildaPinpointDriver pinpoint;

    ////////////////////////////
    /// BUTTON PRESSED ARRAY ///
    ////////////////////////////

    boolean[] buttonsToggled;

    private boolean a;
    private boolean b;
    private boolean x;
    private boolean y;

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

        FL = initMotor("FL", true, DcMotor.ZeroPowerBehavior.BRAKE);    // (Port 0) = false)
        FR = initMotor("FR", false, DcMotor.ZeroPowerBehavior.BRAKE);   // (Port 1) = true)
        BL = initMotor("BL", true, DcMotor.ZeroPowerBehavior.BRAKE);    // (Port 2) = false)
        BR = initMotor("BR", false, DcMotor.ZeroPowerBehavior.BRAKE); // (Port 3) = true)
        intakeMotor = initMotor("intakeMotor", false, DcMotor.ZeroPowerBehavior.FLOAT);
        indexMotor = initMotor("indexMotor", false, DcMotor.ZeroPowerBehavior.BRAKE);
        leftDeliveryMotor = initMotor("leftDeliveryMotor", true, DcMotor.ZeroPowerBehavior.FLOAT);
        rightDeliveryMotor = initMotor("rightDeliveryMotor", false, DcMotor.ZeroPowerBehavior.FLOAT);

        pinpoint = i.get(GoBildaPinpointDriver.class, "pinpoint");

        buttonsToggled = new boolean[]{false, false, false, false};
        a = false;
        b = false;
        x = false;
        y = false;

        previous_error = 0;
    }

    /// MOTOR INIT

    public DcMotor initMotor(String name, boolean isReverse, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        if (isReverse) {
            motor.setDirection(DcMotor.Direction.REVERSE);
        } else {
            motor.setDirection(DcMotor.Direction.FORWARD);
        }

        motor.setPower(0.0);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        return motor;
    }

    /////////////////////
    /// MOTOR HELPERS ///
    /////////////////////


    /// ROTATIONAL PID

    double previous_error = 0;

    /*
        This is just a PID loop used for rotational stuff
        Specifically, this PID is generally used for correcting drift when driving
     */
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

    //////////////////////
    /// BUTTON TOGGLES ///
    //////////////////////

    // This basically is used to toggle on and off the buttonsToggled array
    // to use this, just input gamepad1.a, gamepad1.b, gamepad1.x, and gamepad1.y as the arguments, and access buttonsToggled
    public void setButtonsToggled(boolean a, boolean b, boolean x, boolean y){
        if(!this.a && a){ // If a is pressed and it wasn't pressed previously
            buttonsToggled[0] = !buttonsToggled[0]; // Toggle a on or off
            this.a = true; // we can now say a was previously pressed
        } else if(this.a && !a) { this.a = false; } // If we think a was previously pressed, but it's no longer pressed, then a is no longer previously pressed

        if(!this.b && b){ // If b is pressed and it wasn't pressed previously
            buttonsToggled[1] = !buttonsToggled[1]; // Toggle b on or off
            this.b = true; // we can now say b was previously pressed
        } else if(this.b && !b) { this.b = false; } // If we think b was previously pressed, but it's no longer pressed, then b is no longer previously pressed

        if(!this.x && x){ // If x is pressed and it wasn't pressed previously
            buttonsToggled[2] = !buttonsToggled[2]; // Toggle x on or off
            this.x = true; // we can now say x was previously pressed
        } else if(this.x && !x) { this.x = false; } // If we think x was previously pressed, but it's no longer pressed, then x is no longer previously pressed

        if(!this.y && y){ // If y is pressed and it wasn't pressed previously
            buttonsToggled[3] = !buttonsToggled[3]; // Toggle y on or off
            this.y = true; // we can now say y was previously pressed
        } else if(this.y && !y) { this.y = false; } // If we think y was previously pressed, but it's no longer pressed, then y is no longer previously pressed
    }
}