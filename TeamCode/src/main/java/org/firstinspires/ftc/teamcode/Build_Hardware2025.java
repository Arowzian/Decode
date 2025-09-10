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

    // returns a 2d array of coordinates of the april tags on backboards
    public double[] getCoords(int id) {
        // These are the coordinates of where the IDs are measured in inches
        // The coordinates are measured from the Red side opposite the scoring area
        // Coordinates are (X,Y)
        int[] coordIDs = {7, 8, 9, 10, 1, 2, 3, 4, 5, 6};
        double[][] coords = {
                {144, 114},
                {144, 109},
                {144, 35},
                {144, 30},
                {9, 29},
                {9, 35},
                {9, 41},
                {9, 103},
                {9, 109},
                {9, 115}
        };

        for (int i = 0; i < coordIDs.length; i++) {
            if (coordIDs[i] == id) {
                return coords[i];
            }
        }
        return null;
    }

    // returns the average position of the wheel encoders
    public double getAveragePosition() {
        if (FL.getCurrentPosition() < 0 && BL.getCurrentPosition() < 0 && FR.getCurrentPosition() < 0 && BR.getCurrentPosition() < 0) {
            return (FL.getCurrentPosition() + FR.getCurrentPosition() + BL.getCurrentPosition() + BR.getCurrentPosition()) / 4.0;
        }
        return (Math.abs(FL.getCurrentPosition()) + Math.abs(FR.getCurrentPosition()) + Math.abs(BL.getCurrentPosition()) + Math.abs(BR.getCurrentPosition())) / 4.0;
    }

    // returns wheel speed in encoder counts per millisecond
    public double getSpeed(double time1, double time2, int position1, int position2) {
        return (position2 - position1) / (time2 - time1);
    }

    // returns the hardware map
    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }


    //TODO These are methods that help with turning

    public boolean autonTurn(double target){
        /*if(Math.abs(angleWrap(Math.toRadians(checkOrientation().thirdAngle)) - target) < 0.5){
            return true;
        }
        else{
            driveMecanum(0,0, autoYawTrim(0.25,0.01,0.01, Math.toRadians(checkOrientation().thirdAngle), target));
            return false;
        }*/
        if(target > 0){
            if(checkOrientation().thirdAngle < target - 5){
                driveMecanum(0, 0, -0.4);
            }
            else return true;
        }
        if(target < 0){
            if(checkOrientation().thirdAngle > target + 5){
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
        I += i * (current_error * (current_time - previous_time));

        double D = d * (current_error - previous_error) / (current_time - previous_time);

        double output = P + I + D;

        previous_error = current_error;
        previous_time = current_time;

        return output;
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
        BR.setPower(br);
    }

    public boolean autonDrive(double power, int target) {
        autonAngleCheck();
        double correction = autonPID.performPID(getAngle());
        if(target < 0){
            if (Math.abs(getAveragePosition()) < target) {
                setAllPower(0);
                return true;
            } else {
                driveMecanum(-power, 0, -correction);
                return false;
            }
        }
        else{
            if (Math.abs(getAveragePosition()) > target) {
                setAllPower(0);
                return true;
            } else {
                driveMecanum(power, 0, -correction);
                return false;
            }
        }
    }

    public void autonDrive(double power) {
        autonAngleCheck();
        double correction = autonPID.performPID(getAngle());

        driveMecanum(power, 0, -correction);

    }

    public boolean autonStrafe(double power, int target) {
        autonAngleCheck();
        double correction = autonPID.performPID(getAngle());
        if (Math.abs(getAveragePosition()) > target) {
            setAllPower(0);
            return true;
        } else {
            strafeWithPower(power, correction);
            return false;
        }
    }

    public void autonStrafe(double power) {
        autonAngleCheck();
        double correction = autonPID.performPID(getAngle());
        strafeWithPower(power, correction);
    }


    public boolean fixTurnProblem() {
        if (Math.abs(checkOrientation().thirdAngle) < 3) return true;
        else {
            double correction = autonPID.performPID(getAngle());
            driveMecanum(0.08, 0, -correction);
            return false;
        }
    }

    public boolean fixDriveProblem(double power, boolean isRed, boolean isBoard) {
        if (Math.abs(checkOrientation().thirdAngle) < 3) return true;
        else {
            double correction = autonPID.performPID(getAngle());
            if((isRed && !isBoard) || (!isRed && isBoard)){
                driveMecanum(0, power, -correction);
            }
            else{
                driveMecanum(0, -power, -correction);
            }
            return false;
        }
    }

    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public void autonAngleCheck(){
        if(checkOrientation().thirdAngle < -70){
            autonPID.setSetpoint(-90);
        }
        else if(checkOrientation().thirdAngle > 70){
            autonPID.setSetpoint(90);
        }
        else{
            autonPID.setSetpoint(0);
        }
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

    // drives the robot to the specified encoder position
    public boolean runUsingEncoders(int distance, double power) {
        double p = 0.01;
        double i = 0.05;
        double d = 0.5;

        double current_time = System.currentTimeMillis();
        double current_error = distance - getAveragePosition();

        double P = p * current_error;
        IEncoder += i * (current_error * (current_time - previous_timeEncoder));

        double D = d * (current_error - previous_errorEncoder) / (current_time - previous_timeEncoder);

        double output = P + I + D;

        previous_errorEncoder = current_error;
        previous_timeEncoder = current_time;
        previousOutput = output;

        output = limitToPoles(output, power, -power);
        setAllPower(output);


        return ((output < 0.01) && (output > -0.01)) && ((previousOutput < 0.01) && (previousOutput > -0.01));
    }


    // sets all wheel power to inputed power
    public void setAllPower(double power) {
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);
    }

    public void strafeWithPower(double power) {
        FL.setPower(power);
        FR.setPower(-power);
        BL.setPower(-power);
        BR.setPower(power);
    }

    public void strafeWithPower(double power, double correction) {
        FL.setPower(power + correction);
        FR.setPower(-power - correction);
        BL.setPower(-power + correction);
        BR.setPower(power - correction);
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

    public void reverseMotors() {
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /*
    public void runUsingProfile(double max_acceleration, double max_velocity, double distance, double elapsed_time){
            while(Math.abs(distance - Math.abs(getAveragePosition())) != 0){
                double instantTarget = motionProfile(max_acceleration, max_velocity, distance, elapsed_time);
                setAllPower((instantTarget - getAveragePosition()) * 0.3);
            }
    }

    public double motionProfile(double max_acceleration, double max_velocity, double distance, double elapsed_time) {

        // Calculate the time it takes to accelerate to max velocity
        double acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = distance / 2;
        double acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        if (acceleration_distance > halfway_distance) {
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));
        }

        acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        double deceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        double cruise_distance = distance - 2 * acceleration_distance;
        double cruise_dt = cruise_distance / max_velocity;
        double deceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
        if (elapsed_time > entire_dt) {
            return distance;
        }

        // if we're accelerating
        if (elapsed_time < acceleration_dt) {
            // use the kinematic equation for acceleration
            return 0.5 * max_acceleration * Math.pow(elapsed_time, 2);
        }

        // if we're cruising
        else if (elapsed_time < deceleration_time) {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            double cruise_current_dt = elapsed_time - acceleration_dt;

            // use the kinematic equation for constant velocity
            return acceleration_distance + max_velocity * cruise_current_dt;
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            cruise_distance = max_velocity * cruise_dt;
            deceleration_time = elapsed_time - deceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * Math.pow(deceleration_time, 2);
        }
    }
*/
    public void moveToColor(double power, int zone, int maxEncoders) {
        if (zone == 2) {
            while (colorSensor.red() < 1000 && colorSensor.blue() < 1300 && getAveragePosition() < maxEncoders) {
                setAllPower(power);
            }
            finalEncoder = (int) getAveragePosition();
        } else if (zone == 1) {
            while (colorSensor.red() < 1000 && colorSensor.blue() < 1300 && getAveragePosition() < maxEncoders) {
                strafeWithPower(-power - 0.1);
            }
            finalEncoder = (int) getAveragePosition();

        } else {
            while (colorSensor.red() < 1000 && colorSensor.blue() < 1300 && getAveragePosition() < maxEncoders) {
                strafeWithPower(power + 0.1);
            }
            finalEncoder = (int) getAveragePosition();
        }
    }

    //public void moveToColor(double power, int zone, int maxEncoders, int averageColor, boolean isRed) {
        /*if(isRed){
            if (zone == 2) {
                while (averageColor < colorSensor.red() + 8 && getAveragePosition() < maxEncoders) {
                    setAllPower(power);
                }
                finalEncoder = (int) getAveragePosition();
            } else if (zone == 1) {
                while (averageColor < colorSensor.red() + 8 && getAveragePosition() < maxEncoders) {
                    strafeWithPower(-power);
                }
                finalEncoder = (int) getAveragePosition();

            } else {
                while (averageColor < colorSensor.red() + 8 && getAveragePosition() < maxEncoders) {
                    strafeWithPower(power);
                }
                finalEncoder = (int) getAveragePosition();
            }
        }
        else {
            if (zone == 2) {
                while (averageColor < colorSensor.blue() + 8 && getAveragePosition() < maxEncoders) {
                    setAllPower(power);
                }
                finalEncoder = (int) getAveragePosition();
            } else if (zone == 1) {
                while (averageColor < colorSensor.blue() + 8 && getAveragePosition() < maxEncoders) {
                    strafeWithPower(-power);
                }
                finalEncoder = (int) getAveragePosition();

            } else {
                while (averageColor < colorSensor.blue() + 8 && getAveragePosition() < maxEncoders) {
                    strafeWithPower(power);
                }
                finalEncoder = (int) getAveragePosition();
            }
        }
        if(isRed){
            if (zone == 2) {
                while (averageColor < colorSensor.red() + 8 && getAveragePosition() < maxEncoders) {
                    setAllPower(power);
                }
                finalEncoder = (int) getAveragePosition();
            } else if (zone == 1) {
                while (averageColor < colorSensor.red() + 8 && getAveragePosition() < maxEncoders) {
                    strafeWithPower(-power);
                }
                finalEncoder = (int) getAveragePosition();

            } else {
                while (averageColor < colorSensor.red() + 8 && getAveragePosition() < maxEncoders) {
                    strafeWithPower(power);
                }
                finalEncoder = (int) getAveragePosition();
            }
        }
        else {
            if (zone == 2) {
                while (averageColor < colorSensor.blue() + 4 && getAveragePosition() < maxEncoders) {
                    setAllPower(power);
                }
                finalEncoder = (int) getAveragePosition();
            } else if (zone == 1) {
                while (averageColor < colorSensor.blue() + 4 && getAveragePosition() < maxEncoders) {
                    strafeWithPower(-power);
                }
                finalEncoder = (int) getAveragePosition();

            } else {
                while (averageColor < colorSensor.blue() + 4 && getAveragePosition() < maxEncoders) {
                    strafeWithPower(power);
                }
                finalEncoder = (int) getAveragePosition();
            }
        }
    }*/

    public void moveToCenter(double power, int zone) {
        if (zone == 2) {
            while (getAveragePosition() > -finalEncoder) {
                setAllPower(-0.3);
            }
        } else if (zone == 1) {
            while (getAveragePosition() < finalEncoder) {
                strafeWithPower(power);
            }
        } else {
            while (getAveragePosition() < finalEncoder) {
                strafeWithPower(-power);
            }
        }
    }


    //TODO These methods all help with moving the lift

    // runs both lift motors at inputed power
    public void runLift(double power) {
        liftR.setPower(power);
        liftL.setPower(power);
    }

    // uses PIDDrive loop to input power for running the lift to inputed position
    public void PIDLift(int target) {
        double current_timeLift = System.currentTimeMillis();
        double current_errorLift = target - liftR.getCurrentPosition();

        double P = 0.0013 * current_errorLift;
        ILift += 0.005 * (current_errorLift * (current_timeLift - previous_timeLift));

        double D = 0 * (current_errorLift - previous_errorLift) / (current_timeLift - previous_timeLift);


        double liftPower = P + I + D;

        previous_errorLift = current_errorLift;
        previous_timeLift = current_timeLift;
        liftPower = limitToPoles(liftPower, 1, -1);

        runLift(liftPower);
    }

    public double accelerationLimiter(double previous_power, double current_power, double max) {
        double error = current_power - previous_power;
        boolean isNegative = (error < 0);
        error = Math.abs(error);

        if (error > max) {
            if (isNegative) {
                return -error;
            } else {
                return error;
            }
        } else {
            return current_power;
        }


    }



    //PREVVARS CODE: lx, ly, previous Strafe error, previous Fore error, previous turn error
    public double[] alignToTag(AprilTagProcessor tagProcessor, int targetID, double[] prevVars) {
        double ly = 0.0;
        double lx = 0.0;
        double rx = 0.0;
        double max = 0.5;

        double current_errorFore = 0.0;
        double current_errorStrafe = 0.0;
        double current_errorTurn = 0.0;

        if(tagProcessor.getDetections().size() > 0) {
            for (AprilTagDetection tag : tagProcessor.getDetections()) {
                if (tag.id == targetID) {
                    double current_time = System.currentTimeMillis();

                    current_errorTurn = angleWrap(0 - Math.toRadians(tag.ftcPose.yaw));

                    double P = 0.5 * current_errorTurn;

                    double D = 0.005 * (current_errorTurn - prevVars[4]) / (current_time - previous_time);

                    rx = P + D;

                    rx = limitToPoles(rx,max,-max);

                    rx *= -1;

                    current_errorStrafe = 0 - tag.ftcPose.x;

                    P = 0.09 * current_errorStrafe;

                    D = 0.01 * (current_errorStrafe - prevVars[2]) / (current_time - previous_time);

                    lx = P + D;

                    lx = -lx;

                    lx = limitToPoles(lx, max, -max);

                    lx = accelerationLimiter(prevVars[0], lx, 0.05);

                    /*

                    current_errorFore = 5 - tag.ftcPose.range;

                    P = 0.09 * current_errorFore;

                    D = 0.15 * (current_errorFore - prevVars[3]) / (current_time - previous_time);

                    ly = P + D;

                    ly = limitToPoles(ly, max * 0.75, -max * 0.75);

                    ly = accelerationLimiter(prevVars[0], ly, 0.05); */

                    previous_time = current_time;
                }
            }
        }

        driveMecanum(ly,lx,rx);


        return new double[]{lx,ly,current_errorStrafe,current_errorFore,current_errorTurn};
    }
/*
    public boolean openIris(){
        if(color2.red() + color2.blue() + color2.green() > 250) {
            iris.setPower(-0.3);
        }
        else{
            iris.setPower(0);
            return true;
        }
        return false;
    }*/

    public void refresh(){
        setAllPower(0);
        resetEncoders();
    }


}
