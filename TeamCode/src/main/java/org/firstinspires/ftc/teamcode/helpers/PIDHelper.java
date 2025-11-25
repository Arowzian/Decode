package org.firstinspires.ftc.teamcode.helpers;

public class PIDHelper {
    private double Kp = 0; // Gain for proportional term
    private double Kd = 0; // Gain for derivative term

    /* Constructor!!!
    Constructs the object!!!
    Takes a double, Kp, the initial value for the proportional term's gain
    Takes another double, Kd, the initial value for the derivative term's gain
     */
    public PIDHelper (double Kp, double Kd){
        this.Kp = Kp;
        this.Kd = Kd;
    }
    double setpoint = 0; // Where we want to be
    double currentPosition; // Where we are
    double previousError = 0; // The previous error (from last calc tick)

    public double calcPID() {
        double currentError = setpoint - currentPosition; // The distance from where we want to be

        double p = Kp * currentError; // The P term
        double d = Kd * (previousError - currentError); // The D term

        previousError = currentError; // Now that we've got both of our terms, we can update the previous error to be whatever the error is now for the next tick

        return p + d; // Give 'em what they want!
    }
    public void setSetpoint(double setpoint){ this.setpoint = setpoint; } // This is how we tell the PIDHelper where we want to be

    public void setCurrentPosition(double currentPosition){ this.currentPosition=currentPosition; } // This is how we tell them where we currently are

}
