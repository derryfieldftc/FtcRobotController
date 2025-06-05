package org.firstinspires.ftc.teamcode.binarybot;

public class PID {
    // for now I'm making member variables (for convenience).
    public double kp, ki, kd;
    public double integral, previous_error;
    public double threshold;

    public PID(double kp, double ki, double kd, double threshold) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.threshold = threshold;
    }

    public void clear() {
        integral = 0;
        previous_error = 0;
    }

    public double calculate(double error, double dt) {
        if (error < threshold) {
            // reset values.
            integral = 0;
            previous_error = 0;
            return 0;

        } else {
            integral += error * dt;

            // derivative is the rate of change of error.
            double derivative = (error - previous_error) / dt;
            previous_error = error;
            return kp * error + ki * integral + kd * derivative;
        }
    }
}