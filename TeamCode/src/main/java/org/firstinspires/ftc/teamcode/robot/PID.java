package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.RobotLog;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.charset.StandardCharsets;
import java.util.Scanner;

public class PID {
    // for now I'm making member variables (for convenience).
    public double kp, ki, kd;
    public double integral, previous_error;
    public double threshold;

    /**
     * Parameters for the PID controller, sane defaults are around .2 (generally)
     * set unneeded values to 0
     * @param kp proportional portion importance (coefficient)
     * @param ki integral portion importance (coefficient)
     * @param kd derivative imprtance (coefficient)
     * @param threshold Error threshold
     */
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

    public boolean close_enough(double error) {
        if (Math.abs(error) < threshold) {
            return true;
        } else {
            return false;
        }
    }

    public double calculate(double error, double dt) {
        if (close_enough(error)) {
            // reset values.
            integral = 0;
            previous_error = 0;

            // no correction needed since we're close enough.
            return 0;
        } else {
            // cumulative error
            integral += error * dt;

            // derivative is the rate of change of error.
            double derivative = (error - previous_error) / dt;
            previous_error = error;
            return kp * error + ki * integral + kd * derivative;
        }
    }
}