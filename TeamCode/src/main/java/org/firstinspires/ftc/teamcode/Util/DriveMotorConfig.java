package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.function.DoubleUnaryOperator;

/**
 * Holds 4 <code>double</code> values, one for each drive motor
 */
public class DriveMotorConfig {

    public double rightFrontPower;
    public double leftFrontPower;
    public double rightBackPower;
    public double leftBackPower;

    public DriveMotorConfig(
            double frontRight,
            double frontLeft,
            double backRight,
            double backLeft
    ) {
        this.rightFrontPower = frontRight;
        this.leftFrontPower = frontLeft;
        this.rightBackPower = backRight;
        this.leftBackPower = backLeft;
    }

    /**
     * Sets a specified power to each motor.
     */
    public static DriveMotorConfig splat(double power) {
        return new DriveMotorConfig(power, power, power, power);
    }

    /**
     * Applies power to each of the motors.
     */
    public void applyPowerTo(
            DcMotor frontRightMotor,
            DcMotor frontLeftMotor,
            DcMotor backRightMotor,
            DcMotor backLeftMotor
    ) {
        frontRightMotor.setPower(rightFrontPower);
        frontLeftMotor.setPower(leftFrontPower);
        backRightMotor.setPower(rightBackPower);
        backLeftMotor.setPower(leftBackPower);
    }

    /**
     * Applies a function to each of the motor powers.
     */
    public DriveMotorConfig map(DoubleUnaryOperator f) {
        rightBackPower = f.applyAsDouble(rightBackPower);
        leftBackPower = f.applyAsDouble(leftBackPower);
        rightFrontPower = f.applyAsDouble(rightFrontPower);
        leftFrontPower = f.applyAsDouble(leftFrontPower);
        return this;
    }

}
