package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.function.DoubleUnaryOperator;

/**
 * Holds 4 <code>double</code> values, one for each drive motor
 */
public class DriveMotorConfig {

    double frontRight;
    double frontLeft;
    double backRight;
    double backLeft;

    public DriveMotorConfig(
            double frontRight,
            double frontLeft,
            double backRight,
            double backLeft
    ) {
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        this.backRight = backRight;
        this.backLeft = backLeft;
    }

    public static DriveMotorConfig splat(double power) {
        return new DriveMotorConfig(power, power, power, power);
    }

    public void applyTo(
            DcMotor frontRightMotor,
            DcMotor frontLeftMotor,
            DcMotor backRightMotor,
            DcMotor backLeftMotor
    ) {
        frontRightMotor.setPower(frontRight);
        frontLeftMotor.setPower(frontLeft);
        backRightMotor.setPower(backRight);
        backLeftMotor.setPower(backLeft);
    }

    public DriveMotorConfig map(DoubleUnaryOperator f) {
        backRight = f.applyAsDouble(backRight);
        backLeft = f.applyAsDouble(backLeft);
        frontRight = f.applyAsDouble(frontRight);
        frontLeft = f.applyAsDouble(frontLeft);
        return this;
    }

}
