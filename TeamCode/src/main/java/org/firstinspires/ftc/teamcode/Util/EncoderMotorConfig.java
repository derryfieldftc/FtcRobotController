package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.function.DoubleUnaryOperator;

public class EncoderMotorConfig {
    public double rightFrontTarget;
    public double leftFrontTarget;
    public double rightBackTarget;
    public double leftBackTarget;
    public double targetEncoderCount;

    public EncoderMotorConfig(
            double frontRightTarget,
            double frontLeftTarget,
            double backRightTarget,
            double backLeftTarget){
        this.rightFrontTarget = frontRightTarget;
        this.leftFrontTarget = frontLeftTarget;
        this.rightBackTarget = backRightTarget;
        this.leftBackTarget = backLeftTarget;
    }

    public EncoderMotorConfig(double targetEncoderCount){
        this.targetEncoderCount = targetEncoderCount;
    }

    /**
     * Sets target positions for each of the motor encoders.
     */
    public void addForwardTargetTo(
            DcMotor frontRightMotor,
            DcMotor frontLeftMotor,
            DcMotor backRightMotor,
            DcMotor backLeftMotor
    ) {
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + (int)targetEncoderCount);
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + (int)targetEncoderCount);
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + (int)targetEncoderCount);
        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + (int)targetEncoderCount);
    }

    public void addStrafeTargetTo(
            DcMotor frontRightMotor,
            DcMotor frontLeftMotor,
            DcMotor backRightMotor,
            DcMotor backLeftMotor
    ) {
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + (int)targetEncoderCount);
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() - (int)targetEncoderCount);
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + (int)targetEncoderCount);
        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() - (int)targetEncoderCount);
    }
}
