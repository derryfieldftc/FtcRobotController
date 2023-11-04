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
    public void setTargetTo(
            DcMotor frontRightMotor,
            DcMotor frontLeftMotor,
            DcMotor backRightMotor,
            DcMotor backLeftMotor
    ) {
        frontRightMotor.setTargetPosition((int)rightFrontTarget);
        frontLeftMotor.setTargetPosition((int)leftFrontTarget);
        backRightMotor.setTargetPosition((int)rightBackTarget);
        backLeftMotor.setTargetPosition((int)leftBackTarget);
    }

    public EncoderMotorConfig updateTargets(){
        rightFrontTarget += targetEncoderCount;
        leftBackTarget += targetEncoderCount;
        rightBackTarget += targetEncoderCount;
        leftFrontTarget += targetEncoderCount;
        return this;
    }
}
