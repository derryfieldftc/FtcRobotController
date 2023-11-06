package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.function.DoubleUnaryOperator;

public class EncoderMotorConfig {
    public double rightFrontTarget;
    public double leftFrontTarget;
    public double rightBackTarget;
    public double leftBackTarget;

    public EncoderMotorConfig(
            double frontRightTarget,
            double frontLeftTarget,
            double backRightTarget,
            double backLeftTarget
    ) {
        this.rightFrontTarget = frontRightTarget;
        this.leftFrontTarget = frontLeftTarget;
        this.rightBackTarget = backRightTarget;
        this.leftBackTarget = backLeftTarget;
    }

    public EncoderMotorConfig(double allTargets) {
        this.rightFrontTarget = allTargets;
        this.leftFrontTarget = allTargets;
        this.rightBackTarget = allTargets;
        this.leftBackTarget = allTargets;
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
        DcMotor fr = frontRightMotor, fl = frontLeftMotor, br = backRightMotor, bl = backLeftMotor;

        fr.setTargetPosition(fr.getCurrentPosition() + (int)rightFrontTarget);
        fl.setTargetPosition(fl.getCurrentPosition() + (int)leftFrontTarget);
        br.setTargetPosition(br.getCurrentPosition() + (int)rightBackTarget);
        bl.setTargetPosition(bl.getCurrentPosition() + (int)leftBackTarget);
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
