package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.DcMotor;

public class EncoderMotorConfig {
    public double frontRightTarget;
    public double frontLeftTarget;
    public double backRightTarget;
    public double backLeftTarget;

    public EncoderMotorConfig(
            double frontRightTarget,
            double frontLeftTarget,
            double backRightTarget,
            double backLeftTarget
    ) {
        this.frontRightTarget = frontRightTarget;
        this.frontLeftTarget = frontLeftTarget;
        this.backRightTarget = backRightTarget;
        this.backLeftTarget = backLeftTarget;
    }

    public EncoderMotorConfig(double allTargets) {
        this.frontRightTarget = allTargets;
        this.frontLeftTarget = allTargets;
        this.backRightTarget = allTargets;
        this.backLeftTarget = allTargets;
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

        fr.setTargetPosition(fr.getCurrentPosition() + (int) frontRightTarget);
        fl.setTargetPosition(fl.getCurrentPosition() + (int) frontLeftTarget);
        br.setTargetPosition(br.getCurrentPosition() + (int) backRightTarget);
        bl.setTargetPosition(bl.getCurrentPosition() + (int) backLeftTarget);
    }

    public void addStrafeTargetTo(
            DcMotor frontRightMotor,
            DcMotor frontLeftMotor,
            DcMotor backRightMotor,
            DcMotor backLeftMotor
    ) {
        DcMotor fr = frontRightMotor, fl = frontLeftMotor, br = backRightMotor, bl = backLeftMotor;
        fr.setTargetPosition(fr.getCurrentPosition() + (int) frontRightTarget);
        fl.setTargetPosition(fl.getCurrentPosition() - (int) frontLeftTarget);
        br.setTargetPosition(br.getCurrentPosition() + (int) backRightTarget);
        bl.setTargetPosition(bl.getCurrentPosition() - (int) backLeftTarget);
    }
}
