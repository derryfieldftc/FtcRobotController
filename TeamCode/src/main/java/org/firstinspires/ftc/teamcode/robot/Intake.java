package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake extends RobotPart {
    DcMotor intake;
    protected double speed = 0;

    public Intake(OpMode opMode) {
        super(opMode);
        intake = hardwareMap.dcMotor.get(Part.intakeMotor.name);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public Intake setSpeed(double speed) {
        this.speed = speed;
        intake.setPower(speed);
        return this;
    }

    public double getSpeed() {
        return speed;
    }
}