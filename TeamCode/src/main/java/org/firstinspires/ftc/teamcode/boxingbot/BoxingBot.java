package org.firstinspires.ftc.teamcode.boxingbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BoxingBot {
    // member variables.
    HardwareMap hardwareMap;
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBR;
    DcMotor motorBL;

    // constructor.
    public BoxingBot(HardwareMap hardwareMap) {
        // get reference to hardware map.
        this.hardwareMap = hardwareMap;

        // use hardware map to get references to our devices.
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");

        // set directions.
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        // set zero power behavior.
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // this method will make the robot drive forward.
    public void drive(float drive, float strafe, float twist) {
        // set the motor power for the robot.
        float powerFL = drive + strafe - twist;
        float powerFR = drive - strafe + twist;
        float powerBR = drive + strafe + twist;
        float powerBL = drive - strafe - twist;

        // we have scale the power values so that they are all <= 1 in magnitude.
        float scalar = Math.max(Math.abs(powerFL), Math.max(Math.abs(powerFR),
                Math.max(Math.abs(powerBL), Math.abs(powerBR))));

        if (scalar > 1) {
            powerFL = powerFL / scalar;
            powerFR = powerFR / scalar;
            powerBR = powerBR / scalar;
            powerBL = powerBL / scalar;
        }

        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBR.setPower(powerBR);
        motorBL.setPower(powerBL);
    }
}
