package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {

    private HardwareMap hardwareMap;

    private DcMotor driveBL;
    private DcMotor driveBR;
    private DcMotor driveFL;
    private DcMotor driveFR;
    //private DcMotor liftMotor;
    //private DcMotor tiltMotor;
    //private RevTouchSensor tiltLimit;

    public Robot(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        driveBL = this.hardwareMap.get(DcMotor.class, "motorBL");
        driveBR = this.hardwareMap.get(DcMotor.class, "motorBR");;
        driveFL = this.hardwareMap.get(DcMotor.class, "motorFL");;
        driveFR = this.hardwareMap.get(DcMotor.class, "motorFR");

        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveBL.setDirection(DcMotor.Direction.REVERSE);
        driveFR.setDirection(DcMotor.Direction.FORWARD);
        driveBR.setDirection(DcMotor.Direction.FORWARD);

        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void drive(double axial, double lateral, double yaw){
        double max;

        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial + lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial - lateral - yaw;
        double liftServoPower;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        driveBL.setPower(backLeftPower);
        driveBR.setPower(backRightPower);
        driveFL.setPower(frontLeftPower);
        driveFR.setPower(frontRightPower);
    }
}
