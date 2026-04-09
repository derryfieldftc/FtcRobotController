package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TiltRobot extends Robot {
    private HardwareMap hardwareMap;

//    private DcMotor driveBL;
//    private DcMotor driveBR;
//    private DcMotor driveFL;
//    private DcMotor driveFR;
    private DcMotor liftMotor;
    private DcMotor tiltMotor;
    private RevTouchSensor tiltLimit;

    public TiltRobot(HardwareMap hardwareMap){
        super(hardwareMap);
        // NOTE: Commented out is from Robot.java.
//        this.hardwareMap = hardwareMap;
//        driveBL = this.hardwareMap.get(DcMotor.class, "motorBL");
//        driveBR = this.hardwareMap.get(DcMotor.class, "motorBR");
//        driveFL = this.hardwareMap.get(DcMotor.class, "motorFL");
//        driveFR = this.hardwareMap.get(DcMotor.class, "motorFR");
        liftMotor = this.hardwareMap.get(DcMotor.class, "lift");
        tiltMotor = this.hardwareMap.get(DcMotor.class, "tilt");
        tiltLimit = this.hardwareMap.get(RevTouchSensor.class, "tiltLimit");

//        driveFL.setDirection(DcMotor.Direction.REVERSE);
//        driveBL.setDirection(DcMotor.Direction.REVERSE);
//        driveFR.setDirection(DcMotor.Direction.FORWARD);
//        driveBR.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        tiltMotor.setDirection(DcMotor.Direction.FORWARD);

//        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }
    public void lift(boolean up, boolean down) {
        double liftMotorPower = 0;
        if (up) {
            liftMotorPower += 1;
        }
        if (down) {
            liftMotorPower += -1;
        }
        liftMotor.setPower(liftMotorPower);
    }
    public void tilt(boolean up, boolean down, boolean limit) {
        double tiltMotorPower = 0;
        if (limit) {
            tiltMotorPower = 0;
        }else if (up) {
            tiltMotorPower += 0.25;
        }
        if (down) {
            tiltMotorPower += -0.25;
        }

        tiltMotor.setPower(tiltMotorPower);
    }
}
