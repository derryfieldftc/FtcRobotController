package org.firstinspires.ftc.teamcode.boxingBot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class boxingBot {
    HardwareMap hardwareMap;

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;


    // constructor
    public boxingBot(HardwareMap hardwareMap) {
        // get reference to hardwaremap

        this.hardwareMap = hardwareMap;

        //use hardwaremap to get references to devices
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");

        //set directions
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        // set braking power behavior
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }
    //This method will make the robot drive forward
    public void drive(float drive, float strafe, float twist) {
        // set motor power for robot
        float powerFL = drive + strafe- twist;
        float powerFR = drive + strafe + twist;
        float powerBL = drive - strafe - twist;
        float powerBR = drive - strafe + twist;

        // sale the power values so they are all <= 1 in magnitude
        float scaler = Math.max(Math.abs(powerFL), Math.max(Math.abs(powerFR), Math.max(Math.abs(powerBL), Math.abs(powerBR))));

        if (scaler > 1) {
            powerFL = powerFL / scaler;
            powerFR = powerFR / scaler;
            powerBL = powerBL / scaler;
            powerBR = powerBR / scaler;

        }
        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);



    }
    public void strafe(float power){

        motorFL.setPower(power);
        motorFR.setPower(-power);
        motorBL.setPower(-power);
        motorBR.setPower(power);
    }

}