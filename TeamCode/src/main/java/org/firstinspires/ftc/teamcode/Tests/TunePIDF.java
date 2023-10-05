package org.firstinspires.ftc.teamcode.Tests;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

@Autonomous(name="TunePIDF")
public class TunePIDF extends LinearOpMode {

        @SuppressLint("SdCardPath")
        public String fileName = "/sdcard/data/TunePIDF";
        public String fileExtension = ".csv";
        public FileWriter fileWriter;
        public PrintWriter printWriter;

        DcMotorEx lfMotor;
        DcMotorEx rfMotor;
        DcMotorEx lrMotor;
        DcMotorEx rrMotor;
        double currentVelocity;
        double lfMaxVelocity = 0.0;
        double rfMaxVelocity = 0.0;
        double lrMaxVelocity = 0.0;
        double rrMaxVelocity = 0.0;

        private String string(double num) {
            return Double.toString(num);
        }

        private void createFile() {
            try {
                File myObj = new File(fileName + fileExtension);
                int addNum = 1;
                while(myObj.exists() && !myObj.isDirectory()) {
                    //noinspection StringConcatenationInLoop
                    fileName += "(" + string(addNum) + ")";
                    addNum++;
                    myObj = new File(fileName + fileExtension);
                }
                if (myObj.createNewFile()) {
                    System.out.println("File created: " + myObj.getName());
                }
                else {
                    System.out.println("File already exists.");
                }
            }
            catch (IOException e) {
                System.out.println("An error occurred.");
                e.printStackTrace();
            }
        }

        public void author(String quote) {
            printWriter.print(quote);
        }

        @Override
        public void runOpMode() {
            lfMotor = hardwareMap.get(DcMotorEx.class, "LFMotor");
            lrMotor = hardwareMap.get(DcMotorEx.class, "LRMotor");
            rfMotor = hardwareMap.get(DcMotorEx.class, "RFMotor");
            rrMotor = hardwareMap.get(DcMotorEx.class, "RRMotor");

            lfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            lrMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            lfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            lfMotor.setVelocityPIDFCoefficients(1.19, 0.119, 0, 11.9);
            rfMotor.setVelocityPIDFCoefficients(1.20, 0.120, 0, 12.0);
            lrMotor.setVelocityPIDFCoefficients(1.22, 0.122, 0, 12.2);
            rrMotor.setVelocityPIDFCoefficients(1.21, 0.121, 0, 12.1);

            // Change these to optimize the robot
            double distance = 50;
            double positionPIDF = 4.0;
            double maxPower = 0.8;
            double power = 0.3;
            double powerIncrement = 0.01;

            double WheelCircumferenceInInches = (96 / 25.4) * Math.PI; // 10.0625

            lfMotor.setPositionPIDFCoefficients(positionPIDF);
            rfMotor.setPositionPIDFCoefficients(positionPIDF);
            lrMotor.setPositionPIDFCoefficients(positionPIDF);
            rrMotor.setPositionPIDFCoefficients(positionPIDF);

            waitForStart();

            double ticksPerMotorRev = 530.3;
            double ticksPerInch = ticksPerMotorRev/ WheelCircumferenceInInches;
            int distanceToTravel = (int) (distance * ticksPerInch);

            lfMotor.setTargetPosition(distanceToTravel);
            rfMotor.setTargetPosition(distanceToTravel);
            lrMotor.setTargetPosition(distanceToTravel);
            rrMotor.setTargetPosition(distanceToTravel);

            lfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (opModeIsActive()) {
                createFile();
                try {
                    fileWriter = new FileWriter(fileName + fileExtension);
                }
                catch (IOException e) {
                    e.printStackTrace();
                }
                printWriter = new PrintWriter(fileWriter);

                author("LF, RF, LR, RR\n");

                lfMotor.setPower(power);
                rfMotor.setPower(power);
                lrMotor.setPower(power);
                rrMotor.setPower(power);

                while (opModeIsActive() && lfMotor.isBusy() && rfMotor.isBusy() && lrMotor.isBusy() && rrMotor.isBusy()) {

                    if(power < maxPower) {
                        power += powerIncrement;
                    }

                    lfMotor.setPower(power);
                    rfMotor.setPower(power);
                    lrMotor.setPower(power);
                    rrMotor.setPower(power);

                    currentVelocity = lfMotor.getVelocity();
                    if (currentVelocity > lfMaxVelocity) {
                        lfMaxVelocity = currentVelocity;
                    }

                    currentVelocity = rfMotor.getVelocity();
                    if (currentVelocity > rfMaxVelocity) {
                        rfMaxVelocity = currentVelocity;
                    }
                    currentVelocity = lrMotor.getVelocity();
                    if (currentVelocity > lrMaxVelocity) {
                        lrMaxVelocity = currentVelocity;
                    }

                    currentVelocity = rrMotor.getVelocity();
                    if (currentVelocity > rrMaxVelocity) {
                        rrMaxVelocity = currentVelocity;
                    }

                    telemetry.addData("LF", "TPS (%.0f)", lfMaxVelocity);
                    System.out.printf("LF - TPS (%.0f)\n", lfMaxVelocity);
                    author(string(lfMaxVelocity) + ",");

                    telemetry.addData("RF", "TPS (%.0f)", rfMaxVelocity);
                    System.out.printf("RF - TPS (%.0f)\n", rfMaxVelocity);
                    author(string(rfMaxVelocity) + ",");

                    telemetry.addData("LR", "TPS (%.0f)", lrMaxVelocity);
                    System.out.printf("LR - TPS (%.0f)\n", lrMaxVelocity);
                    author(string(lrMaxVelocity) + ",");

                    telemetry.addData("RR", "TPS (%.0f)", rrMaxVelocity);
                    System.out.printf("RR - TPS (%.0f)\n", rrMaxVelocity);
                    author(string(rrMaxVelocity) + "\n");

                    telemetry.update();
                }
                lfMotor.setPower(0);
                rfMotor.setPower(0);
                lrMotor.setPower(0);
                rrMotor.setPower(0);

                System.out.println(power);
                System.out.println("Robot has stopped");
            }
        }
    }
