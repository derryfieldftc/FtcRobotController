package org.firstinspires.ftc.teamcode.plugins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotPlugin;

/**
 * Claw
 */
public class Claw extends RobotPlugin {

    HardwareMap hardwareMap;
    Servo rightServo;
    Servo leftServo;
    OpMode opMode;
    Telemetry telemetry;

    /**
     * Clas constructor, needs Claw(OpMode).Builder() to function (generally)
     * @param opMode
     */
    public Claw(OpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
    }

    public class Builder {

        String rightServoName;
        Servo.Direction rightServoDirection;
        double rightOpenPosition, rightClosePosition;
        String leftServoName;
        Servo.Direction leftServoDirection;
        double leftOpenPosition, leftClosePosition;

		public Builder() {
        }

        /**
         * left claw servo
         * @param name the servo name
         * @param direction the servo direction (Servo.Direction.REVERSED or Servo.Direction.FORWARD)
         * @param openPosition the servo position when the claw is open
         * @param closePosition the servo position when the claw is closed
         */
        public Builder rightServo(String name, Servo.Direction direction, double openPosition, double closePosition) {
            rightServoName = name;
            rightServoDirection = direction;
            rightOpenPosition = openPosition;
            return this;
        }

        /**
         * right claw servo
         * @param name the servo name
         * @param direction the servo direction (Servo.Direction.REVERSED or Servo.Direction.FORWARD)
         * @param openPosition the servo position when the claw is open
         * @param closePosition the servo position when the claw is closed
         */
        public Builder leftServo(String name, Servo.Direction direction, double openPosition, double closePosition) {
            leftServoName = name;
            leftServoDirection = direction;
            leftOpenPosition = openPosition;
            return this;
        }

        public Claw build(OpMode opMode) {
            Claw claw = new Claw(opMode);
            return null; // TODO: fix
        }

    }

    // Code is run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    public void init_loop() {}

    // Code is run ONCE when the driver hits PLAY
    public void start() {}

    // Code is run REPEATEDLY after the driver hits PLAY but before they hit STOP
    public void loop() {}

    // Code to run ONCE after the driver hits STOP
    public void stop() {}

}
