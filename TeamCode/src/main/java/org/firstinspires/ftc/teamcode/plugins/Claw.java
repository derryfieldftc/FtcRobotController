package org.firstinspires.ftc.teamcode.plugins;
import android.graphics.Path;

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

    public class config {

        public config() {}
        /**
         * left claw servo
         * @param name the servo name
         * @param direction the servo direction (Servo.Direction.REVERSED or Servo.Direction.FORWARD)
         * @param openPosition the servo position when the claw is open
         * @param closePosition the servo position when the claw is closed
         */
        public void leftServo(String name, Servo.Direction direction, double openPosition, double closePosition) {
            leftServo = hardwareMap.servo.get(name);
            leftServo.setDirection(direction);
        }

        /**
         * right claw servo
         * @param name the servo name
         * @param direction the servo direction (Servo.Direction.REVERSED or Servo.Direction.FORWARD)
         * @param openPosition the servo position when the claw is open
         * @param closePosition the servo position when the claw is closed
         */
        public void rightServo(String name, Servo.Direction direction, double openPosition, double closePosition) {
            rightServo = hardwareMap.servo.get(name);
            rightServo.setDirection(direction);
        }

    }

    public void init() {

    }

    public void init_loop() {}

    public void start() {}

    public void loop() {}

    public void stop() {}


}
