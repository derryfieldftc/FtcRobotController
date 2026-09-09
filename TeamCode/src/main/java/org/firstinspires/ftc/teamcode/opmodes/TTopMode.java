// This is an example OpMode. It shows the basic flow and function of an opMode, and should be used as a reference for new programmers.
// Every, and I mean every line of this file will be explained. Even if you have an understanding of java, please continue reading, as it is important we all follow a similar standard.

// This line declares what package (or folder) our code resides in.
// This is important to keep organized, all teleop opmodes should stay in this (opmodes) folder. All autonomous opmodes go into the autonomous folder, and all robot parts go in... you guessed it, the robot folder. There is also the pedro folder, which contains code for pedropathing, which is a library we use and will be touched upon later.
package org.firstinspires.ftc.teamcode.opmodes;

// These lines declare the imports this file has, in general we import most of the robot parts, along with things like telemetry, any helper functions, and pedro pathing things.
// Importing a file allows you to use code from that file in you project. Generally you will not write these imports yourself, but Android Studio will auto-magically import them for you.
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.autonmous.actions.TeleOpAction;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotPart;

// After the imports comes the actual code.

// This line is an annotation. Things that start with an `@` are annotations. Annotations convey something to the java compiler, in this case, it tells it that we want this file to be an OpMode.
// Forgetting this line stops the OpMode from appearing on the driver station. Another common place you will see annotations is @Configurable, which we will touch on later.
// Also important to note with annotations is they can receive arguments. In the case of @TeleOp, you can supply a name and group for it. The syntax for this is @TeleOp(name = "this is a name", group = "This is a group")
// Both the name and group arguments are not actually necessary, and I would argue that the name argument actively makes code worse, as without it the OpMode shares the name of the class, making it much easier to find.
@TeleOp

// This line declares the class. The name of this class always matches the name
// of the file.
// The extends keyword here means that this class is a sub-class of the OpMode
// class. This means it inherits its members (like hardwaremap), but also means
// that you have to implement some functions.
// The two main classes that you will extend is an OpMode, and a LinearOpMode.
// Most beginners extend LinearOpMode, as it is "simpler" I disagree with this
// notion. I believe that the iterative nature (loop function) of a normal
// OpMode is much better to learn, because it is most commonly used (an
// exception to this is an autonomous opmode, which can be whatever)
public class TTopMode extends OpMode {
	// Here is where we declare all of our member variables. It is convention to
	// leave variables at the top like this.
	// This is our robot, which is a container for all the subsystems, and a place
	// for functions that require multiple subsystems are held
	Robot robot;

	// This is used to show an example of using an action during teleop
	TeleOpAction doSomethingImportant;
	TeleOpAction intake;
	// This is a declaration of our motors. **DO NOT DO THIS**. The robot should
	// hold all the subsystems. This is just used in the example to make it not
	// useless
	DcMotor motorFR, motorFL, motorBR, motorBL;

	// We use a class called GamepadManager (cutesy of https://github.com/asmi57)
	// Its preference, but we use this cause it is more ergonomic.
	// If you decide not to use this, the gamepad class has methods like
	// gamepad.aWasPressed()
	GamepadManager mgamepad1;
	GamepadManager mgamepad2;

	@Override
	public void init() {
		robot = new Robot(this);
		doSomethingImportant = new TeleOpAction(() -> robot.doSomethingAction());


		//******Drive Motors*******
		motorFR = hardwareMap.dcMotor.get(RobotPart.Part.MotorFR.name);
		motorFL = hardwareMap.dcMotor.get(RobotPart.Part.MotorFL.name);
		motorBR = hardwareMap.dcMotor.get(RobotPart.Part.MotorBR.name);
		motorBL = hardwareMap.dcMotor.get(RobotPart.Part.MotorBL.name);

		motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
		motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
		//*************************


		mgamepad1 = new GamepadManager(gamepad1);
		mgamepad2 = new GamepadManager(gamepad2);
	}

	@Override
	public void loop() {
		boolean doingSomethingImportant = doSomethingImportant.run(); // Because this is a TeleOpAction we do not care
																		// about the result of run(); We use it here for
																		// telemetry though
		doSomethingImportant.run();
		if (mgamepad1.justPressed(GamepadManager.Button.A))
			doSomethingImportant.start();

		telemetry.addData("Doing something important", doingSomethingImportant);

		// Mecanum Logic
		double y = -gamepad1.left_stick_y;
		double x = gamepad1.left_stick_x;
		double rx = gamepad1.right_stick_x;
		double powerFL = y + x + rx;
		double powerBL = y - x + rx;
		double powerFR = y - x - rx;
		double powerBR = y + x - rx;

		double multiplier = 1 - gamepad1.left_trigger;

		motorFL.setPower(clamp(1, -1, powerFL) * multiplier);
		motorBL.setPower(clamp(1, -1, powerBL) * multiplier);
		motorFR.setPower(clamp(1, -1, powerFR) * multiplier);
		motorBR.setPower(clamp(1, -1, powerBR) * multiplier);
		// End of Mecanum logic
		robot.intake.setSpeed(gamepad1.right_trigger);
		telemetry.update(); // notice how this is ONLY called ONCE at the end of the whole block
		mgamepad1.poll(); // We also only poll the gamepadManager once per loop
	}

	private double clamp(double max, double min, double num) {
		if (num > max) {
			return max;
		} else if (num < min) {
			return min;
		} else {
			return num;
		}
	}
}
