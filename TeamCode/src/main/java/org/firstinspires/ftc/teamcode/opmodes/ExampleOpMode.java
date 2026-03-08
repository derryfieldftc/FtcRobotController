// This is an example OpMode. It shows the basic flow and function of an opMode, and should be used as a reference for new programmers.
// Every, and I mean every line of this file will be explained. Even if you have an understanding of java, please continue reading, as it is important we all follow a similar standard.

// This line declares what package (or folder) our code resides in.
// This is important to keep organized, all teleop opmodes should stay in this (opmodes) folder. All autonomous opmodes go into the autonomous folder, and all robot parts go in... you guessed it, the robot folder. There is also the pedro folder, which contains code for pedropathing, which is a library we use and will be touched upon later.
package org.firstinspires.ftc.teamcode.opmodes;

// These lines declare the imports this file has, in general we import most of the robot parts, along with things like telemetry, any helper functions, and pedro pathing things.
// Importing a file allows you to use code from that file in you project. Generally you will not write these imports yourself, but Android Studio will auto-magically import them for you.
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;

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
public class ExampleOpMode extends OpMode {
	Robot robot;

	@Override
	public void init() {

	}

	@Override
	public void loop() {

	}
}
