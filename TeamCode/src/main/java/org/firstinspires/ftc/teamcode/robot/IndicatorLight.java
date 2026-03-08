package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Use this within other robot parts (dont question that its not a RobotPart
 * itself)
 */
public class IndicatorLight {
	Servo light; // Not a servo but the color is controlled via pwm and acts similar to a servo

	/**
	 * Color of led
	 */
	public enum Color {
		// Values found from https://www.gobilda.com/rgb-indicator-light-pwm-controlled/
		// color image thing
		Off(0),
		Red(0.280),
		Orange(0.333),
		Yellow(0.388),
		Sage(0.444),
		Green(0.500),
		Azure(0.555),
		Blue(0.611),
		Indigo(0.666),
		Violet(0.722),
		White(1);

		public final double pwm;

		Color(double pwm) {
			this.pwm = pwm;
		}
	}

	public IndicatorLight(Servo light) {
		this.light = light;
	}

	/**
	 * Sets color from a predetermined list
	 * 
	 * @param color
	 */
	public void setColor(Color color) {
		light.setPosition(color.pwm);
	}

	/**
	 * Set a raw pwm value for color, find colors at
	 * https://www.gobilda.com/rgb-indicator-light-pwm-controlled/
	 * 
	 * @param pwm
	 */
	public void setRawColor(double pwm) {
		light.setPosition(pwm);
	}
}
