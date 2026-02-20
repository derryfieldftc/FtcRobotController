package org.firstinspires.ftc.teamcode.robot;

import static com.qualcomm.robotcore.util.RobotLog.d;
import static com.qualcomm.robotcore.util.RobotLog.w;

import com.bylazar.configurables.annotations.Configurable;

/**
 * A place to store all field related data, methods, and anything else
 */
public class Field {
	// This class contains some terrible code

	public static Obelisk.Motif motif = Obelisk.Motif.PGP; // default

	// I hate java so much, all I wanted was a struct
	public static class ColorSensorValue {
		public float[] purple;
		public float[] purpleStandardDev;
		public float[] green;
		public float[] greenStandardDev;
		public float[] none;
		public float[] noneStandardDev;

		/**
		 * Users should not manually construct this
		 * @param values
		 */
		public ColorSensorValue(float[][] values) {
			purple = values[0];
			purpleStandardDev = values[1];
			green = values[2];
			greenStandardDev = values[3];
			none = values[4];
			noneStandardDev = values[5];
		}
	}
	@Configurable
	public abstract static class ColorSensorValues {
		public static double NormalGreenValue = .5;
		public static int NormalPurpleHue = 155;
		public static double NormalNoneDistance = 4;

		abstract ColorSensorValue getValues();
		@Configurable
		public static class Zero extends ColorSensorValues {
			public static float[] Purple = {167.294F, 0.3593159F, 2.17007F};
			public static float[] PurpleStandardDev = {3F, 0.1F, 0.3F};
			public static float[] Green = {151.853F, 0.532245F, 2.47099F};
			public static float[] GreenStandardDev = {3F, 0.1F, 0.3F};
			public static float[] None = {153.347F, 0.468059F, 1.77127F};
			public static float[] NoneStandardDev = {3F, 0.1F, 0.3F};

			@Override
			public ColorSensorValue getValues() {
				return new ColorSensorValue(new float[][]{Purple, PurpleStandardDev, Green, GreenStandardDev, None, NoneStandardDev});
			}
		}
		@Configurable
		public static class One extends ColorSensorValues{
			public static float[] Purple = {225.2889038F, 0.5683059188F, 1.764237594F};
			public static float[] PurpleStandardDev = {11.56284251F, 0.05297879102F, 1.637031313F};
			public static float[] Green = {164.4971972F, 0.7140865378F, 1.419567729F};
			public static float[] GreenStandardDev = {6.262410882F, 0.05661928264F, 0.8646563786F};
			public static float[] None = {144.175044F, 0.3857241012F, 0.3136627566F};
			public static float[] NoneStandardDev = {4.090624246F, 0.02153544195F, 0.01629610523F};

			@Override
			ColorSensorValue getValues() {
				return new ColorSensorValue(new float[][]{Purple, PurpleStandardDev, Green, GreenStandardDev, None, NoneStandardDev});
			}
		}
		@Configurable
		public static class Two extends ColorSensorValues {
			public static float[] Purple = {220.7293108F, 0.5350395014F, 0.9359466158F};
			public static float[] PurpleStandardDev = {7.055582598F, 0.0391077265F, 0.9913694394F};
			public static float[] Green = {162.5407762F, 0.715224336F, 0.981909621F};
			public static float[] GreenStandardDev = {5.568532156F, 0.02193374589F, 0.3044184598F};
			public static float[] None = {136.966684F, 0.3739539013F, 0.2455866667F};
			public static float[] NoneStandardDev = {1.598480686F, 0.005626596963F, 0.0171454886F};

			@Override
			ColorSensorValue getValues() {
				return new ColorSensorValue(new float[][]{Purple, PurpleStandardDev, Green, GreenStandardDev, None, NoneStandardDev});
			}
		}
	}

	/**
	 * A standard way to represent the type of ball in any place, Unknown is used when it is known
	 * that there is a ball, but the color could not be determined
	 */
	public enum Ball {
		Green,
		Purple,
		Unknown,
		None;

		/**
		 * hsv is an array of length three, and colorSensor must be one of the color sensors
		 * @param hsv Hue, Saturation, Value
		 * @param colorSensorValues
		 */
		public static Ball getBallFromColor(float[] hsv, ColorSensorValues colorSensorValues, double distance) {
			ColorSensorValue csvn = colorSensorValues.getValues();

//			float purpleScore = getScore(hsv, csvn.purple, csvn.purpleStandardDev);
//			float greenScore  = getScore(hsv, csvn.green, csvn.greenStandardDev);
//			float noneScore   = getScore(hsv, csvn.none, csvn.noneStandardDev);
//
//			float min = Math.min(Math.min(purpleScore, greenScore), noneScore);
//
//			if (purpleScore == min) {
//				return Ball.Purple;
//			}
//			if (greenScore == min) {
//				return Ball.Green;
//			}
//			return Ball.None;
			d("AHM color " + colorSensorValues.getClass().toString().substring(67) + " " + hsv[0] + " " + hsv[1] + " " + hsv[2] + " d " + distance);

			if (colorSensorValues.getClass() != ColorSensorValues.One.class) {
				if (distance > ColorSensorValues.NormalNoneDistance) // If we dont have a ball
					return None;

				if (hsv[1] < ColorSensorValues.NormalGreenValue)
					return Green;

				if (hsv[0] > ColorSensorValues.NormalPurpleHue)
					return Purple;

				return None;
			} else {
				return None;
			}
		}

		private static float getScore(float[] hsv, float[] compare, float[] stddev) {
			float errorH = (hsv[0] - compare[0]) / stddev[0];
			float errorS = (hsv[1] - compare[1]) / stddev[1];
			float errorV = (hsv[2] - compare[2]) / stddev[2];
			return errorH + errorS + errorV;
		}
	}

	public enum Alliance {
		Red,
		Blue
	}

}