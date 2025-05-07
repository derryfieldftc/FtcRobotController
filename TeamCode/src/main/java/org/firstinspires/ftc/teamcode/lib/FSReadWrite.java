package org.firstinspires.ftc.teamcode.lib;

import static com.qualcomm.robotcore.util.ReadWriteFile.*;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Scanner;

public class FSReadWrite {
	static File file = AppUtil.getInstance().getSettingsFile("FSReadWriteInfo.txt");

	public static void store(String key, String data) {
		writeFile(file, key.length() + " " + key + " " + data, ReadWriteFile.readFile(file));
	}

	public static Object get(String key) {
		String data = readFile(file);
		return data;
	}

	public static class Serializer {
		public static String serialize(Object data) {
			return data.toString();
		}
		public static Object deSerialize(String key) {
			return key;
		}
	}

	static class IntSerializer extends Serializer {

		public static String serialize(Object data) {
			return String.valueOf((int)data);
		}

		public static Object deSerialize(String key) {
			return Integer.valueOf(key);
		}
	}
}
