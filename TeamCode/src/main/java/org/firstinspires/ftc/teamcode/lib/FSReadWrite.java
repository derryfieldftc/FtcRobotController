package org.firstinspires.ftc.teamcode.lib;

import static com.qualcomm.robotcore.util.ReadWriteFile.*;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

public class FSReadWrite {
	static File directory = AppUtil.getInstance().getSettingsFile("FSReadWrite");
	static File file = new File(directory, "data");
	static FileWriter writer;
	static Scanner reader;


	/**
	 * Do not use unless you know what you are doing
	 */
	public static void setUp() {
		try {
			file.createNewFile();
		} catch (IOException e) {
			throw new RuntimeException(e);
		}

		try {
			writer = new FileWriter(file);
			reader = new Scanner(file);
		} catch (IOException e) {
			throw new RuntimeException(e);
		}

	}

	public static void store(String key, String data) throws IOException {
		//writer.append(key.length() + " " + key + " " + data);
		writer.write(1);
		//writeFile(file, key.length() + " " + key + " " + data, ReadWriteFile.readFile(file));
	}

	public static Object get(String key) throws IOException {
		String data = null;
		while (reader.hasNextLine()) {
			data = reader.nextLine();
			Scanner dataScanner = new Scanner(data);
		}

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
