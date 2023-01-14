package org.firstinspires.ftc.teamcode;
import java.io.*;

public class EmergencyLogger
{
	public static final String LOGDIR = "/sdcard/FIRST/emergency_logs";
	private String logname;
	private FileWriter writer;

	public static void initLogDirectory()
	{
		File dir = new File(LOGDIR);
		if (!dir.exists())
			dir.mkdir();
	}

	public static void eWrite(String fmt, Object... args)
	{
		EmergencyLogger logger = new EmergencyLogger();
		logger.write(fmt, args);
		logger.close();
	}

	public EmergencyLogger(String logname)
	{
		this.logname = logname;
		try {
		this.writer = new FileWriter(LOGDIR + "/" + logname);
		} catch (Exception e) {
			System.err.println("Failed to open log file " + logname);
		}
	}

	public EmergencyLogger()
	{
		this(System.nanoTime() + ".log");
	}

	public boolean write(String fmt, Object... args)
	{
		String msg = String.format(fmt, args) + System.lineSeparator();
		try {
			this.writer.write(msg);
			return true;
		} catch (Exception e) {
			System.err.println("Failed to write msg to file: \n" + msg);
		}
		return false;
	}

	public boolean close()
	{
		try {
			this.writer.close();
			return true;
		} catch (Exception e) {
			;
		}
		return false;
	}
}
