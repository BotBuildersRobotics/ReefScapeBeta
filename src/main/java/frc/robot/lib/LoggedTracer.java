package frc.robot.lib;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Utility class for logging code execution times. */
public class LoggedTracer {
	private LoggedTracer() {}

	private static double startTime = -1.0;

	/** Reset the clock. */
	public static void reset() {
		startTime = Timer.getFPGATimestamp();
	}

	/** Save the time elapsed since the last reset or record. */
	public static void record(String epochName) {
		double now = Timer.getFPGATimestamp();
		SmartDashboard.putNumber(
				"Logged Tracer/" + epochName + " Milliseconds", Units.secondsToMilliseconds(now - startTime));
		startTime = now;
	}
}