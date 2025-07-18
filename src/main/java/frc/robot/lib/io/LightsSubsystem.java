package frc.robot.lib.io;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.io.LightsIO;

public class LightsSubsystem<IO extends LightsIO> extends SubsystemBase {
	protected final IO io;
	protected final String name;

	public LightsSubsystem(String name, IO io) {
		super(name);
		this.io = io;
		this.name = name;
	}

	@Override
	public void periodic() {
		io.update();
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		io.initSendable(builder);
	}
}