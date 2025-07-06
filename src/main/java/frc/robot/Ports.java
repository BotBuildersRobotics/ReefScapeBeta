package frc.robot;

import frc.robot.lib.CanDeviceId;

public class Ports {

    //each CAN device will have a name and a unique ID.
    //Right now each one has a random ID

    
    //Elevator Subsystem Motors
    public static final CanDeviceId ELEVATOR = new CanDeviceId(18, "canivore");
    
    //Intake Subsystem Motors
    public static final CanDeviceId INTAKE = new CanDeviceId(53, "canivore");
    
    public static final CanDeviceId INTAKE_1_CANRANGE = new CanDeviceId(44, "canivore"); 
   
    

	public static final int PIGEON = 13;

	public static final int INTAKE_BEAMBREAK_ONE = 0;
	public static final int INTAKE_BEAMBREAK_TWO = 2;
    
    public static final int ELEVATOR_BEAMBREAK = 4;

    // Wheel krakens
   /* public static final CanDeviceId TOPRIGHT_ROT = new CanDeviceId(1, "canivore");
    public static final CanDeviceId TOPRIGHT_DRIVE = new CanDeviceId(3, "canivore");
    public static final CanDeviceId BOTTOMRIGHT_ROT = new CanDeviceId(4, "canivore");
    public static final CanDeviceId BOTTOMRIGHT_DRIVE = new CanDeviceId(6, "canivore");
    public static final CanDeviceId BOTTOMLEFT_ROT = new CanDeviceId(7, "canivore");
    public static final CanDeviceId BOTTOMLEFT_DRIVE = new CanDeviceId(9, "canivore");
    public static final CanDeviceId TOPLEFT_ROT = new CanDeviceId(10, "canivore");
    public static final CanDeviceId TOPLEFT_DRIVE = new CanDeviceId(12, "canivore");

    // Wheel cancoder
    public static final CanDeviceId TOPRIGHT_ENCODER = new CanDeviceId(2, "canivore");
    public static final CanDeviceId BOTTOMRIGHT_ENCODER = new CanDeviceId(5, "canivore");
    public static final CanDeviceId BOTTOMLEFT_ENCODER = new CanDeviceId(8, "canivore");
    public static final CanDeviceId TOPLEFT_ENCODER = new CanDeviceId(11, "canivore");*/
}
