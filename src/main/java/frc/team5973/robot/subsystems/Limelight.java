package frc.team5973.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.team5973.robot.RobotBase;

public class Limelight extends SubsystemBase {

	private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

	// Variables for calculations //TODO: need to find these once the cameras are mounted
	private double cameraHeight = 0;
	private double cameraAngle = 0;
	private double targetHeight = 0;

	public Limelight(RobotBase robot) {

		super(robot);

	}

	public boolean isTargetValid() {

		double tv = table.getEntry("tv").getDouble(0);

		if (tv == 0) return false;
		else return true;

	}

	public double getOffsetX() {

		return table.getEntry("tx").getDouble(0);

	}

	public double getOffsetY() {

		return table.getEntry("ty").getDouble(0);

	}

	public double getTargetArea() {

		return table.getEntry("ta").getDouble(0);

	}

	public double getSkew() {

		return table.getEntry("ts").getDouble(0);

	}

	public int getPipe() {

		return (int) table.getEntry("getpipe").getDouble(0);

	}

	public void setPipeline(int index) {

		table.getEntry("pipeline").setNumber(index);

	}

	public void setCameraHeight(double cameraHeight) {

		this.cameraHeight = cameraHeight;

	}

	public void setCameraAngle(double cameraAngle) {

		this.cameraAngle = cameraAngle;

	}

	public void setTargetHeight(double targetHeight) {

		this.targetHeight = targetHeight;

	}

	public double getDistance() {

		return (targetHeight - cameraHeight) / Math.tan(Math.toRadians(cameraAngle) + Math.toRadians(getOffsetY()));

	}

	double integralX = 0;
	public double limelightXPID() {
		// double kP = 0.008;
		// double correctionMin = 0.003;
		// double deadzone = 0.05;
		// double correction = getOffsetX() * kP;

		// if(correction < correctionMin)
		// 	correction = Math.copySign(correctionMin, correction);

		// if(Math.abs(getOffsetX()) < deadzone)
		// 	correction = 0;
		
		// return correction;

		double kP = 0.01;
		double deadzone = 0.05;
		double correction = getOffsetX() * kP;

		if(Math.abs(getOffsetX()) < deadzone)
			correction = 0;

		return correction;

		// double kP = 0.008;
		// double deadzone = 0.05;
		// double kI = 0.001;
		// double IZone = 4.0;
		// double ILimit = 1000;

		// if(Math.abs(getOffsetX()) < IZone)
		// 	integralX += getOffsetX();

		// if(Math.abs(integralX) > ILimit)
		// 	integralX = 600 * (integralX / Math.abs(integralX));

		// double correction = getOffsetX() * kP + integralX * kI;

		// if(Math.abs(getOffsetX()) < deadzone)
		// 	correction = 0;
		
		// return correction;

	}

	double integralY = 0;
	public double limelightYPID() {
		// double kP = 0.008;
		// double correctionMin = 0.003;
		// double deadzone = 0.05;
		// double correction = getOffsetY() * kP;

		// if(correction < correctionMin)
		// 	correction = Math.copySign(correctionMin, correction);

		// if(Math.abs(getOffsetY()) < deadzone)
		// 	correction = 0;
		
		// return correction;

		double kP = 0.05;
		double correction = getOffsetY() * kP;

		return correction;

		// double kP = 0.008;
		// double deadzone = 0.05;
		// double kI = 0.001;
		// double IZone = 4.0;
		// double ILimit = 1000;

		// if(Math.abs(getOffsetY()) < IZone)
		// 	integralY += getOffsetY();

		// if(Math.abs(integralY) > ILimit)
		// 	integralY = 600 * (integralY / Math.abs(integralY));

		// double correction = getOffsetY() * kP + integralY * kI;

		// if(Math.abs(getOffsetY()) < deadzone)
		// 	correction = 0;
		
		// return correction;

	}

	@Override
	public void initSendable(SendableBuilder builder) {

		super.initSendable(builder);

		builder.addBooleanProperty("Is Target Valid", () -> isTargetValid(), null);
		builder.addDoubleProperty("Offset X", () -> getOffsetX(), null);
		builder.addDoubleProperty("Offset Y", () -> getOffsetY(), null);
		builder.addDoubleProperty("Area", () -> getTargetArea(), null);
		builder.addDoubleProperty("Skew", () -> getSkew(), null);
		builder.addDoubleProperty("Pipeline", () -> getPipe(), null);
		builder.addDoubleProperty("Distance", () -> getDistance(), null);

	}

}