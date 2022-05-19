package frc.team5973.robot.rapidreact;

import java.util.Map;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team5973.robot.RobotBase;
import frc.team5973.robot.XinputController;
import frc.team5973.robot.rapidreact.Intake.IntakeInput;
import frc.team5973.robot.rapidreact.Shooter.ShooterInput;
import frc.team5973.robot.rapidreact.commands.ballCollectionCommands.CenterBallCommand;
import frc.team5973.robot.rapidreact.commands.ballCollectionCommands.CollectBallCommand;
import frc.team5973.robot.rapidreact.commands.defaultCommands.IntakeDefaultCommand;
import frc.team5973.robot.rapidreact.commands.defaultCommands.ShooterDefaultCommand;
import frc.team5973.robot.rapidreact.commands.defaultCommands.SwerveDriveDefaultCommand;
import frc.team5973.robot.subsystems.Limelight;
import frc.team5973.robot.subsystems.SwerveDrive;
import frc.team5973.robot.subsystems.SwerveDrive.Axis;
import frc.team5973.robot.subsystems.SwerveDrive.DriveMode;

public class Robot extends RobotBase {
	
	private DetectionData detectionData;
	private Limelight limelight;
	private SwerveDrive drive;
	private Shooter shooter;
	private Intake intake;

	private Command autonomousCommand;
	private Auto auto;

	private int driverPort  = 0;
	private int copilotPort = 1;

	private final double SPEED_NORMAL  = 0.9;
	private final double SPEED_SAFE    = 0.3;
	
	private final double DEADBAND_LOW  = 0.04;
	private final double DEADBAND_HIGH = 1;

	private XinputController driver  = new XinputController(driverPort);
	private XinputController copilot = new XinputController(copilotPort);

	public Robot() {
		super("Rapidreact");

        port("shooterWheel",  12);  
		port("shooterOutake", 8); 
		
		port("intakeWheels", 10);
		port("leftArm", 0);
		port("rightArm", 1);
				
		port("frontRightSpeedMotor", 3);
        port("frontLeftSpeedMotor",  0);
        port("backRightSpeedMotor",  2);
        port("backLeftSpeedMotor",   1);
        
        port("frontRightAngleMotor", 7);
        port("frontLeftSAngleMotor", 4);
        port("backRightAngleMotor",  6);
        port("backLeftAngleMotor",   5);

		drive = new SwerveDrive(this);
		
		shooter = new Shooter(this);
		intake  = new Intake(this);

		detectionData = new DetectionData(this);
		limelight     = new Limelight(this);
	
		drive.setDefaultCommand(new SwerveDriveDefaultCommand(drive, limelight, 
																	 detectionData, DEADBAND_HIGH,
																	 				DEADBAND_LOW,
																					SPEED_NORMAL, 
																					SPEED_SAFE, 
																					Map.of(
			Axis.FORWARD, () -> driver.getLeftY(),
			Axis.STRAFE,  () -> driver.getLeftX(),
			Axis.TURN,    () -> driver.getRightX()),
			Map.of(
			DriveMode.SAFEMMODE, () -> driver.getLeftStickButton() && driver.getRightStickButton(),
			DriveMode.FIELDMODE, () -> copilot.getBButton(),
			DriveMode.GOALMODE,  () -> copilot.getXButton(),
			DriveMode.BALLMODE,  () -> copilot.getYButton(),
			DriveMode.ZERO_GYRO, () -> copilot.getStartButton()
			)));

		shooter.setDefaultCommand(new ShooterDefaultCommand(shooter, limelight, Map.of(
			ShooterInput.BUTTON, () -> driver.getBButton()
		)));

		intake.setDefaultCommand(new IntakeDefaultCommand(intake, Map.of(
			IntakeInput.EXTEND, () -> driver.getAButton(),
			IntakeInput.COLLECT, () -> driver.getXButton()
		)));
	
		// mainJoystick.getButton(kX).whenHeld(new CenterBall(this, drive));
		// mainJoystick.getButton(kY).whenHeld(new CollectBall(this, drive, intake, 0.7));
		
		//initialize auto commands
		//auto = new Auto(drive, intake, shooter);

	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null)
			autonomousCommand.cancel();

		// if (auto != null)
		// 	auto.cancel();
	}

	@Override
	public void teleopPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void testPeriodic() {
		CommandScheduler.getInstance().run();
		
	}

	@Override
	public void disabledInit() {
		
	}

	@Override
	public void autonomousInit() {

		//auto.schedule();
		
	}

	@Override
	public void autonomousPeriodic() {
		CommandScheduler.getInstance().run();
	}

}
