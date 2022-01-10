package frc.robot.recharge;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotBase;
import frc.robot.subsystems.SubsystemBase;

public class GeneratorHanger extends SubsystemBase {

	private final double PULLER_SPEED = 0.3;

	private WPI_TalonSRX hangerPuller;

	public GeneratorHanger(RobotBase robot) {

		super(robot);

		hangerPuller = new WPI_TalonSRX(configInt("hangerPuller"));
		hangerPuller.setInverted(true);

		initDefaultCommand();

	}

	public void initDefaultCommand() {

		setDefaultCommand(new CommandBase() {
		
			{
				addRequirements(GeneratorHanger.this);
			}

			@Override
			public void execute() {

				if (button("hangerUp")) {

					hangerPuller.set(-PULLER_SPEED);

				} else if (button("hangerDown")) {

					hangerPuller.set(PULLER_SPEED);

				}

			}
		
		});

	}

	@Override
	public String getConfigName() {
		return "hanger";
	}

}
