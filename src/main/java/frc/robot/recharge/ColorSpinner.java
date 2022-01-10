package frc.robot.recharge;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotBase;
import frc.robot.subsystems.SubsystemBase;

public class ColorSpinner extends SubsystemBase {

	private final double SPIN_SPEED = 0.3;

	private Solenoid extendSpinner;
	private VictorSPX colorSpinner;

    private PneumaticsModuleType CTREPCM;

	public ColorSpinner(RobotBase robot) {

		super(robot);

		extendSpinner = new Solenoid(1 ,CTREPCM, configInt("extendSpinner"));
		colorSpinner = new VictorSPX(configInt("spinner"));

		initDefaultCommand();

	}

	public void initDefaultCommand() {

		setDefaultCommand(new CommandBase() {
		
			{
				addRequirements(ColorSpinner.this);
			}

			@Override
			public void execute() {

				if (button("spin")) {
			
					colorSpinner.set(ControlMode.PercentOutput, SPIN_SPEED);
					extendSpinner.set(true);
			
				} else {

					colorSpinner.set(ControlMode.PercentOutput, 0);
					extendSpinner.set(false);

				}
				
			}
		
		});

	}

	@Override
	public String getConfigName() {
		return "spinner";
	}

}