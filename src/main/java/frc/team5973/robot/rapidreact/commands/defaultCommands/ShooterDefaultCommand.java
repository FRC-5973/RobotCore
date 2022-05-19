package frc.team5973.robot.rapidreact.commands.defaultCommands;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5973.robot.rapidreact.Shooter;
import frc.team5973.robot.rapidreact.Shooter.ShooterInput;
import frc.team5973.robot.subsystems.Limelight;

public class ShooterDefaultCommand extends CommandBase {
    final Shooter shooter;
    final Limelight limelight;
    final Map<ShooterInput, BooleanSupplier> inputMap;

    private double shooterSpeed = 0;

    public ShooterDefaultCommand(final Shooter shooter, final Limelight limelight, Map<ShooterInput, BooleanSupplier> inputMap) {
        this.limelight = limelight;
        this.shooter = shooter;
        this.inputMap = inputMap;

        addRequirements(shooter);
        SendableRegistry.addChild(ShooterDefaultCommand.this, this);
    }

    @Override
	public void execute() {
        
        if(limelight.getOffsetY() <= 13.5) {
            shooterSpeed = 0.9688 - (0.0399 * limelight.getOffsetY()) +
                           ((0.0055) * Math.pow(limelight.getOffsetY(), 2)) - (0.000273 * Math.pow(limelight.getOffsetY(), 3));
        } else if(limelight.getOffsetY() > 13.5) {
            shooterSpeed = -0.6264 + (0.237 * limelight.getOffsetY()) - (0.01312 * Math.pow(limelight.getOffsetY(), 2)) + 
                                                                        (0.000233 * Math.pow(limelight.getOffsetY(), 3));
        }

        //System.out.println(shooterSpeed);
        
        if(button(ShooterInput.BUTTON)) {   
            
            shooter.shoot(shooterSpeed);

        } else {
           
            shooter.halt();

        }
	
	}

    private final boolean button(ShooterInput intakeInput) {
        return inputMap.get(intakeInput).getAsBoolean();
    }

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		
	}
    
}
