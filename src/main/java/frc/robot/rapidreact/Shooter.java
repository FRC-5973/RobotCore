package frc.robot.rapidreact;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotBase;
import frc.robot.subsystems.SubsystemBase;

public class Shooter extends SubsystemBase {
    
    private final double SHOOTER_SPEED = 1;
    private final double OUTAKE_SPEED = 0.3;

    private WPI_TalonSRX shooterWheel;
    private WPI_TalonSRX shooterOutake;

    private ColorSensorV3 colorSensor;
    private ColorMatch   colorMatcher;

    private Color redBallColor;
    private Color blueBallColor;

    public enum ShooterInput {BUTTON}

    private final Timer timer = new Timer();
    
    public Shooter(RobotBase robot) {

        super(robot);

        configureMotors();
        //configureColorSensors();

    }

    private void configureMotors() {
        
        shooterWheel  = new WPI_TalonSRX(port("shooterWheel"));
        shooterOutake = new WPI_TalonSRX(port("shooterOutake"));

        addChild("shooterWheel", shooterWheel);
        addChild("shooterOutake", shooterOutake);

        shooterWheel.setInverted(false);
        shooterOutake.setInverted(true);
    }

    private void configureColorSensors() {
        
        colorSensor = new ColorSensorV3(Port.kMXP);
        colorMatcher = new ColorMatch();

        redBallColor  = new Color(0.143, 0.427, 0.429); //TODO need to calibrate these to detect the color of the balls
        blueBallColor = new Color(0.197, 0.561, 0.240);
        
    }

    public void shoot() {
            
            //spin up flywheel
            shooterWheel.set(SHOOTER_SPEED);
            
            // Wait for 1 second to allow wheel to spin up   
            timer.reset();
            timer.start();
            while (!timer.hasElapsed(1)) {}
            timer.stop();

            //send ball through
            shooterOutake.set(OUTAKE_SPEED);
    }

    public void halt() {
        shooterWheel.set(0);
        shooterOutake.set(0);
    }

    public void shootForTime(double time) {

        //spin up flywheel
        shooterWheel.set(SHOOTER_SPEED);
        
        // Wait for 1 second to allow wheel to spin up   
        timer.reset();
        timer.start();
        while (!timer.hasElapsed(1)) {}
        timer.stop();

        //send ball through
        shooterOutake.set(OUTAKE_SPEED);

        timer.reset();
        timer.start();
        while (!timer.hasElapsed(time)) {}
        timer.stop();

        shooterWheel.set(0);
        shooterOutake.set(0);

    }

    public String detectedBallColor(ColorSensorV3 colorSensor) {
        
        String colorString;

        Color detectedColor = colorSensor.getColor();
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

        if (match.color == blueBallColor) {
            colorString = "Blue";
        } else if (match.color == redBallColor) {
            colorString = "Red";
        } else {
            colorString = "Unknown";
        }

        return colorString;
    }


}