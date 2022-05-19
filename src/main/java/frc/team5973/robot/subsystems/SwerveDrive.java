package frc.team5973.robot.subsystems;

//pheonix libraries imports
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2;

//WPILib imports
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;

import frc.team5973.robot.RobotBase;
public class SwerveDrive extends SubsystemBase {
    
    private Pigeon2 gyro; //declare gyro

    private double KpAnglefl = 0.015; //PID values for each rotation encoder
	private double KiAnglefl = 0;
	private double KdAnglefl = 0;

    private double KpAnglefr = 0.015; //PID values for each rotation encoder
	private double KiAnglefr = 0;
	private double KdAnglefr = 0;

    private double KpAnglebl = 0.015; //PID values for each rotation encoder
	private double KiAnglebl = 0;
	private double KdAnglebl = 0;

    private double KpAnglebr = 0.015; //PID values for each rotation encoder
	private double KiAnglebr = 0;
	private double KdAnglebr = 0;

    private WPI_TalonFX frontRightSpeedMotor; //declare speed motors
    private WPI_TalonFX frontLeftSpeedMotor;
    private WPI_TalonFX backRightSpeedMotor;
    private WPI_TalonFX backLeftSpeedMotor;
    
    private WPI_TalonSRX frontRightAngleMotor; //declare rotation motors
    private WPI_TalonSRX frontLeftAngleMotor;
    private WPI_TalonSRX backRightAngleMotor;
    private WPI_TalonSRX backLeftAngleMotor;

    private boolean sensorPhasefl = false; //decide whether to invert the sensor values on the encoder or not
    private boolean sensorPhasefr = true;
    private boolean sensorPhaseBr = false;
    private boolean sensorPhaseBl = false;

    private PIDController pidAnglefl; //declare pid controllers
    private PIDController pidAnglefr;
    private PIDController pidAnglebl;
    private PIDController pidAnglebr;

    private final double L  = 23; //vehicle tracklength
    private final double W  = 23; //vehicle trackwidth
    private final double R  = Math.sqrt(Math.pow(L, 2) + Math.pow(W, 2)); //math needed for the swerve drive calculations
    private final double PI = Math.PI;

    private double positionAlongField = 0; //robot odemetry position
    private double positionAcrossField = 0;

    public enum Axis {FORWARD, STRAFE, TURN, BUTTON} //controller inputs for swerve
    public enum DriveMode {SAFEMMODE, FIELDMODE, GOALMODE, BALLMODE, ZERO_GYRO}

    private double STRNew = 0;
    private double FWDNew = 0;

    private double storedHeading = 0;
    private double correction = 0;

    public SwerveDrive(RobotBase robot) {
        
        super(robot);
        configureMotors();
        configureGyro();
        configurePID();

        reset();        
    }

    private void configureMotors() {

        //declare ports for the motors using our custom port mapping
        // if you dont want to use the custom port mapping just replace port() with the port number of the motor or your own variables
        frontRightSpeedMotor = new WPI_TalonFX(port("frontRightSpeedMotor"));
        frontLeftSpeedMotor  = new WPI_TalonFX(port("frontLeftSpeedMotor"));
        backRightSpeedMotor  = new WPI_TalonFX(port("backRightSpeedMotor"));
        backLeftSpeedMotor   = new WPI_TalonFX(port("backLeftSpeedMotor"));

        frontRightAngleMotor = new WPI_TalonSRX(port("frontRightAngleMotor"));
        frontLeftAngleMotor  = new WPI_TalonSRX(port("frontLeftSAngleMotor"));
        backRightAngleMotor  = new WPI_TalonSRX(port("backRightAngleMotor"));
        backLeftAngleMotor   = new WPI_TalonSRX(port("backLeftAngleMotor"));

        addChild("frontRightSpeedMotor", frontRightSpeedMotor);
        addChild("frontLeftSpeedMotor",  frontLeftSpeedMotor);
        addChild("backRightSpeedMotor",  backRightSpeedMotor);
        addChild("backLeftSpeedMotor",   backLeftSpeedMotor);

        addChild("frontRightAngleMotor", frontRightAngleMotor);
        addChild("frontLeftSAngleMotor", frontLeftAngleMotor);
        addChild("backRightAngleMotor",  backRightAngleMotor);
        addChild("backLeftAngleMotor",   backLeftAngleMotor);

        //invert speed motors
        frontRightSpeedMotor.setInverted(true);
        frontLeftSpeedMotor.setInverted(false);
        backRightSpeedMotor.setInverted(true);
        backLeftSpeedMotor.setInverted(false);

        //invert angle motors
        frontRightAngleMotor.setInverted(false);
        frontLeftAngleMotor.setInverted(true);
        backRightAngleMotor.setInverted(true);
        backLeftAngleMotor.setInverted(true);

        //set angle motors to their default facotry state
        frontRightAngleMotor.configFactoryDefault();
        frontLeftAngleMotor.configFactoryDefault();
        backRightAngleMotor.configFactoryDefault();
        backLeftAngleMotor.configFactoryDefault();
    
        //configure the rotation encoders correctly so that they can be read by the talons
        frontRightAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        frontLeftAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        backRightAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        backLeftAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);

        frontRightAngleMotor.setSensorPhase(sensorPhasefr); //set the sensorphase
        frontLeftAngleMotor.setSensorPhase(sensorPhasefl);
        backLeftAngleMotor.setSensorPhase(sensorPhaseBl);
        backRightAngleMotor.setSensorPhase(sensorPhaseBr);
        
        //configure built in encoders on the falcons so they can be correctly used
        frontRightSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        frontLeftSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        backRightSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        backLeftSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        //turn off ramping for angleMotors
        backLeftAngleMotor.configOpenloopRamp(0);
        backLeftAngleMotor.configClosedloopRamp(0);

        backRightAngleMotor.configOpenloopRamp(0);
        backRightAngleMotor.configClosedloopRamp(0);

        frontLeftAngleMotor.configOpenloopRamp(0);
        frontLeftAngleMotor.configClosedloopRamp(0);

        frontRightAngleMotor.configOpenloopRamp(0);
        frontRightAngleMotor.configClosedloopRamp(0);

        //brake mode for speed motors
        backLeftSpeedMotor.setNeutralMode(NeutralMode.Brake);
        backRightSpeedMotor.setNeutralMode(NeutralMode.Brake);
        frontLeftSpeedMotor.setNeutralMode(NeutralMode.Brake);
        frontRightSpeedMotor.setNeutralMode(NeutralMode.Brake);

        //brake mode for angle motors
        backLeftAngleMotor.setNeutralMode(NeutralMode.Brake);
        backRightAngleMotor.setNeutralMode(NeutralMode.Brake);
        frontLeftAngleMotor.setNeutralMode(NeutralMode.Brake);
        frontRightAngleMotor.setNeutralMode(NeutralMode.Brake);

    }

    private void configureGyro() { //set up pigeon
        gyro = new Pigeon2(0);

       gyro.setYaw(0, 69);

    }

    private void configurePID() { //set pid values for the encoders

        pidAnglefl = new PIDController(KpAnglefl, KiAnglefl, KdAnglefl);
        pidAnglefr = new PIDController(KpAnglefr, KiAnglefr, KdAnglefr);
        pidAnglebl = new PIDController(KpAnglebl, KiAnglebl, KdAnglebl);
        pidAnglebr = new PIDController(KpAnglebr, KiAnglebr, KdAnglebr);  

	}

    //use this method if you are using encoders that don't plug directly into motor controllers
    // private void configureEncoders() {

    //     frontRightEncoder = new Encoder(7, 6);//new Encoder(4, 5);
    //     frontLeftEncoder  = new Encoder(1, 0);//new Encoder(7, 6);
    //     backRightEncoder  =  new Encoder(5,4);//new Encoder(3, 2);
    //     backLeftEncoder   = new Encoder(3,2); //new Encoder(0, 1);
        
    //     final double TICKS_TO_DEGREES = 1.12;

    //     frontRightEncoder.setDistancePerPulse(1./TICKS_TO_DEGREES);
    //     frontLeftEncoder.setDistancePerPulse(1./TICKS_TO_DEGREES);
    //     backRightEncoder.setDistancePerPulse(1./TICKS_TO_DEGREES);
    //     backLeftEncoder.setDistancePerPulse(1./TICKS_TO_DEGREES);

    // }
    
    //this method takes in the inputs from a controller or seperate forward strafe and rotate values and converts them into the correct wheel speeds and positions
    private void calculateDrive(double FWD, double STR, double RCW, boolean useGyro) {
               
        if(useGyro) { //determine whether to use field centric orientation or not

            double gyroAngle = -gyro.getYaw() * PI / 180;

            FWDNew = FWD*Math.cos(gyroAngle) + STR*Math.sin(gyroAngle); //uses the gyro to calcualte field centric transformation
            STRNew = STR*Math.cos(gyroAngle) - FWD*Math.sin(gyroAngle);

        } else {
            
            FWDNew = FWD*Math.cos(0) + STR*Math.sin(0); //passes zero for robot oriented drive
            STRNew = STR*Math.cos(0) - FWD*Math.sin(0);

        }

        double A = STRNew - RCW*(L/R); //values a b c and d are used to calcualte meaningful wheel speeds
        double B = STRNew + RCW*(L/R);
        double C = FWDNew - RCW*(W/R);
        double D = FWDNew + RCW*(W/R); 

        double frontRightWheelSpeed = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2)); //average the values of a b c and d to get wheel speeds
        double frontLeftWheelSpeed  = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));
        double backLeftWheelSpeed   = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));
        double backRightWheelSpeed  = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));

        double frontRightWheelAngle = (Math.atan2(B, C) * (180 / PI)); // calculate the proper angles that each of the wheels ned to be
        double frontLeftWheelAngle  = (Math.atan2(B, D) * (180 / PI));
        double backLeftWheelAngle   = (Math.atan2(A, D) * (180 / PI));
        double backRightWheelAngle  = (Math.atan2(A, C) * (180 / PI));

        //normalize wheel speeds
        double max = frontRightWheelSpeed;

        //find which wheel spped is the greatest
        if(frontLeftWheelSpeed > max) {
            max = frontLeftWheelSpeed;
        } else if(backLeftWheelSpeed > max) {
            max = backLeftWheelSpeed;
        } else if (backRightWheelSpeed > max) {
            max = backRightWheelSpeed;
        } else {}

        //set each wheel to the greatest wheel speed
        frontRightWheelSpeed = max; 
        frontLeftWheelSpeed  = max; 
        backLeftWheelSpeed   = max; 
        backRightWheelSpeed  = max;

        //set the angle and speed of all motors
        backLeftAngleMotor.set(pidAnglebl.calculate(ticksToDegrees(backLeftAngleMotor),      setDirection(backLeftWheelAngle,   backLeftAngleMotor, backLeftSpeedMotor, backLeftWheelSpeed)));
        backRightAngleMotor.set(pidAnglebr.calculate(ticksToDegrees(backRightAngleMotor),    setDirection(backRightWheelAngle,  backRightAngleMotor, backRightSpeedMotor, backRightWheelSpeed)));
        frontRightAngleMotor.set(pidAnglefr.calculate(ticksToDegrees(frontRightAngleMotor),  setDirection(frontRightWheelAngle, frontRightAngleMotor, frontRightSpeedMotor, frontRightWheelSpeed)));
        frontLeftAngleMotor.set(pidAnglefl.calculate(ticksToDegrees(frontLeftAngleMotor),    setDirection(frontLeftWheelAngle,  frontLeftAngleMotor, frontLeftSpeedMotor, frontLeftWheelSpeed)));

    }

    //Get the closest angle between the given angles.
    private static double closestAngle(double a, double b) {
            // get direction
            double dir = (b % 360) - (a % 360);

            // convert from -360 to 360 to -180 to 180
            if(Math.abs(dir) > 180.0) {
                    dir = -(Math.signum(dir) * 360.0) + dir;
            }
            return dir;
    }

    private double setDirection(double setpoint, WPI_TalonSRX motor, WPI_TalonFX speedMotor, double speed) {
        
        double direction = 0;
        double currentAngle = ticksToDegrees(motor);
        // find closest angle to setpoint
        double setpointAngle = closestAngle(currentAngle, setpoint);
        // find closest angle to setpoint + 180
        double setpointAngleFlipped = closestAngle(currentAngle, setpoint + 180.0);
        // if the closest angle to setpoint is shorter
        if(Math.abs(setpointAngle) <= Math.abs(setpointAngleFlipped)) {
            // unflip the motor direction use the setpoint
            direction = currentAngle + setpointAngle;
        }
        // if the closest angle to setpoint + 180 is shorter
        else {
            // flip the motor direction and use the setpoint + 180
            speed = -speed;
            direction = currentAngle + setpointAngleFlipped;
        }

        speedMotor.set(speed);
        return direction;
    }
    
    //converts ticks to degrees so the encoder values are easy to read
    private double ticksToDegrees(WPI_TalonSRX motor) {
        return motor.getSelectedSensorPosition() / (2.844444444444444444444444444444444444444444444);
    }
    
    private double calcYawStraight(double targetAngle, double currentAngle, double kP) {
        double errorAngle = (targetAngle - (currentAngle % 360));
        double correction = errorAngle * kP;

        return correction;
    }

    public double correctHeading(double kP, double FWD, double STR, double RCW) {
        if(RCW != 0) {
            storedHeading = -gyro.getYaw();
        } else {

            if(Math.abs(FWD) > 0 || Math.abs(STR) > 0) {
                correction = calcYawStraight(storedHeading, -gyro.getYaw(), kP);

                if(Math.abs(correction) > 1) {
                    correction = 0;
                }
            }
        }

        return correction;
    }
    
    //TODO: need to test this
    public void calculateRobotPosition() {

        double Bfl = Math.sin(frontLeftAngleMotor.getSelectedSensorPosition())  * frontLeftSpeedMotor.getSelectedSensorVelocity();
        double Bfr = Math.sin(frontRightAngleMotor.getSelectedSensorPosition()) * frontRightSpeedMotor.getSelectedSensorVelocity();
        double Abl = Math.sin(backLeftAngleMotor.getSelectedSensorPosition())   * backLeftSpeedMotor.getSelectedSensorVelocity();
        double Abr = Math.sin(backRightAngleMotor.getSelectedSensorPosition())  * backRightSpeedMotor.getSelectedSensorVelocity();

        double Dfl = Math.cos(frontLeftAngleMotor.getSelectedSensorPosition())  * frontLeftSpeedMotor.getSelectedSensorVelocity();
        double Cfr = Math.cos(frontRightAngleMotor.getSelectedSensorPosition()) * frontRightSpeedMotor.getSelectedSensorVelocity();
        double Dbl = Math.cos(backLeftAngleMotor.getSelectedSensorPosition())   * backLeftSpeedMotor.getSelectedSensorVelocity();
        double Cbr = Math.cos(backRightAngleMotor.getSelectedSensorPosition())  * backRightSpeedMotor.getSelectedSensorVelocity();

        double A = (Abr + Abl) / 2;
        double B = (Bfl + Bfr) / 2;
        double C = (Cfr + Cbr) / 2;
        double D = (Dfl + Dbl) / 2;

        double rotation1 = (B - A) / L;
        double rotation2 = (C - D) / W;
        double rotation = (rotation1  + rotation2) / 2;

        double forward1 = rotation * (L / 2) + A;
        double forward2 = -rotation * (L / 2) + B;
        double forward = (forward1 + forward2) / 2;

        double strafe1 = rotation * (W / 2) + C;
        double strafe2 = -rotation * ( W / 2) + D;
        double strafe = (strafe1 + strafe2) / 2;

        double forwardNew = (forward * Math.cos(gyro.getYaw())) + (strafe *  Math.sin(gyro.getYaw())); 
        double strafeNew  = (strafe *  Math.cos(gyro.getYaw())) - (forward * Math.sin(gyro.getYaw()));

        double lastTime = Timer.getFPGATimestamp();
        double timeStep = Timer.getFPGATimestamp() - lastTime; //milliseconds //Timer.getFPGATimestamp() - lastTime //don't know how to do this so just constant from whitepaper
        System.out.println("time step: " + timeStep);

        positionAlongField = positionAlongField + (forwardNew * timeStep);
        positionAcrossField = positionAcrossField + (strafeNew * timeStep);

    }

    public void goToPosition(double posX, double posY) {
        
        //so we need to reverse the above method to get motor outputs
        while(!(posX == positionAcrossField) && !(posY == positionAlongField)) {

        }
    }

    public double[] getRobotPosition() {

        double[] coordinates = {positionAcrossField, positionAlongField};
        return coordinates;
    }

    public void resetRobotPosition() {
        positionAlongField = 0;
        positionAcrossField = 0;
    }

    public double getGyroAngle() {
        return -gyro.getYaw();
    }

    public void resetGyro() {
        gyro.configFactoryDefault();
    }

    //Drive methods for commands
    public void driveForward(double speed, boolean useGyro) {
        calculateDrive(speed, 0, 0, useGyro);
    }

    public void driveBackward(double speed, boolean useGyo) {
        calculateDrive(-speed, 0, 0, useGyo);
    }

    public void strafeLeft(double speed, boolean useGyro) {
        calculateDrive(0, speed, 0, useGyro);
    }

    public void strafeRight(double speed, boolean useGyro) {
        calculateDrive(0, -speed, 0, useGyro);
    }

    public void rotate(double speed, boolean useGyro) {
        calculateDrive(0, 0, speed, useGyro);
    }

    public void halt() {
        calculateDrive(0, 0, 0, false);
    }

    public void swerveDrive(double FWD, double STR, double RCW, boolean useGyro) {
        calculateDrive(FWD, STR, RCW, useGyro);
    }

	@Override
	public void periodic() {
		
	}

	public void stop() {    

	}

	public void reset() {
		
	}
	@Override
	public void initSendable(SendableBuilder builder) {

		super.initSendable(builder);

        builder.addDoubleProperty("PAnglefl", () -> KpAnglefl, (value) -> KpAnglefl = value);
		builder.addDoubleProperty("IAnglefl", () -> KiAnglefl, (value) -> KiAnglefl = value);
		builder.addDoubleProperty("DAnglefl", () -> KdAnglefl, (value) -> KdAnglefl = value);

        builder.addDoubleProperty("PAnglefr", () -> KpAnglefr, (value) -> KpAnglefr = value);
		builder.addDoubleProperty("IAnglefr", () -> KiAnglefr, (value) -> KiAnglefr = value);
		builder.addDoubleProperty("DAnglefr", () -> KdAnglefr, (value) -> KdAnglefr = value);

        builder.addDoubleProperty("PAnglebl", () -> KpAnglebl, (value) -> KpAnglebl = value);
		builder.addDoubleProperty("IAnglebl", () -> KiAnglebl, (value) -> KiAnglebl = value);
		builder.addDoubleProperty("DAnglebl", () -> KdAnglebl, (value) -> KdAnglebl = value);

        builder.addDoubleProperty("PAnglebr", () -> KpAnglebr, (value) -> KpAnglebr = value);
		builder.addDoubleProperty("IAnglebr", () -> KiAnglebr, (value) -> KiAnglebr = value);
		builder.addDoubleProperty("DAnglebr", () -> KdAnglebr, (value) -> KdAnglebr = value);

		builder.addDoubleProperty("Forward", () ->   FWDNew, null);
		builder.addDoubleProperty("Strafe",  () ->  STRNew, null);
		// builder.addDoubleProperty("Rotate",  () ->   MathUtil.applyDeadband(axis("rotate"),  DEADBAND), null);

        builder.addDoubleProperty("Gyro Angle", () -> -gyro.getYaw(), null);
        
        builder.addDoubleProperty("Front Right Angle", () -> ticksToDegrees(frontRightAngleMotor), null);		
		builder.addDoubleProperty("Front Left Angle",  () -> ticksToDegrees(frontLeftAngleMotor),  null);
		builder.addDoubleProperty("Back Right Angle",  () -> ticksToDegrees(backRightAngleMotor),  null);
		builder.addDoubleProperty("Back Left Angle",   () -> ticksToDegrees(backLeftAngleMotor),   null);

        builder.addDoubleArrayProperty("Robot Position", () -> getRobotPosition(), null);
	
        builder.addDoubleProperty("frontRightSpeedMotor", () -> frontRightSpeedMotor.get(), null);		
		builder.addDoubleProperty("frontLeftSpeedMotor",  () -> frontLeftSpeedMotor.get(),  null);
		builder.addDoubleProperty("backRightSpeedMotor",  () -> backRightSpeedMotor.get(),  null);
		builder.addDoubleProperty("backLeftSpeedMotor",   () -> backLeftSpeedMotor.get(),   null);
        
        builder.addDoubleProperty("frontRightAngleMotor", () -> frontRightAngleMotor.get(), null);		
		builder.addDoubleProperty("frontLeftSAngleMotor", () -> frontLeftAngleMotor.get(), null);
		builder.addDoubleProperty("backRightAngleMotor",  () -> backRightAngleMotor.get(),  null);
		builder.addDoubleProperty("backLeftAngleMotor",   () -> backLeftAngleMotor.get(),   null);
		
	}

   
}
