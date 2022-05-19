package frc.team5973.robot.rapidreact.commands.defaultCommands;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team5973.robot.rapidreact.DetectionData;
import frc.team5973.robot.subsystems.Limelight;
import frc.team5973.robot.subsystems.SwerveDrive;
import frc.team5973.robot.subsystems.SwerveDrive.Axis;
import frc.team5973.robot.subsystems.SwerveDrive.DriveMode;

public class SwerveDriveDefaultCommand extends CommandBase {
    final SwerveDrive drive;
    final Limelight limelight;
    final DetectionData detectionData;
    final Map<Axis, DoubleSupplier> axisMap;
    final Map<DriveMode, BooleanSupplier> buttonMap;

    private final double DEADBAND_LOW;
    private final double DEADBAND_HIGH;

    private final double SPEED_NORMAL;
    private final double SPEED_SAFE;

    private double speed;
 
    private double comboStartTimeSafe  = 0;
    private double comboStartTimeField = 0;
    private double comboStartTimeGoal  = 0;
    private double comboStartTimeBall  = 0;
    
    private boolean alreadyToggledSafeMode  = false;
    private boolean alreadyToggledFieldMode = false;
    private boolean alreadyToggledGoalMode  = false;
    private boolean alreadyToggledBallMode   = false;

    private boolean safeMode          = false;
    private boolean fieldOrientedMode = false;
    private boolean goalOrientedMode  = false;
    private boolean ballOrientedMode  = false;

    private double buttonDelay = 0.1;

    private double forward;
    private double strafe;
    private double rotate;

    private double yawCorrection;
   
    private int driveMode;

    private final int fieldOriented = 1;
    private final int robotOriented = 2;
    private final int goalOriented  = 3;
    private final int ballOriented  = 4;

    public SwerveDriveDefaultCommand(final SwerveDrive drive,
                                     final Limelight limelight,
                                     final DetectionData detectionData, 
                                     final double DEADBAND_HIGH,
                                     final double DEADBAND_LOW,
                                     final double SPEED_NORMAL,
                                     final double SPEED_SAFE,
                                     final Map<Axis, DoubleSupplier> axisMap,
                                     final Map<DriveMode, BooleanSupplier> buttonMap) {
        
        this.detectionData = detectionData;
        this.limelight     = limelight;
        this.buttonMap     = buttonMap;
        this.axisMap       = axisMap;
        this.drive         = drive;
        
        this.DEADBAND_HIGH = DEADBAND_HIGH;
        this.DEADBAND_LOW  = DEADBAND_LOW;
        this.SPEED_NORMAL = SPEED_NORMAL;
        this.SPEED_SAFE   = SPEED_SAFE;
    
        addRequirements(drive);
        SendableRegistry.addChild(SwerveDriveDefaultCommand.this, this);

    }

    @Override
	public void execute() {
        
        speed = safeMode ? SPEED_SAFE : SPEED_NORMAL;
        
        // forward =  MathUtil.clamp(MathUtil.applyDeadband(applyRadialDeadZoneX(axis(Axis.FORWARD), axis(Axis.STRAFE), DEADBAND_LOW, DEADBAND_HIGH), DEADBAND_LOW) * speed, -1, 1);
        // strafe  = -MathUtil.clamp(MathUtil.applyDeadband(applyRadialDeadZoneY(axis(Axis.FORWARD), axis(Axis.STRAFE), DEADBAND_LOW, DEADBAND_HIGH), DEADBAND_LOW) * speed, -1, 1);
        // rotate  = -MathUtil.clamp(MathUtil.applyDeadband(axis(Axis.TURN),    DEADBAND_LOW) * speed, -1, 1);

        forward =   MathUtil.clamp(MathUtil.applyDeadband(axis(Axis.FORWARD), DEADBAND_LOW) * speed, -1, 1);
        strafe  =  -MathUtil.clamp(MathUtil.applyDeadband(axis(Axis.STRAFE),  DEADBAND_LOW) * speed, -1, 1);
        rotate  =  -MathUtil.clamp(MathUtil.applyDeadband(axis(Axis.TURN),    DEADBAND_LOW) * speed, -1, 1);

        if(driveMode == goalOriented || driveMode == ballOriented) {
            yawCorrection = 0;
        } else if (driveMode == robotOriented || driveMode == fieldOriented) {
            yawCorrection = drive.correctHeading(0.004, forward, strafe, rotate);

        }
        

        //puts robot into safemode where the robot will go slower
         if(button(DriveMode.SAFEMMODE)) {

            if(comboStartTimeSafe == 0)
                comboStartTimeSafe = Timer.getFPGATimestamp();
            else if(Timer.getFPGATimestamp() - comboStartTimeSafe >= 3.0 && !alreadyToggledSafeMode) {
            
                safeMode = !safeMode;
                
                alreadyToggledSafeMode = true;
                System.out.println("Safemode is " + (safeMode ? "Enabled" : "Disabled") + ".");
            
            }

        } else {

            comboStartTimeSafe = 0;
            alreadyToggledSafeMode = false;
        
        }

        // toggle POV and field mode
        if(button(DriveMode.FIELDMODE) && !button(DriveMode.GOALMODE) && !button(DriveMode.BALLMODE)) {

            if(comboStartTimeField == 0) {
                comboStartTimeField = Timer.getFPGATimestamp();
            } else if(Timer.getFPGATimestamp() - comboStartTimeField >= buttonDelay && !alreadyToggledFieldMode) {
                
                fieldOrientedMode = !fieldOrientedMode;

                alreadyToggledFieldMode = true;
                driveMode = fieldOrientedMode ? fieldOriented : robotOriented;
                System.out.println("Switching to " + (fieldOrientedMode ? "Field Oriented" : "Robot POV") + ".");
            }

        } else {

            comboStartTimeField = 0;
            alreadyToggledFieldMode = false;
            
        }

        //toggle goal centric mode
        if(button(DriveMode.GOALMODE) && !button(DriveMode.FIELDMODE) && !button(DriveMode.BALLMODE) && limelight.isTargetValid()) {
            
            if(comboStartTimeGoal == 0) {
                comboStartTimeGoal = Timer.getFPGATimestamp();
            } else if(Timer.getFPGATimestamp() - comboStartTimeGoal >= buttonDelay && !alreadyToggledGoalMode) {
                goalOrientedMode = !goalOrientedMode;

                alreadyToggledGoalMode = true;
                driveMode = goalOrientedMode ? goalOriented : fieldOriented;
                System.out.println("Switching to " + (goalOrientedMode ? "Goal Oriented" : "Field Oriented") + ".");
            }
            
        } else if (button(DriveMode.GOALMODE) && !button(DriveMode.FIELDMODE) && !button(DriveMode.BALLMODE) && !limelight.isTargetValid()) {

            comboStartTimeGoal = 0;
            alreadyToggledGoalMode = false;
            
            driveMode = fieldOriented;
            System.out.println("No valid target to change drive mode" + "\n Switching to Field Oriented Mode");
        } else {

            comboStartTimeGoal = 0;
            alreadyToggledGoalMode = false;
        }

        //toggle ball centric mode
        if(button(DriveMode.BALLMODE) && !button(DriveMode.GOALMODE) && !button(DriveMode.FIELDMODE)) { //&& detectionData.isAnyBallDetected()) {

            if(comboStartTimeBall == 0) {
                comboStartTimeBall = Timer.getFPGATimestamp();
            } else if(Timer.getFPGATimestamp() - comboStartTimeBall >= buttonDelay && !alreadyToggledBallMode) {
                ballOrientedMode = !goalOrientedMode;

                alreadyToggledBallMode = true;
                driveMode = ballOrientedMode ? ballOriented : fieldOriented;
                System.out.println("Switching to " + (ballOrientedMode ? "Ball Oriented" : "Field Oriented") + ".");
            }

        } else if(button(DriveMode.BALLMODE) && !button(DriveMode.GOALMODE) && !button(DriveMode.FIELDMODE)) { //&& !detectionData.isAnyBallDetected()) {

            comboStartTimeBall = 0;
            alreadyToggledBallMode = false;
            
            driveMode = fieldOriented;
            System.out.println("No valid target to change drive mode" + "\n Switching to Field Oriented Mode");
        } else {
            comboStartTimeBall = 0;
            alreadyToggledBallMode = false;
        }

        if(button(DriveMode.ZERO_GYRO)) {
            drive.resetGyro();

            System.out.println("Gyro reset");
        }

        //set drive modes
        switch (driveMode) {
            case fieldOriented: drive.swerveDrive(forward, strafe, rotate - yawCorrection, true);
                    break;
            case robotOriented: drive.swerveDrive(forward, strafe, rotate - yawCorrection, false);
                    break;
            case goalOriented: drive.swerveDrive(-forward, 
                                                 -strafe, 
                                                 rotate - limelight.limelightXPID(), 
                                                 false);
                    break;
            case ballOriented: drive.swerveDrive(-forward + detectionData.piYPID(), 
                                                 -strafe, 
                                                 -rotate - detectionData.piXPID("blue"), 
                                                  false);
                    break;
            default: drive.swerveDrive(forward, strafe, rotate - yawCorrection, true);
                     break;
        }

        //System.out.println(detectionData.getOffsetX("blue"));
        System.out.println(detectionData.piXPID("blue"));

        //System.out.println(driveMode);

        //robot odometry
        //drive.calculateRobotPosition();
	
	}

	private final double axis(Axis axis) {
		return axisMap.get(axis).getAsDouble();
	}

    private final boolean button(DriveMode button) {
        return buttonMap.get(button).getAsBoolean();
    }

    private double applyRadialDeadZoneX(double x, double y, double deadZoneLow, double deadZoneHigh) {
        double output[] = {0,0};
        double mag = Math.sqrt(x*x + y*y);
    
        if (mag > deadZoneLow)
        {
            // scale such that output magnitude is in the range [0.0f, 1.0f]
            double legalRange = 1.0d - deadZoneHigh - deadZoneLow;
            double normalizedMag = Math.min(1.0d, (mag - deadZoneLow) / legalRange);
            double scale = normalizedMag / mag; 
            output[0] = x * scale;
            output[1] = y * scale;
        }
        else
        {
            // stick is in the inner dead zone
            output[0] = 0.0d;
            output[1] = 0.0d;
        }

        return output[0];
    }

    private double applyRadialDeadZoneY(double x, double y, double deadZoneLow, double deadZoneHigh) {
        double output[] = {0,0};
        double mag = Math.sqrt(x*x + y*y);
    
        if (mag > deadZoneLow)
        {
            // scale such that output magnitude is in the range [0.0f, 1.0f]
            double legalRange = 1.0d - deadZoneHigh - deadZoneLow;
            double normalizedMag = Math.min(1.0d, (mag - deadZoneLow) / legalRange);
            double scale = normalizedMag / mag; 
            output[0] = x * scale;
            output[1] = y * scale;
        }
        else
        {
            // stick is in the inner dead zone
            output[0] = 0.0d;
            output[1] = 0.0d;
        }

        return output[1];
    }


	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);

        
		
	}

}
