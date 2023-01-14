package frc.robot.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Swerve {
    
    private static final double MAX_SPEED_METERS_PER_SEC = 2;
    
    private final SwerveModule
        flModule = new SwerveModule(
            "front left",
            Constants.FL_DRIVE_MOTOR_ID,
            Constants.FL_STEER_MOTOR_ID,
            Constants.FL_STEER_ENCODER_ID),
        frModule = new SwerveModule(
            "front right",
            Constants.FR_DRIVE_MOTOR_ID,
            Constants.FR_STEER_MOTOR_ID,
            Constants.FR_STEER_ENCODER_ID),
        rlModule = new SwerveModule(
            "rear left",
            Constants.RL_DRIVE_MOTOR_ID,
            Constants.RL_STEER_MOTOR_ID,
            Constants.RL_STEER_ENCODER_ID),
        rrModule = new SwerveModule(
            "rear right",
            Constants.RR_DRIVE_MOTOR_ID,
            Constants.RR_STEER_MOTOR_ID,
            Constants.RR_STEER_ENCODER_ID);
    
    // TODO: Measure me!
    private final Translation2d
        flTranslation = new Translation2d(1, 1),
        frTranslation = new Translation2d(1, -1),
        rlTranslation = new Translation2d(-1, 1),
        rrTranslation = new Translation2d(-1, -1);
    
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        flTranslation,
        frTranslation,
        rlTranslation,
        rrTranslation);
    
    public void update (ChassisSpeeds speeds) {
        
        // Log data to smartdashboard
        SmartDashboard.putNumber("x-speed", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("y-speed", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("rotational-speed (radians)", speeds.omegaRadiansPerSecond);
        
        // Get the desired swerve module states based on the desired ChassisSpeeds
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        
        // Scale down drive speeds to ensure they are proportional
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_SPEED_METERS_PER_SEC);
        
        // Update swerve modules
        flModule.setDesiredState(moduleStates[0]);
        frModule.setDesiredState(moduleStates[1]);
        rlModule.setDesiredState(moduleStates[2]);
        rrModule.setDesiredState(moduleStates[3]);
        
    }
    
    public void stop () {
        flModule.stop();
        frModule.stop();
        rlModule.stop();
        rrModule.stop();
    }
    
}
