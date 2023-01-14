package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    
    // TODO: Tune me!
    private static final double
        DRIVE_METERS_PER_SEC_TO_MOTOR_VOLTAGE = 2.5,
        STEER_ANGLE_TOLERANCE_DEGREES = 4;
    
    private final String moduleName;
    private final WPI_TalonFX
        driveMotor,
        steerMotor;
    
    private final ResettableEncoder steerEncoder;
    
    /**
     * Convert angular offset (degrees) into a voltage for the steer motor.
     * (offset degrees) * kP = voltage
     */
    private final PIDController steerMotorPID = new PIDController(0.02, 0, 0);
    
    public SwerveModule (String moduleName, int driveMotorCANId, int steerMotorCANId, int steerCANCoderId) {
        this.moduleName = moduleName;
        driveMotor = new WPI_TalonFX(driveMotorCANId);
        steerMotor = new WPI_TalonFX(steerMotorCANId);
        steerEncoder = new ResettableEncoder(steerCANCoderId);
        
        // Steering PID configuration
        steerMotorPID.setTolerance(STEER_ANGLE_TOLERANCE_DEGREES);
    }
    
    public void setDesiredState (SwerveModuleState state) {
        
        // Log data to smart dashboard
        SmartDashboard.putNumber(moduleName + " current angle", steerEncoder.getAngle());
        SmartDashboard.putNumber(moduleName + " desired drive speed", state.speedMetersPerSecond);
        SmartDashboard.putNumber(moduleName + " desired angle (degrees)", state.angle.getDegrees());
        
        // Get the voltages for the drive and steering motors
        final double
            driveVoltage = DRIVE_METERS_PER_SEC_TO_MOTOR_VOLTAGE * state.speedMetersPerSecond,
            steerVoltage = steerMotorPID.calculate(steerEncoder.getAngle(), state.angle.getDegrees());
        
        // Set the motor controllers
        driveMotor.setVoltage(driveVoltage);
        steerMotor.setVoltage(steerVoltage);
        
        // Log voltages to smartdashboard
        SmartDashboard.putNumber(moduleName + " drive voltage", driveVoltage);
        SmartDashboard.putNumber(moduleName + " steer voltage", steerVoltage);
        
    }
    
    public void stop () {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }
    
}
