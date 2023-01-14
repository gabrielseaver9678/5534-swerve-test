package frc.robot.swerve;

import com.ctre.phoenix.sensors.CANCoder;

public class ResettableEncoder {
    
    private final CANCoder canCoder;
    private double offset;
    
    public ResettableEncoder (int cancoderId) {
        this.canCoder = new CANCoder(cancoderId);
        offset = getOffset();
    }
    
    public double getAngle () {
        // TODO: Fix this
        return canCoder.getAbsolutePosition() + offset;
    }
    
    private double getOffset () {
        return canCoder.configGetCustomParam(0);
    }
    
}