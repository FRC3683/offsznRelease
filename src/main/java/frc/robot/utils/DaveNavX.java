package frc.robot.utils;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class DaveNavX extends AHRS {

    private boolean reversed;
    private AHRS navx;

    public DaveNavX(Port serial_port_id, boolean reversed, float offset) {
        navx = new AHRS(serial_port_id);
        navx.setAngleAdjustment(offset);
        this.reversed = reversed;
    }

    // degrees from -180 to 180
    public double getHeadingDeg() {
        return Math.IEEEremainder((navx.getAngle()) * (reversed ? -1 : 1), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeadingDeg());
    }
    
}