package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.Controls;
import frc.robot.utils.OI;

public class SwerveTeleop extends CommandBase {
    private final SwerveSubsystem swerve;
    private OI oi;
    private SlewRateLimiter drive_limiterX = new SlewRateLimiter(1.5);
    private SlewRateLimiter drive_limiterY = new SlewRateLimiter(1.5);
    private SlewRateLimiter turn_limiter = new SlewRateLimiter(1.5);

    public SwerveTeleop() {
        swerve = SwerveSubsystem.getInstance();
        oi = OI.getInstance();
        addRequirements(swerve);
    }

    @Override
    public void initialize(){

        swerve.setCurrentState(swerve.FIELD_RELATIVE_CLASSIC);

    }

    @Override
    public void execute(){
        if(Controls.resetEncoders()) {
            swerve.resetEncoders();
        }

        if(Controls.zeroHeading()) {
            swerve.zeroHeading();
        }

        if(Controls.zeroOdom()){
            swerve.zeroOdometry();
        }

        if(Controls.baseLock()){
            swerve.setCurrentState(swerve.BASE_LOCK_ACTIVE);
        } else {
            swerve.setCurrentState(swerve.FIELD_RELATIVE_CLASSIC);
            swerve.fromController(-OI.precise(oi.getYLeftDriver()), -OI.precise(oi.getXLeftDriver()), -OI.precise(oi.getXRightDriver()));
        }

        
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
        swerve.setCurrentState(swerve.DISABLED);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }




}
