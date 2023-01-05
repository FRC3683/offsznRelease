// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class DaveSwerveControllerCommand extends CommandBase{    
    private final Timer timer = new Timer();
    private final DaveSegment seg;
    private final SwerveSubsystem swerve;
    private boolean holoTarget;

    public DaveSwerveControllerCommand(DaveSegment seg, boolean holoTarget){
        this.holoTarget = holoTarget;
        this.seg = seg;
        swerve = SwerveSubsystem.getInstance();
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        if(holoTarget){
            swerve.setCurrentState(swerve.AUTO);
            swerve.setTargetHeading(seg.finalHeading());
        } else{            
            swerve.setCurrentState(swerve.AUTO_TARGET_POINT);
            swerve.setTargetPoint(seg.getTargetPoint().x, seg.getTargetPoint().y);
        }
        swerve.con.setThetaContraints(seg.getMaxAngularVel(), seg.getMaxAngularAcc());
    }

    @Override
    public void execute() {
        double curTime = timer.get();
        var desiredState = seg.getTraj().sample(curTime);
        var targetChassisSpeeds = swerve.getTargetChassisSpeeds(swerve.getPose(), desiredState, swerve.getTargetHeading());
        var targetModuleStates = swerve.kin.toSwerveModuleStates(targetChassisSpeeds);
        swerve.setModuleStates(targetModuleStates);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(seg.getTraj().getTotalTimeSeconds());
    }

}
