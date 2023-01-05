// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.AutoConstants;

/** Add your docs here. */
public class DaveSwerveController {
    private Pose2d poseError;
    private Rotation2d rotationError;
    private Pose2d poseTolerance;
    private boolean enabled;
    private boolean firstRun;

    private final PIDController xCon;
    private final PIDController yCon;
    private final ProfiledPIDController thetaCon;

    public DaveSwerveController(double xP, double xI, double xD, double yP, double yI, double yD, double thetaP, double thetaI, double thetaD){
        xCon = new PIDController(xP, xI, xD);
        yCon = new PIDController(yP, yI, yD);
        thetaCon = new ProfiledPIDController(thetaP, thetaI, thetaD, AutoConstants.kThetaControllerConstraints);
        thetaCon.enableContinuousInput(-Math.PI, Math.PI);
        poseError = new Pose2d();
        rotationError = new Rotation2d();
        poseTolerance = new Pose2d();
        enabled = true;
        firstRun = true;
    }

    public boolean atReference() {
        final var eTranslate = poseError.getTranslation();
        final var eRotate = rotationError;
        final var tolTranslate = poseTolerance.getTranslation();
        final var tolRotate = poseTolerance.getRotation();
        return Math.abs(eTranslate.getX()) <= tolTranslate.getX()
            && Math.abs(eTranslate.getY()) <= tolTranslate.getY()
            && Math.abs(eRotate.getRadians()) <= tolRotate.getRadians();
      }

    public void setTolerance(double xtol, double ytol, double thetaTol_deg) {
        poseTolerance = new Pose2d(xtol, ytol, Rotation2d.fromDegrees(thetaTol_deg));
    }

    public void setEnabled(boolean enabled){
        this.enabled = enabled;
    }

    public void setThetaContraints(double max_radps, double max_radps2){
        thetaCon.setConstraints(new Constraints(max_radps, max_radps2));
    }

    public PIDController getXCon(){
        return xCon;
    }

    public PIDController getYCon(){
        return yCon;
    }

    public ProfiledPIDController getThetaCon(){
        return thetaCon;
    }

    public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters, Rotation2d angleRef){
        // If this is the first run, then we need to reset the theta controller to the current pose's
        // heading.
        if (firstRun) {
            thetaCon.reset(currentPose.getRotation().getRadians());
            firstRun = false;
        }
  
        // Calculate feedforward velocities (field-relative).
        double xFF = linearVelocityRefMeters * poseRef.getRotation().getCos();
        double yFF = linearVelocityRefMeters * poseRef.getRotation().getSin();
        double thetaFF = thetaCon.calculate(currentPose.getRotation().getRadians(), angleRef.getRadians());
        
        poseError = poseRef.relativeTo(currentPose);
        rotationError = angleRef.minus(currentPose.getRotation());
        
        if (!enabled) {
          return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, thetaFF, currentPose.getRotation());
        }
      
        // Calculate feedback velocities (based on position error).
        double xFeedback = xCon.calculate(currentPose.getX(), poseRef.getX());
        double yFeedback = yCon.calculate(currentPose.getY(), poseRef.getY());
      
        // Return next output.
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback, yFF + yFeedback, thetaFF, currentPose.getRotation());
    }

}
