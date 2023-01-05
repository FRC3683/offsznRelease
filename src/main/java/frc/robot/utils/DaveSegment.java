// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class DaveSegment {

    public enum DaveSegmentType {
        WAIT,
        HOLONOMIC,
        TRACK_TARGET_POINT
    }

    private Trajectory traj;
    private double delay;
    private DavePoint targetPoint;
    private DaveSegmentType type;
    private double timeOffset;
    private final DavePoint initialPoint;
    private DavePoint finalPoint;


    private double maxAngularVel, maxAngularAcc;
    private SwerveSubsystem swerve;

    public static Pose2d defaultPose = new Pose2d(0, 0, new Rotation2d(0.0));

    public static final int WAYPOINT_WIDTH = 4;

    public DaveSegment(double delay, double timeOffset){
        this.delay = delay;
        type = DaveSegmentType.WAIT;
        this.timeOffset = timeOffset;
        targetPoint = new DavePoint();
        initialPoint = new DavePoint();
        finalPoint = new DavePoint();
        swerve = SwerveSubsystem.getInstance();
    }

    public DaveSegment(double maxVel, double maxAcc, double maxAngularVel, double maxAngularAcc, double vel0_mps, double vel1_mps, double head0_deg, double head1_deg, double timeOffset, String s) throws Exception{
        this(maxVel, maxAcc, maxAngularVel, maxAngularAcc, vel0_mps, vel1_mps, head0_deg, head1_deg, timeOffset, TrajectoryLoader.loadAutoTrajectory(s));
    }

    public DaveSegment(double maxVel, double maxAcc, double maxAngularVel, double maxAngularAcc, double vel0_mps, double vel1_mps, double head0_deg, double head1_deg, double timeOffset, double... points) throws Exception{
        int len = points.length;

        if(len < 2 * WAYPOINT_WIDTH){
            throw new Exception("Dave segment needs at least 2 points (" + (2 * WAYPOINT_WIDTH) + " doubles)");
        } else if (len % WAYPOINT_WIDTH != 0){
            throw new Exception("Dave segment needs multiple of " + WAYPOINT_WIDTH + " doubles");
        }

        this.delay = 0.0;
        type = DaveSegmentType.HOLONOMIC;
        this.timeOffset = timeOffset;
        this.maxAngularVel = maxAngularVel;
        this.maxAngularAcc = maxAngularAcc;
        swerve = SwerveSubsystem.getInstance();

        ControlVectorList waypoints = new ControlVectorList(len / WAYPOINT_WIDTH);
        
        
        targetPoint = new DavePoint();
        initialPoint = new DavePoint(points[0], points[1], points[2], points[3], vel0_mps, head0_deg);
        finalPoint = new DavePoint(points[len-4], points[len-3], points[len-2], points[len-1], vel1_mps, head1_deg);

        for(int i = 0; i < len; i+=WAYPOINT_WIDTH){
            double theta = Math.toRadians(points[i+2]), weight = points[i+3];
            double[] x = {points[i+0], weight * Math.cos(theta), 0}; //Temp place holders while I figure out ControlVector derivatives
            double[] y = {points[i+1], weight * Math.sin(theta), 0};
            waypoints.add(new ControlVector(x, y));
        }
        
        TrajectoryConfig trajConfig = new TrajectoryConfig(maxVel, maxAcc)
                                      .setStartVelocity(vel0_mps)
                                      .setEndVelocity(vel1_mps)
                                      .setKinematics(swerve.kin);

        traj = TrajectoryGenerator.generateTrajectory(waypoints, trajConfig);

    }

    public void setTargetPoint(double x, double y){
        this.type = DaveSegmentType.TRACK_TARGET_POINT;
        targetPoint = new DavePoint(x, y);
        finalPoint = new DavePoint(finalPoint.x, finalPoint.y, finalPoint.theta, finalPoint.weight, finalPoint.vel, Math.toDegrees(targetPoint.radiansFrom(finalPoint.x, finalPoint.y)));
    }

    public DavePoint getTargetPoint(){
        return targetPoint;
    }

    public boolean isWait(){
        return type == DaveSegmentType.WAIT;
    }

    public Command getCommand(){
        switch(type){
            case WAIT:
                return new WaitCommand(delay);
            case HOLONOMIC:
                return new DaveSwerveControllerCommand(this, true);
            case TRACK_TARGET_POINT:            
                return new DaveSwerveControllerCommand(this, false);
            default:
                return null;
        }
    }

    public Trajectory getTraj(){
        return traj;
    }

    public Pose2d getInitialPose(){
        switch(type){
            case WAIT:
                return defaultPose;
            case HOLONOMIC:
            case TRACK_TARGET_POINT:            
                return new Pose2d(traj.getInitialPose().getTranslation(), Rotation2d.fromDegrees(initialPoint.head));
            default:
                return null;
        }
    }

    public double getTotalTime(){
        switch(type){
            case WAIT:
                return timeOffset + delay;
            case HOLONOMIC:
            case TRACK_TARGET_POINT:            
                return timeOffset + traj.getTotalTimeSeconds();
            default:
                return timeOffset;
        }
    }

    public double getMaxAngularVel(){
        return maxAngularVel;
    }

    public double getMaxAngularAcc(){
        return maxAngularAcc;
    }

    public double initialHeading(){
        return initialPoint.head;
    }

    public double finalHeading(){
        return finalPoint.head;
    }

    public double initialVelocity(){
        return initialPoint.vel;
    }

    public double finalVelocity(){
        return finalPoint.vel;
    }

    public double initialX(){
        return initialPoint.x;
    }

    public double finalX(){
        return finalPoint.x;
    }

    public double initialY(){
        return initialPoint.y;
    }

    public double finalY(){
        return finalPoint.y;
    }

    public double initialTheta(){
        return initialPoint.theta;
    }

    public double finalTheta(){
        return finalPoint.theta;
    }

    public double initialWeight(){
        return initialPoint.weight;
    }

    public double finalWeight(){
        return finalPoint.weight;
    }

    public DavePoint initialPoint(){
        return initialPoint;
    }

    public DavePoint finalPoint(){
        return finalPoint;
    }
}
