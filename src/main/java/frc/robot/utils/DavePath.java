// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.DoubleStream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class DavePath extends SequentialCommandGroup{

    protected SwerveSubsystem swerve;

    Timer timer;
    OI oi;
    private ArrayList<DaveSegment> segs;
    protected boolean loaded = true;
    private DavePoint initialPoint;
    private DavePoint finalPoint;

    public DavePath(){
        swerve = SwerveSubsystem.getInstance();
        addRequirements(swerve);
        oi = OI.getInstance();
        segs = new ArrayList<>();
        timer = new Timer();
    }

    @Override
    public void initialize() {
        super.initialize();
        //swerve.setCurrentState(swerve.AUTO);
        if(initialPoint == null){
            swerve.resetOdometry(DaveSegment.defaultPose);
        }else{
            swerve.resetOdometry(new Pose2d(initialPoint.x, initialPoint.y, Rotation2d.fromDegrees(initialPoint.head))); 
            //swerve.resetOdometry(new Pose2d(initialPoint.x, initialPoint.y, swerve.getHeading())); 
        }
        timer.start();
    }

    public DavePath addHolonomicSegment(double maxVel, double maxAcc, double maxAngularVel, double maxAngularAcc, double vel0_mps, double vel1_mps, double head0_deg, double head1_deg, double... points) throws Exception{
        DaveSegment s = new DaveSegment(maxVel, maxAcc, maxAngularVel, maxAngularAcc, vel0_mps, vel1_mps, head0_deg, head1_deg, getTotalTime(), points);
        segs.add(s);
        addCommands(s.getCommand());
        if(initialPoint == null){
            initialPoint = s.initialPoint();
        }
        finalPoint = s.finalPoint();
        return this;
    }

    public DavePath appendHolonomicSegment(double maxVel, double maxAcc, double maxAngularVel, double maxAngularAcc, double vel_mps, double head_deg, double... points) throws Exception{
        if(finalPoint == null){
            throw new Exception("Append to what? Look at yourself, trying to attach endpoints to segments that don't exist. Maybe go attach some meaning to your life.");
        }
        double[] newpoints = DoubleStream.concat(Arrays.stream(finalPoint.segpoint), Arrays.stream(points)).toArray();
        DaveSegment s = new DaveSegment(maxVel, maxAcc, maxAngularVel, maxAngularAcc, finalPoint.vel, vel_mps, finalPoint.head, head_deg, getTotalTime(), newpoints);
        segs.add(s);
        addCommands(s.getCommand());
        finalPoint = s.finalPoint();
        return this;
    }    

    public DavePath addPathWeaverSegment(double maxVel, double maxAcc, double maxAngularVel, double maxAngularAcc, double vel0_mps, double vel1_mps, double head0_deg, double head1_deg, String traj) throws Exception{
        DaveSegment s = new DaveSegment(maxVel, maxAcc, maxAngularVel, maxAngularAcc, vel0_mps, vel1_mps, head0_deg, head1_deg, getTotalTime(), traj);
        segs.add(s);
        addCommands(s.getCommand());
        if(initialPoint == null){
            initialPoint = s.initialPoint();
        }
        finalPoint = s.finalPoint();
        return this;
    }  
    
    public DavePath addTargetTrackingSegment(double maxVel, double maxAcc, double maxAngularVel, double maxAngularAcc, double vel0_mps, double vel1_mps, double head0_deg, double targx, double targy, double... points) throws Exception{
        DaveSegment s = new DaveSegment(maxVel, maxAcc, maxAngularVel, maxAngularAcc, vel0_mps, vel1_mps, head0_deg, head0_deg, getTotalTime(), points);
        s.setTargetPoint(targx, targy);
        segs.add(s);
        addCommands(s.getCommand());
        if(initialPoint == null){
            initialPoint = s.initialPoint();
        }
        finalPoint = s.finalPoint();
        return this;
    }

    public DavePath appendTargetTrackingSegment(double maxVel, double maxAcc, double maxAngularVel, double maxAngularAcc, double vel_mps, double targx, double targy, double... points) throws Exception{
        if(finalPoint == null){
            throw new Exception("Append to what? Look at yourself, trying to attach endpoints to segments that don't exist. Maybe go attach some meaning to your life.");
        }
        double[] newpoints = DoubleStream.concat(Arrays.stream(finalPoint.segpoint), Arrays.stream(points)).toArray();
        DaveSegment s = new DaveSegment(maxVel, maxAcc, maxAngularVel, maxAngularAcc, finalPoint.vel, vel_mps, finalPoint.head, finalPoint.head, getTotalTime(), newpoints);
        s.setTargetPoint(targx, targy);
        segs.add(s);
        addCommands(s.getCommand());
        finalPoint = s.finalPoint();
        return this;
    } 

    public DavePath addTargetTrackingSegment(double maxVel, double maxAcc, double maxAngularVel, double maxAngularAcc, double vel0_mps, double vel1_mps, double head0_deg, double targx, double targy, String traj) throws Exception{
        DaveSegment s = new DaveSegment(maxVel, maxAcc, maxAngularVel, maxAngularAcc, vel0_mps, vel1_mps, head0_deg, head0_deg, getTotalTime(), traj);
        s.setTargetPoint(targx, targy);
        segs.add(s);
        addCommands(s.getCommand());
        if(initialPoint == null){
            initialPoint = s.initialPoint();
        }
        finalPoint = s.finalPoint();
        return this;
    }

    public DavePath addWaitSegment(double time) {
        DaveSegment s = new DaveSegment(time, getTotalTime());
        segs.add(s);
        addCommands(s.getCommand());
        return this;
    }
    

    public double getTotalTime(){
        if(segs.isEmpty()) return 0.0;
        return segs.get(segs.size() - 1).getTotalTime();
    }

    public double time() {
        return timer.get();
    }

    public void reset() {
        timer.reset();
    }

    public boolean after(double seconds) {
        return timer.hasElapsed(seconds);
    }

    public boolean before(double seconds) {
        return !timer.hasElapsed(seconds);
    }

    public boolean between(double earlier, double later) {
        return after(earlier) && before(later);
    }

    public double alpha(double earlier, double later) {
        double t = timer.get();
        if (t <= earlier)
            return 0.0;
        if (t >= later)
            return 1.0;
        return MathUtils.unlerp(earlier, later, t);
    }

    protected void trackHolonomic(){
        swerve.setCurrentState(swerve.AUTO);
    }

    protected void trackTargetPoint(){
        swerve.setCurrentState(swerve.AUTO_TARGET_POINT);
    }

    protected double segEnd(int i){
        return segs.get(i).getTotalTime();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || oi.driverInput();
    }

    @Override
    public void end(boolean interuppted) {
        oi.rumbleDriverFor(0.2, 0.2, 2.0);
        swerve.setCurrentState(swerve.DISABLED);
        super.end(interuppted);
    }

}
