package frc.robot.subsystems;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.AutoConstants;
import frc.robot.Constants;
import frc.robot.commands.SwerveTeleop;
import frc.robot.utils.DaveNavX;
import frc.robot.utils.DavePoint;
import frc.robot.utils.DaveSegment;
import frc.robot.utils.DaveSwerveController;
import frc.robot.utils.MathUtils;
import frc.robot.utils.OI;

public class SwerveSubsystem extends DaveSubsystem {
    private static SwerveSubsystem instance;

    public static SwerveSubsystem getInstance() {
        if (instance == null) {
            instance = new SwerveSubsystem();
        }
        return instance;
    }

    public final State DISABLED, FIELD_RELATIVE_CLASSIC, FIELD_RELATIVE_ABSOLUTE, AUTO, AUTO_TARGET_POINT, BASE_LOCK_PASSIVE, BASE_LOCK_ACTIVE;

    private final SwerveModule sm_fl, sm_fr, sm_br, sm_bl;
    
    private final DaveNavX navx;
    
    public final SwerveDriveKinematics kin;    
    private final SwerveDriveOdometry odom;
    public final DaveSwerveController con;
    
    private Field2d field;
    
    private double xspeed_mps;
    private double yspeed_mps;
    private double turnspeed_radps;

    private Rotation2d holonomicTarget = new Rotation2d();
    private DavePoint targetPoint;

    public boolean recording = false;
    public ArrayList<Pose2d> recordedPath = new ArrayList<Pose2d>();

    //Convention: X is forward positive, Y is left positive, angle is ccw positive
    public void fromController(double xspeed, double yspeed, double turnspeed){
        this.xspeed_mps = xspeed * Constants.swerveMaxSpeed_mps; //TODO: figure out scaling factors
        this.yspeed_mps = yspeed * Constants.swerveMaxSpeed_mps;
        this.turnspeed_radps = turnspeed * Constants.swerveMaxTurn_radps;
    }

    public void fromControllerRaw(double xspeed, double yspeed, double turnspeed){
        this.xspeed_mps = xspeed; //TODO: figure out scaling factors
        this.yspeed_mps = yspeed;
        this.turnspeed_radps = turnspeed;
    }

    private void WriteDashboardSwerveModule(SwerveModule module, String name) {
        SmartDashboard.putNumber(name + " drive pos", module.getDrivePosition());
        SmartDashboard.putNumber(name + " drive speed", module.getDriveVelocity());
        SmartDashboard.putNumber(name + " turn pos", module.getTurningPosition());
        SmartDashboard.putNumber(name + " turn pos mod tau", module.getTurningPosition()%(Math.PI*2));
        SmartDashboard.putNumber(name + " abs enc", module.getAbsoluteEncoderRad());
        SmartDashboard.putNumber(name + " abs enc raw", module.getAbsoluteEncoderRaw());
    }

    private void AddDashboardEntrySwerveModule(SwerveModule module, String name){
        AddDashboardEntryWrite(name + " drive pos", 0.0 , () -> { return module.getDrivePosition();});
        AddDashboardEntryWrite(name + " drive speed", 0.0 , () -> { return module.getDriveVelocity();});
        AddDashboardEntryWrite(name + " turn pos", 0.0 , () -> { return module.getTurningPosition();});
        AddDashboardEntryWrite(name + " turn pos mod tau", 0.0 , () -> { return module.getTurningPosition()%(Math.PI*2);});
        AddDashboardEntryWrite(name + " abs enc", 0.0 , () -> { return module.getAbsoluteEncoderRad();});
        AddDashboardEntryWrite(name + " abs enc raw", 0.0 , () -> { return module.getAbsoluteEncoderRaw();});
    }

    public void resetEncoders() {
        sm_fl.resetEncoders();
        sm_fr.resetEncoders();
        sm_bl.resetEncoders();
        sm_br.resetEncoders();
    }

    private void formX() {
        sm_fl.setDesiredState(MathUtils.swerveModuleState(45, 0), false, true, true);
        sm_fr.setDesiredState(MathUtils.swerveModuleState(-45, 0), false, true, true);
        sm_bl.setDesiredState(MathUtils.swerveModuleState(-45, 0), false, true, true);
        sm_br.setDesiredState(MathUtils.swerveModuleState(45, 0), false, true, true);
    }

    private SwerveSubsystem() {
        super("SWERVY BOI");

        sm_fl = new SwerveModule(
            6,     //drive id
            2,      //turn id
            false,  // drive reversed?
            true,  //turn reversed?
            9,      //abs enc id
            -2.222,  //abs enc offset
            false    //abs enc reversed?
        );

        sm_fr  = new SwerveModule(
            7,     //drive id
            3,      //turn id
            false,  // drive reversed?
            true,  //turn reversed?
            5,      //abs enc id
            -0.105,  //abs enc offset
            false    //abs enc reversed?
        );

        sm_br  = new SwerveModule(
            8,     //drive id
            4,      //turn id
            false,  // drive reversed?
            true,  //turn reversed?
            8,      //abs enc id
            -1.364,  //abs enc offset
            false    //abs enc reversed?
        );

        sm_bl  = new SwerveModule(
            9,     //drive id
            5,      //turn id
            true,  // drive reversed?
            true,  //turn reversed?
            4,      //abs enc id
            -0.446,  //abs enc offset
            false    //abs enc reversed?
        );

        kin = new SwerveDriveKinematics(
            new Translation2d(Constants.wheelBase_meters * 0.5, Constants.wheelBase_meters * 0.5),   // fl
            new Translation2d(Constants.wheelBase_meters * 0.5, -Constants.wheelBase_meters * 0.5),  // fr
            new Translation2d(-Constants.wheelBase_meters * 0.5, Constants.wheelBase_meters * 0.5), // bl
            new Translation2d(-Constants.wheelBase_meters * 0.5, -Constants.wheelBase_meters * 0.5)   // br
        );

        con = new DaveSwerveController(AutoConstants.kPXController, 0.0, 0.0,
                                       AutoConstants.kPYController, 0.0, 0.0,
                                       AutoConstants.kPThetaController, 0.0, 0.0);
        
        navx = new DaveNavX(Port.kUSB1, true, 0);
        odom = new SwerveDriveOdometry(kin, new Rotation2d(0));
        field = new Field2d();

        holonomicTarget = new Rotation2d();
        targetPoint = new DavePoint(0, 0);

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                resetEncoders();
                zeroHeading();
                zeroOdometry();
            } catch (Exception e) {
            }
        }).start();

        DISABLED = new State("DISABLED") {
            @Override
            public void init() {
                stopModules();
            }

            @Override
            public void periodic() {
                
            }
        };

        FIELD_RELATIVE_CLASSIC = new State("FIELD_RELATIVE_CLASSIC") {
            @Override
            public void init() {
                
            }

            @Override
            public void periodic() {
                swerveDrive(xspeed_mps, yspeed_mps, turnspeed_radps);
            }
        };

        FIELD_RELATIVE_ABSOLUTE = new State("FIELD_RELATIVE_ABSOLUTE") {
            @Override
            public void init() {
                
            }

            @Override
            public void periodic() {
                swerveDrive(xspeed_mps, yspeed_mps, turnspeed_radps);
            }
        };

        AUTO = new State("AUTO") {
            @Override
            public void init() {
                
            }

            @Override
            public void periodic() {
                
            }
        };

        AUTO_TARGET_POINT = new State("AUTO_TARGET_POINT") {
            @Override
            public void init() {
                
            }

            @Override
            public void periodic() {
                
                setTargetHeadingRads(targetPoint.radiansFrom(getPose().getX(), getPose().getY()));
            }
        };

        BASE_LOCK_PASSIVE = new State("BASE_LOCK_PASSIVE"){
            @Override
            public void init(){

            }

            @Override
            public void periodic() {
                formX();
            }
        };

        BASE_LOCK_ACTIVE = new State("BASE_LOCK_ACTIVE"){
            Pose2d pose;
            DaveSwerveController pid = new DaveSwerveController(
                10, 0, 0, // x
                10, 0, 0, // y
                0.1, 0, 0 // theta
            );

            @Override
            public void init(){
                pose = getPose();
            }

            @Override
            public void periodic() {
                double dx = getPose().getX() - pose.getX();
                double dy = getPose().getY() - pose.getY();
                
                if(MathUtils.withinCircle(dx, dy, 0.03)) {
                    formX();
                } else {
                    swerveDrive(
                        pid.getXCon().calculate(dx),
                        pid.getYCon().calculate(dy),
                        0//pid.getXCon().calculate(measurement)
                    );
                }
            }
        };

        setCurrentState(DISABLED);  

        AddDashboardEntryWrite("NAVX", 0.0 , () -> {
            return navx.getHeadingDeg();
        });

        AddDashboardEntryWrite("Magnitude", 0.0 , () -> {
            return getMagnitude();
        });

        AddDashboardEntryWrite("X vel", 0.0 , () -> {
            return getVelX();
        });

        AddDashboardEntryWrite("Y vel", 0.0 , () -> {
            return getVelY();
        });

        AddDashboardEntryWrite("NAVX Rotating", false , () -> {
            return navx.isRotating();
        });

        AddDashboardEntryWrite("xspeed_mps", 0.0 , () -> {
            return xspeed_mps;
        });

        AddDashboardEntryWrite("yspeed_mps", 0.0 , () -> {
            return -yspeed_mps;
        });

        AddDashboardEntryWrite("turnspeed_radps", 0.0 , () -> {
            return turnspeed_radps;
        });

        AddDashboardEntrySwerveModule(sm_fr, "FR");
        AddDashboardEntrySwerveModule(sm_fl, "FL");
        AddDashboardEntrySwerveModule(sm_bl, "BL");
        AddDashboardEntrySwerveModule(sm_br, "BR");

        AddDashboardEntryState(DISABLED);

        SmartDashboard.putData("Field", field);
    }

    public Pose2d getPose() {
        return odom.getPoseMeters();
    }

    public Rotation2d getHeading(){
        return Rotation2d.fromDegrees(navx.getHeadingDeg());
    }

    public void zeroHeading() {
        navx.reset();
    }

    public void resetOdometry(Pose2d pose) {
        odom.resetPosition(pose, navx.getRotation2d());
    }

    public void zeroOdometry() {
        resetOdometry(new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0.0)));
    }

    public Rotation2d getTargetHeading(){
        return holonomicTarget;
    }
    
    public void setTargetHeading(double degrees){
        holonomicTarget = Rotation2d.fromDegrees(degrees);
    }

    public void setTargetHeadingRads(double radians){
        holonomicTarget = new Rotation2d(radians);
    }

    public void setTargetPoint(double x, double y){
        targetPoint.x = x;
        targetPoint.y = y;
    }

    public DavePoint getTargetPoint(){
        return targetPoint;
    }

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    public void swerveDrive(double xSpeed, double ySpeed, double turnSpeed){
        //ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, ySpeed, turnSpeed, new Rotation2d(0.0));
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, navx.getRotation2d());
        SwerveModuleState[] moduleStates = kin.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return chassisSpeeds;
    }

    public ChassisSpeeds getTargetChassisSpeeds(Pose2d currentPose, Trajectory.State desiredState, Rotation2d angleRef){
        return con.calculate(currentPose, desiredState.poseMeters, desiredState.velocityMetersPerSecond, angleRef);
    }

    public double getMagnitude() {
        return Math.sqrt(MathUtils.sq(chassisSpeeds.vxMetersPerSecond)+MathUtils.sq(chassisSpeeds.vyMetersPerSecond));
    }

    public double getVelX(){
        return chassisSpeeds.vxMetersPerSecond;
    }

    public double getVelY(){
        return chassisSpeeds.vyMetersPerSecond;
    }

    @Override
    public void subsystemPeriodic() {
        Pose2d pose = odom.update(navx.getRotation2d(), sm_fl.getState(), sm_fr.getState(), sm_bl.getState(), sm_br.getState());
        // pose = pose.transformBy(new Transform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(-90.0)));
        field.setRobotPose(pose);
        /*WriteDashboardSwerveModule(sm_fr, "FR");
        WriteDashboardSwerveModule(sm_fl, "FL");
        WriteDashboardSwerveModule(sm_bl, "BL");
        WriteDashboardSwerveModule(sm_br, "BR");
        
        
        SmartDashboard.putString("State", getCurrentState().getName());

        SmartDashboard.putNumber("xspeedboiiii", xspeed_mps);*/

        SmartDashboard.putNumber("navxangel", navx.getHeadingDeg());
        SmartDashboard.putNumber("navx vel x", navx.getVelocityX());
        SmartDashboard.putNumber("navx vel y", navx.getVelocityY());
        SmartDashboard.putNumber("navx vel z", navx.getVelocityZ());
        
        if(recording) {
            recordedPath.add(getPose());
        }
    }

    public void stopModules() {
        sm_fl.stop();
        sm_fr.stop();
        sm_bl.stop();
        sm_br.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.swerveMaxSpeed_mps); // TODO
        sm_fl.setDesiredState(desiredStates[0]);
        sm_fr.setDesiredState(desiredStates[1]);
        sm_bl.setDesiredState(desiredStates[2]);
        sm_br.setDesiredState(desiredStates[3]);
    }
}
