package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {

    public final CANSparkMax driveMotor;
    public final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final DutyCycleEncoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private SparkMaxPIDController drivingController;


    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new DutyCycleEncoder(absoluteEncoderId);
        absoluteEncoder.setDutyCycleRange(1.0/4096, 4095.0/4096);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
        driveMotor.setSmartCurrentLimit(40);
        turningMotor.setSmartCurrentLimit(50);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(Constants.motorDistanceMeters);
        driveEncoder.setVelocityConversionFactor(Constants.motorDistanceMeters / 60.0); // TODO
        turningEncoder.setPositionConversionFactor(Math.PI*2*7.0/150.0);
        turningEncoder.setVelocityConversionFactor(Math.PI*2*7.0/150.0);

        turningPidController = new PIDController(0.5, 0, 0); // TODO
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        drivingController = driveMotor.getPIDController();
        drivingController.setFeedbackDevice(driveEncoder);
        drivingController.setFF(0.2, 0);
        drivingController.setP(0.0161, 0);
        drivingController.setI(0.0, 0);
        drivingController.setD(0.0, 0);
        drivingController.setIZone(0.0, 0);
        drivingController.setOutputRange(-1, 1, 0);
    }

    public void setSpeed(double metersPerSecond){
        drivingController.setReference(metersPerSecond, ControlType.kVelocity, 0);
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRaw() { // revolutions
        return absoluteEncoder.getAbsolutePosition() * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public double getAbsoluteEncoderRad() {
        return getAbsoluteEncoderRaw() * Math.PI*2 + absoluteEncoderOffsetRad;
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state, boolean speedCheck, boolean optimize, boolean dashboard) {
        if(speedCheck && Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        if(optimize) state = SwerveModuleState.optimize(state, getState().angle);
        drivingController.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        if(dashboard) SmartDashboard.putString("Swerve[" + absoluteEncoder.getSourceChannel() + "] state", state.toString());
    }

    public void setDesiredState(SwerveModuleState state) {
        setDesiredState(state, true, true, true);
    }

    public void setRawPowers(double speed, double turn){
        driveMotor.set(speed);
        turningMotor.set(turn);
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}