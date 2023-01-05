// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double swerveGearRatio = 1/6.12;
    public static final double swerveCircumference = 0.1016 * Math.PI;
    public static final double motorDistanceMeters = swerveGearRatio * swerveCircumference;

    public static final double swerveMaxSpeed_mps = 2.0;
    public static final double swerveMaxTurn_radps = Math.PI * 2.0;

    public static final double wheelBase_meters = 0.5778; //front back delta = left right delta (distance between the center of wheels)



}
