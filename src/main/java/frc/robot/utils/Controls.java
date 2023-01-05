package frc.robot.utils;

public class Controls {

    private static OI oi = OI.getInstance();

    public static boolean resetEncoders() {
        return oi.getXButtonDriver();
    }

    public static boolean zeroHeading() {
        return oi.getStartButtonDriver();
    }

    public static boolean zeroOdom() {
        return oi.getAButtonDriver();
    }

    public static boolean baseLock() {
        return oi.getBButtonDriver();
    }
    
}
