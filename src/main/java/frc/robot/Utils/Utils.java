package frc.robot.Utils;

public class Utils {
    public static double deadband(double x, double d) {
        return ((x > d)) || (x < -d) ? x : 0.0;
    }

}
