import edu.wpi.first.wpilibj.PS4Controller;


public class InterpolatedPS4Gamepad extends PS4Controller {

    static double deadZoneThreshold;
    static double fullThrottleThreshold;

    public InterpolatedPS4Gamepad(int port) {
        super(port);

        deadZoneThreshold = 0.05;
        fullThrottleThreshold = 0.9;
    }

    public static boolean inDeadZone(double axis) {
        return (axis > -deadZoneThreshold) && (axis < deadZoneThreshold);
    }

    public static boolean isCeiling(double axis) {
        return axis <= -fullThrottleThreshold || axis >= fullThrottleThreshold;
    }

    public double interpolatedLeftYAxis() {
        if (Math.abs(this.getLeftY()) <= 0.05)
            return 0.0;
        return ((Math.sin(this.getLeftY())) * 1.2);
    }

    public double interpolatedLeftXAxis() {
        if (Math.abs(this.getLeftX()) <= 0.05)
            return 0.0;
        return ((Math.sin(this.getLeftX())) * 1.2);
    }

    public double interpolatedRightXAxis() {
        if (Math.abs(this.getRightX()) <= 0.05)
            return 0.0;
        return -(Math.sin(this.getRightX()) * .8);
    }
}


