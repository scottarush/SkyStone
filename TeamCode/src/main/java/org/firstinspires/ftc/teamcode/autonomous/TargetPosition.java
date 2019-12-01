package org.firstinspires.ftc.teamcode.autonomous;

/**
 * Class encapsulate location data returned by getVuforiaLocation
 */
public class TargetPosition {
    public double x = 0d;
    public double y = 0d;
    public double z = 0d;
    public double heading = 0d;
    public boolean headingValid = false;
    public double orientation = 0d;
    public boolean valid = false;

    public String targetLabel = "";
    /**
     * Constructor with a valid location
     *
     * @param x
     * @param y
     * @param z
     * @param heading
     */
    public TargetPosition(String label,double x, double y, double z, double orientation, double heading) {
        valid = true;
        targetLabel = label;
        this.x = x;
        this.y = y;
        this.z = z;
        this.orientation = orientation;
        this.heading = heading;
    }
}
