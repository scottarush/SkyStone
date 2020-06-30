package org.firstinspires.ftc.teamcode.filter;

import java.util.Vector;

public interface IKalmanTracker {
    /**
     * Initializes the Kalman filter
     * @param T sampling interval in seconds
     * @param px0 x coordinate of robot initial position in feet
     * @param py0 y coordinate of robot initial position in feet
     * @param theta0 initial heading in radians
     *               where 0=right side, PI/2=judge side, PI=left side, 3PI/2=audience side.
     * @param lx  Lateral distance from wheel axle to imu center
     * @param ly Lateral distance from wheel axle to imu center
     * @param r radius of wheel
     */
    public void init(double T,double px0,double py0,double theta0,double lx,double ly,double r);

    /**
     * Called at the sampling rate T to update the filter with a new measurement.
     * @param w_lf angular velocity of LF wheel in radians/sec
     * @param w_lr angular velocity of LR wheel in radians/sec
     * @param w_rf angular velocity of RF wheel in radians/sec
     * @param w_rr angular velocity of RR wheel in radians/sec
     * @param ax_imu x coordinate of imu measured acceleration
     * @param ay_imu y coordinate of imu measured acceleration
     * @param wz_imu z coordinate of imu measured angular velocity
     */
    public void updateMeasurement(double w_lf,
                                  double w_lr,
                                  double w_rf,
                                  double w_rr,
                                  double ax_imu,
                                  double ay_imu,
                                  double wz_imu) ;
    /**
     * Called to set the position x coordinate when known from objects in the field
     * @param px updated x position
     */
    public void setKnownXPosition(double px);

    /**
     * Called to update the position y coordinate when known from objects in the field
     * @param py updated y position
     */
    public void setKnownYPosition(double py);

    /**
     * Called to update the heading when detected from objects in the field
     * @param theta updated heading angle
     */
    public void setKnownHeading(double theta);

    /**
     * Returns the current estimated position as a Vector with {0,0} in the center
     * of the playing field and positive y toward the judges.
     * index 0 = estimated px
     * index 1 = estimated py
     */
    public Vector<Double> getEstimatedPosition();

    public Double getEstimatedHeading();

}
