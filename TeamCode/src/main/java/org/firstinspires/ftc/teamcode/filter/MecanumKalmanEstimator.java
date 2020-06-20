package org.firstinspires.ftc.teamcode.filter;


import org.ejml.data.DMatrixRMaj;

import java.util.Vector;

/**
 * Encapsulates a Kalman filter state estimator for a mecanum wheel robot.
 * The filter estimates position (returned by getEstimatedPosition()) and
 * heading (getEstimatedHeading()).
 *
 * The init method must be called with the initial position and heading of the resting
 * robot.
 *
 * When position or heading is known from additional inputs, call the
 * setKnownXXX functions to update the estimate.
 *
 * Measurement vector z=[vx,vy,w_zw,ax,ay,w_zi]^T
 * Estimate vector xhat=[px,vx,vy,ax,ay,w_z,theta]^T
 */
public class MecanumKalmanEstimator {

    private static final int XHAT_PX_INDEX = 0;
    private static final int XHAT_PY_INDEX = 1;
    private static final int XHAT_THETA_INDEX = 7;

    // Initial P covariance values
    final double SIGMA_POSITION = 0.1;
    final double VAR_POSITION = Math.pow(SIGMA_POSITION,2.0d);
    final double SIGMA_VELOCITY = 0.1;
    final double VAR_VELOCITY = Math.pow(SIGMA_VELOCITY,2.0d);
    final double SIGMA_ACCEL = 0.1;
    final double VAR_ACCEL = Math.pow(SIGMA_ACCEL,2.0d);
    final double SIGMA_W = 0.1;
    final double VAR_W = Math.pow(SIGMA_W,2.0d);
    final double SIGMA_THETA = 0.1;
    final double VAR_THETA = Math.pow(SIGMA_THETA,2.0d);

    private KalmanFilterOperations mFilter = null;

    // Process matrix
    private DMatrixRMaj A = new DMatrixRMaj(8,8);
    // Process noise covariance matrix
    private DMatrixRMaj Q = new DMatrixRMaj(8,8);
    // Measurment noise covariance matrix
    private DMatrixRMaj R = new DMatrixRMaj(6,6);
    // Measurement matrix
    private DMatrixRMaj H = new DMatrixRMaj(6,8);


    private double lx;
    private double ly;
    private double radius_wheel;

    public MecanumKalmanEstimator(){

    }

    /**
     * Initializes the Kalman filter
     * @param T sampling interval in secons
     * @param px0 x coordinate of robot initial position in feet
     * @param py0 y coordinate of robot initial position in feet
     * @param theta0 initial heading in radians
     *               where 0=right side, PI/2=judge side, PI=left side, 3PI/2=audience side.
     * @param lx  Lateral distance from wheel axle to imu center
     * @param ly Lateral distance from wheel axle to imu center
     * @param r radius of wheel
     */
    public void init(double T,double px0,double py0,double theta0,double lx,double ly,double r){
        // Save the lx and ly, as we'll need them when calculating the measurements
        this.lx = lx;
        this.ly = ly;
        radius_wheel = r;

        final double[][] A_MATRIX = {
                {1,0,T,0,0,0,0,0},
                {0,1,0,T,0,0,0,0},
                {0,0,1,0,T,0,0,0},
                {0,0,0,1,0,T,0,0},
                {0,0,0,0,1,0,0,0},
                {0,0,0,0,0,1,0,0},
                {0,0,0,0,0,0,1,0},
                {0,0,0,0,0,0,T,0} };

        //-------------------------------------------------------------
        // Q process noise covariance matrix
        //-------------------------------------------------------------

        final double[][] Q_MATRIX = {
                {VAR_POSITION,0,0,0,0,0,0,0},
                {0, VAR_POSITION,0,0,0,0,0,0},
                {0,0, VAR_VELOCITY,0,0,0,0,0},
                {0,0,0, VAR_VELOCITY,0,0,0,0},
                {0,0,0,0, VAR_ACCEL,0,0,0},
                {0,0,0,0,0, VAR_ACCEL,0,0},
                {0,0,0,0,0,0,VAR_W,0},
                {0,0,0,0,0,0,0,VAR_THETA} };

        //-------------------------------------------------------------
        // R measurement noise covariance matrix
        //-------------------------------------------------------------
        final double WHEEL_RADIUS = 0.050;
        final double SIGMA_W_WHL = 0.001;
        final double R2_VAR_W_WHL = Math.pow(WHEEL_RADIUS,2.0d) * Math.pow(SIGMA_W_WHL,2.0d);
        final double VAR_VX = R2_VAR_W_WHL;
        final double VAR_VY = R2_VAR_W_WHL;
        final double VAR_W_WHL = R2_VAR_W_WHL/((Math.pow(ly,2.0d) + Math.pow(ly,2.0d)));
        final double SIGMA_A_IMU = 0.1;
        final double VAR_A_IMU = Math.pow(SIGMA_A_IMU,2.0d);
        final double SIGMA_W_IMU = 0.1;
        final double VAR_W_IMU = Math.pow(SIGMA_W_IMU,2.0d);
        final double[][] R_MATRIX = {
                {VAR_VX,0,0,0,0,0},
                {0,VAR_VY,0,0,0,0},
                {0,0,VAR_W_WHL,0,0,0},
                {0,0,0,VAR_A_IMU,0,0},
                {0,0,0,0,VAR_A_IMU,0},
                {0,0,0,0,0,VAR_W_IMU}};

        final double[][] H_MATRIX = {
                {0,0,1,0,0,0,0,0},
                {0,0,0,1,0,0,0,0},
                {0,0,0,0,0,0,1,0},
                {0,0,0,0,1,0,0,0},
                {0,0,0,0,0,1,0,0},
                {0,0,0,0,0,0,1,0}};


        // Initialize constant matrices
        A = new DMatrixRMaj(A_MATRIX);
        R = new DMatrixRMaj(R_MATRIX);
        Q = new DMatrixRMaj(Q_MATRIX);
        H = new DMatrixRMaj(H_MATRIX);

        // configure the filter
        mFilter.configure(A,Q,H);

        //------------------------------
        // Now initialize the state
        //------------------------------

        // Initialize the state estimate vector to the supplied position and orientation
        DMatrixRMaj xhat = new DMatrixRMaj(new double[][] {{px0},{py0},{0d},{0d},{0d},{0d},{0d},{theta0}});
        // Initialize P to the process noise covariance matrix
        DMatrixRMaj p = new DMatrixRMaj(Q_MATRIX);

        mFilter.setState(xhat,p);
    }


    /**
     * Called at the sampling rate T to update the filter with a new measurement.
     * @param w_lf angular velocity of LF wheel
     * @param w_lr angular velocity of LR wheel
     * @param w_rf angular velocity of RF wheel
     * @param w_rr angular velocity of RR wheel
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
                                  double wz_imu) {
        double rover4 = radius_wheel/4.0d;
        double vx = rover4*(w_lf+w_rf-w_lr-w_rr);
        double vy = rover4*(w_lf+w_rf+w_lr+w_rr);
        double wzw = rover4*(w_lf+w_rf+w_lr+w_rr)/(lx+ly);
        DMatrixRMaj z = new DMatrixRMaj(new double[][] {{vx},{vy},{wzw},{ax_imu},{ay_imu},{wz_imu}});

        // Do Kalman predict step
        mFilter.predict();
        // Do Kalman correct step
        mFilter.update(z,R);
    }
   /**
     * Returns the current estimated position as a Vector with {0,0} in the center
     * of the playing field and positive y toward the judges.
     * index 0 = estimated px
     * index 1 = estimated py
     */
    public Vector<Double> getEstimatedPosition(){
        Vector<Double> location = new Vector<Double>();
        location.add(mFilter.getState().get(XHAT_PX_INDEX,0));
        location.add(mFilter.getState().get(XHAT_PY_INDEX,0));
        return location;
    }
    public Double getEstimatedHeading(){
        return mFilter.getState().get(XHAT_THETA_INDEX,0);
    }
    /**
     * Called to set the position x coordinate when known from objects in the field
     * @param px updated x position
     */
    public void setKnownXPosition(double px){
        DMatrixRMaj xhat = new DMatrixRMaj(mFilter.getState());
        xhat.set(XHAT_PX_INDEX,0,px);
        // And reset the px variance to the default
        DMatrixRMaj p = new DMatrixRMaj(mFilter.getCovariance());
        p.set(XHAT_PX_INDEX,XHAT_PX_INDEX, VAR_POSITION);
        mFilter.setState(xhat,p);
    }
    /**
     * Called to update the position y coordinate when known from objects in the field
     * @param py updated y position
     */
    public void setKnownYPosition(double py){
        DMatrixRMaj xhat = new DMatrixRMaj(mFilter.getState());
        xhat.set(XHAT_PY_INDEX,0,py);
        // And reset the px variance to the default
        DMatrixRMaj p = new DMatrixRMaj(mFilter.getCovariance());
        p.set(XHAT_PY_INDEX,XHAT_PY_INDEX, VAR_POSITION);
        mFilter.setState(xhat,p);
    }
    /**
     * Called to update the heading when detected from objects in the field
     * @param theta updated heading angle
     */
    public void setKnownHeading(double theta){
        DMatrixRMaj xhat = new DMatrixRMaj(mFilter.getState());
        xhat.set(XHAT_THETA_INDEX,0,theta);
        // And reset the px variance to the default
        DMatrixRMaj p = new DMatrixRMaj(mFilter.getCovariance());
        p.set(XHAT_THETA_INDEX,XHAT_THETA_INDEX, VAR_THETA);
        mFilter.setState(xhat,p);
    }

}
