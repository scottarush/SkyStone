package org.firstinspires.ftc.teamcode.guidance;


import org.ejml.data.DMatrixRMaj;

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
 * Estimate vector xhat=[px,vx,vy,ax,ay,w_z,theta]^
 */
public class KalmanTracker {

    private static final int XHAT_PX_INDEX = 0;
    private static final int XHAT_PY_INDEX = 1;
    private static final int XHAT_OMEGAZ_INDEX = 4;
    private static final int XHAT_THETA_INDEX = 5;

    // Initial P covariance values
    final double SIGMA_POSITION = 0.1;
    final double VAR_POSITION = Math.pow(SIGMA_POSITION,2.0d);
    final double SIGMA_VELOCITY = 0.1;
    final double VAR_VELOCITY = Math.pow(SIGMA_VELOCITY,2.0d);
    final double SIGMA_ACCEL = 0.1;
    final double VAR_ACCEL = Math.pow(SIGMA_ACCEL,2.0d);
    final double SIGMA_W = 0.5;
    final double VAR_W = Math.pow(SIGMA_W,2.0d);
    final double SIGMA_THETA = 0.1;
    final double VAR_THETA = Math.pow(SIGMA_THETA,2.0d);

    private KalmanFilterOperations mFilter = null;

    // Process matrix
    private DMatrixRMaj A;
    // Process noise covariance matrix
    private DMatrixRMaj Q;
    // Measurment noise covariance matrix
    private DMatrixRMaj R;
    // Measurement matrix
    private DMatrixRMaj H;


    private double lx;
    private double ly;
    private double radius_wheel;

    public KalmanTracker(){

    }

    /**
     * Initializes the Kalman filter
     * @param T sampling interval in secons
     * @param px0 x coordinate of robot initial position in meters
     * @param py0 y coordinate of robot initial position in meters
     * @param theta0 initial heading in radians
     *               where 0=judge side, PI/2=right side, PI=audience side, 3PI/2=left side
     * @param lx  Lateral distance from wheel axle to imu center in meters
     * @param ly Lateral distance from wheel axle to imu center in meters
     * @param r radius of wheel
     */
    public void init(double T,double px0,double py0,double theta0,double lx,double ly,double r){
        // Save the lx and ly, as we'll need them when calculating the measurements
        this.lx = lx;
        this.ly = ly;
        radius_wheel = r;

        final double[][] A_MATRIX = {
                {1,0,T,0,0,0},
                {0,1,0,T,0,0},
                {0,0,1,0,0,0},
                {0,0,0,1,0,0},
                {0,0,0,0,1,0},
                {0,0,0,0,T,1} };

        //-------------------------------------------------------------
        // Q process noise covariance matrix
        //-------------------------------------------------------------

        final double[][] Q_MATRIX = {
                {VAR_POSITION,0,0,0,0,0},
                {0, VAR_POSITION,0,0,0,0},
                {0,0, VAR_VELOCITY,0,0,0},
                {0,0,0, VAR_VELOCITY,0,0},
                {0,0,0,0,VAR_W,0},
                {0,0,0,0,0,VAR_THETA} };

        //-------------------------------------------------------------
        // R measurement noise covariance matrix
        //-------------------------------------------------------------
        final double WHEEL_RADIUS = 0.050;
        final double SIGMA_W_WHL = 0.001;
        final double R2_VAR_W_WHL = Math.pow(WHEEL_RADIUS,2.0d) * Math.pow(SIGMA_W_WHL,2.0d);
        final double VAR_VX = R2_VAR_W_WHL;
        final double VAR_VY = R2_VAR_W_WHL;
        final double VAR_W_WHL = R2_VAR_W_WHL/((Math.pow(ly,2.0d) + Math.pow(ly,2.0d)));
        final double SIGMA_W_IMU = 0.1;
        final double VAR_W_IMU = Math.pow(SIGMA_W_IMU,2.0d);
        final double[][] R_MATRIX = {
                {VAR_VX,0,0,0},
                {0,VAR_VY,0,0},
                {0,0,VAR_W_WHL,0},
                {0,0,0,VAR_W_IMU}};

        final double[][] H_MATRIX = {
                {0,0,1,0,0,0},
                {0,0,0,1,0,0},
                {0,0,0,0,1,0},
                {0,0,0,0,0,1}};


        // Initialize constant matrices
        A = new DMatrixRMaj(A_MATRIX);
        R = new DMatrixRMaj(R_MATRIX);
        Q = new DMatrixRMaj(Q_MATRIX);
        H = new DMatrixRMaj(H_MATRIX);

        // Create and configure the filter
        mFilter = new KalmanFilterOperations();
        mFilter.configure(A,Q,H);

        // Initialize the state estimate vector to the supplied position and orientation
        DMatrixRMaj xhat = new DMatrixRMaj(new double[][] {{px0},{py0},{0d},{0d},{0d},{theta0}});
        // Initialize P to the process noise covariance matrix
        DMatrixRMaj p = new DMatrixRMaj(Q_MATRIX);

        mFilter.setState(xhat,p);
    }

    /**
     * Called at the sampling rate T to update the filter with a new measurement.
     * @param w_lf angular velocity of LF wheel in radians/sec
     * @param w_lr angular velocity of LR wheel in radians/sec
     * @param w_rf angular velocity of RF wheel in radians/sec
     * @param w_rr angular velocity of RR wheel in radians/sec
      * @param wz_imu z coordinate of imu measured angular velocity radians/sec
     */
    public void updateMeasurement(double w_lf,
                                  double w_lr,
                                  double w_rf,
                                  double w_rr,
                                  double wz_imu) {
        // Compute the robot velocity from the wheel velocities
        double rover4 = radius_wheel/4.0d;
        double vx = rover4*(w_lf+w_rf-w_lr-w_rr);
        double vy = rover4*(w_lf+w_rf+w_lr+w_rr);
        double wzw = rover4*(w_lf+w_rf+w_lr+w_rr)/(lx+ly);

        // Have to negate the wzw and wz_imu because we want to use left-handed orientation angles
        // instead of the right-handed angles produced by the measurements

        DMatrixRMaj z = new DMatrixRMaj(new double[][] {{vx},{vy},{-wzw},{-wz_imu}});

        // Do Kalman predict step
        mFilter.predict();
        // Do Kalman correct step
        mFilter.update(z,R);
    }
    /**
     * Returns the current estimated x position
     */
    public Double getEstimatedXPosition(){
        return mFilter.getState().get(XHAT_PX_INDEX);
    }
    /**
     * Returns the current estimated y position
     */
    public Double getEstimatedYPosition(){
        return mFilter.getState().get(XHAT_PY_INDEX);
    }

    /**
     *
     * @return Estimated heading from 0 to 2*PI in radians.  Note that this is a left-handed angle
     */
    public Double getEstimatedHeading() {
        return mFilter.getState().get(XHAT_THETA_INDEX, 0);
    }

    /**
     *
     * @return Estimated angular velocity in radians/sec.  Note that this is a left-handed angular velocity
     */
    public Double getEstimatedAngularVelocity(){
        return mFilter.getState().get(XHAT_OMEGAZ_INDEX,0);
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
