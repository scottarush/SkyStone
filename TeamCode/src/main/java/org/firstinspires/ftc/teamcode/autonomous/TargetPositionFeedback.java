package org.firstinspires.ftc.teamcode.autonomous;


/**
 * Class encapsulates Vuforia continous Vuforia location checking and issues a callback when
 * location reached
 */
public class TargetPositionFeedback {

    private VuforiaTargetLocator mVuforia;
    private double mXTarget;
    private double mYTarget;
    private double mTolerance;
    private ILocationCallback mCallback;

    public TargetPositionFeedback(VuforiaTargetLocator vuforia, double xtarget, double ytarget, double tolerance, ILocationCallback callback){
        mVuforia = vuforia;
        mXTarget = xtarget;
        mYTarget = ytarget;
        mTolerance = tolerance;
        mCallback = callback;
    }

    public void checkTargetPosition(){
/**       VuforiaTensorFlowObjectDetector.TargetPosition location = mVuforia.getVuforiaNavLocation();
       if (location.valid){
           double x = Math.abs(location.x - mXTarget);
           if (x < mTolerance){
               double y = Math.abs(location.y- mYTarget);
               if (y < mTolerance){
                   if (mCallback != null){
                       mCallback.locationReached();
                   }
               }

           }
       }
 **/
    }

    public interface ILocationCallback {
        void locationReached();
    }
}
