package org.firstinspires.ftc.teamcode.filter;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drivetrain.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.IMU;
import org.firstinspires.ftc.teamcode.speedbot.BaseSpeedBot;

@Autonomous(name="FilterDevelopment", group="Robot")
public class FilterDevelopmentOpMode extends OpMode{

    public static final double INIT_PX = 0D;
    public static final double INIT_PY = 0D;
    public static final double INIT_HEADING = Math.PI/2;
    public static final double T = 0.050d;
    private static final int T_MS = Math.round((float)(T * 1000d));

    private BaseSpeedBot mSpeedBot = null;

    private IKalmanTracker mKalmanTracker = null;

    private long mLastSystemTime = 0;
    private boolean mFirstTime = true;

    @Override
    public void init() {
        msStuckDetectInit = 1000000;
        String initErrs = "";
        try {
            mSpeedBot = new BaseSpeedBot(this, true);
            mSpeedBot.init();
        }
        catch(Exception e){
            initErrs += ","+e.getMessage();
        }

        if (initErrs.length() == 0){
            telemetry.addData("Status:","Robot init complete");
            telemetry.update();
        }
        else {
            telemetry.addData("Init errors:", initErrs);
            telemetry.update();
        }
        // Initialize the KalmanTracker
        mKalmanTracker = new KalmanTracker();
        mKalmanTracker.init(T,INIT_PX,INIT_PY,INIT_HEADING,BaseSpeedBot.LX_MM,BaseSpeedBot.LY_MM,BaseSpeedBot.WHEEL_RADIUS_MM);
    }

    public void loop() {
        if (mFirstTime){
            mFirstTime = false;
            mLastSystemTime = System.currentTimeMillis();
            updateTracker();
            return;
        }
        // Compute the delta time and update the Tracker if we are at the sampel period T
        long systemTime = System.currentTimeMillis();
        int deltat_ms = (int)(systemTime-mLastSystemTime);
        if (deltat_ms > T_MS){
            updateTracker();
            mLastSystemTime = systemTime;   // save for next loop
        }

        // update speeds from the joystick
        double xleft = gamepad1.left_stick_x;
        double yleft = -gamepad1.left_stick_y;
        double xright = gamepad1.right_stick_x;
        double yright = -gamepad1.right_stick_y;
        // apply nonlinear joystick gain to each raw value
        xleft = applyJoystickGain(xleft);
        yleft = applyJoystickGain(yleft);
        xright = applyJoystickGain(xright);
        yright = applyJoystickGain(yright);
        mSpeedBot.getDrivetrain().setTankDriveJoystickInput(xleft,yleft,xright,yright);

        // Service the drivetrain loop to update wheel speed measurements
        mSpeedBot.getDrivetrain().loop();
        // Now output the wheel speeds
        double[] speeds = mSpeedBot.getDrivetrain().getWheelSpeeds();
        telemetry.addData("WhlSpds:","%4.2f,%4.2f,%4.2f,%4.2f",speeds[0],speeds[1],speeds[2],speeds[3]);
        telemetry.update();

        // TODO: Send the estimated position and heading to the state machine controller
        // TODO: update the robot speed
    }

    /**
     * helper function to update the Tracker
     */
    private void updateTracker(){
        // Get the wheel speeds
        double[] wheelSpeeds = mSpeedBot.getDrivetrain().getWheelSpeeds();

        // Now get the IMU data
        IMU imu = mSpeedBot.getIMU();
        Acceleration acceleration = imu.getBNO055IMU().getLinearAcceleration();
        Orientation orientation = imu.getBNO055IMU().getAngularOrientation();
        // Update the tracker
        mKalmanTracker.updateMeasurement(wheelSpeeds[BaseMecanumDrive.LF_WHEEL_ARRAY_INDEX],
                    wheelSpeeds[BaseMecanumDrive.LR_WHEEL_ARRAY_INDEX],
                wheelSpeeds[BaseMecanumDrive.RF_WHEEL_ARRAY_INDEX],
                wheelSpeeds[BaseMecanumDrive.RR_WHEEL_ARRAY_INDEX],
                acceleration.xAccel,
                acceleration.yAccel,
                orientation.firstAngle);
    }

    /**
     * helper function for control gain
     */
    private double applyJoystickGain(double input){
        double output = input * input;
        return output * Math.signum(input);
    }
/**
    public static void main(String[] args) {
        SkystoneAutonomousOpMode mode = new SkystoneAutonomousOpMode();
        mOpmode.mode.gamepad1 = new Gamepad();
        mOpmode.mode.telemetry = new TelemetryImpl(mode);
        mode.init_loop();
    }
**/

}
