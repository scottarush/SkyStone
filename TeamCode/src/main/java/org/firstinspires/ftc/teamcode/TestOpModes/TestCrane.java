package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.speedbot.Crane;

/**
 * OpMode to test the crane and hand using the controller.
 *
 */
@TeleOp(name="TestCrane", group="Robot")
//@Disabled
public class TestCrane extends OpMode {

    private Crane mCrane = null;

    private static final int MIN_DELTA_UPDATE_TIME_MS = 100;

    private long mLastUpdateTime = 0L;

    @Override
    public void init() {
        mCrane = new Crane(this);
        try{
            mCrane.init(hardwareMap);
        }
        catch(Exception e){
            telemetry.addData("Init error:",e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void loop() {
        long delta = System.currentTimeMillis() - mLastUpdateTime;
        if (delta > MIN_DELTA_UPDATE_TIME_MS) {
            mLastUpdateTime = System.currentTimeMillis();
            if (gamepad1.y) {
                mCrane.openHand();
            }
            else if (gamepad1.a) {
                mCrane.closeHand();
            }
            double lower = gamepad1.left_trigger;
            double right = gamepad1.right_trigger;
            if (lower > 0.05d) {
                mCrane.lowerManual(lower);
            }
            else if (right > 0.05d){
                mCrane.raiseManual(right);
            }
            else{
                mCrane.stop();
            }

            if (gamepad1.b){
                mCrane.moveByEncoder(1.0d,12d);
            }
            else if (gamepad1.x){
                mCrane.moveByEncoder(1.0d,-12d);
            }

            mCrane.loop();
        }
    }

}
