package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.speedbot.BaseSpeedBot;

@TeleOp(name="IMUTest", group="Sensor")
//@Disabled
public class TestGuidanceController extends OpMode {
    private BaseSpeedBot mSpeedBot = null;

    @Override
    public void init() {
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
            telemetry.addData("IMU cal status",mSpeedBot.getGuidanceController().getIMUCalibrationStatus());
            telemetry.update();
        }
        else {
            telemetry.addData("Init errors:", initErrs);
            telemetry.update();
        }
    }

    @Override
    public void loop() {
 //       double y = gamepad1.left_stick_y;
        mSpeedBot.getDrivetrain().setSteeringCommand(-1.0d, 0.8d);
        mSpeedBot.getDrivetrain().loop();
    }
}
