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
        double xleft = gamepad1.left_stick_x;
        double yleft = -gamepad1.left_stick_y;
        double xright = gamepad1.right_stick_x;
        double yright = -gamepad1.right_stick_y;
        xleft = applyJoystickGain(xleft);
        yleft = applyJoystickGain(yleft);
        xright = applyJoystickGain(xright);
        yright = applyJoystickGain(yright);
        if (gamepad1.a) {
            mSpeedBot.getDrivetrain().setSteeringCommand(-0.5d, 0.5d);
            mSpeedBot.getDrivetrain().loop();
        }
        else if (gamepad1.b) {
            mSpeedBot.getDrivetrain().setSteeringCommand(0.5d, 0.5d);
            mSpeedBot.getDrivetrain().loop();
        }
        else if (gamepad1.x) {
            mSpeedBot.getDrivetrain().setSteeringCommand(-0.7d, 0.8d);
            mSpeedBot.getDrivetrain().loop();
        }
        else if (gamepad1.y) {
            mSpeedBot.getDrivetrain().setSteeringCommand(0.7d, 0.8d);
        }
        else{
            mSpeedBot.getDrivetrain().setSteeringCommand(0, 0);
        }
        mSpeedBot   .getDrivetrain().setTankDriveJoystickInput(xleft,yleft,xright,yright);

    }
    private double applyJoystickGain(double input){
        double output = input * input;
        return output * Math.signum(input);
    }
    public void stop() {mSpeedBot.getDrivetrain().stop();}
}
