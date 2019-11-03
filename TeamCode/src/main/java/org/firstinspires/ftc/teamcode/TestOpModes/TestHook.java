package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Hook;
import org.firstinspires.ftc.teamcode.arm.FourBarArm;

/**
 * OpMode to test the hook using the gamepad.
 */
@TeleOp(name="TestHook", group="Robot")
//@Disabled
public class TestHook extends OpMode {

    private Hook mHook;
    private DigitalChannel mLimitSwitch = null;

    @Override
    public void init() {
        mHook = new Hook(this);
        try{
            mHook.init(hardwareMap);
        }
        catch(Exception e){
            telemetry.addData("Init error:",e.getMessage());
            telemetry.update();
        }
        try {
            mLimitSwitch = hardwareMap.get(DigitalChannel.class, FourBarArm.LIMIT_SENSOR_NAME);
            mLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        } catch (Exception e) {
            telemetry.addData("Limit switch init error:",e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            mHook.setPosition(Hook.OPEN);
        }
        else if (gamepad1.a){
            mHook.setPosition(Hook.CLOSED);
        }
        else if (gamepad1.b){
            mHook.setPosition(Hook.RETRACTED);
        }
    }

}
