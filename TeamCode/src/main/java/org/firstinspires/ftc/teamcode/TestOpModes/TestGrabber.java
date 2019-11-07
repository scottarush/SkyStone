package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Grabber;

/**
 * OpMode to test the Grabber using the gamepad.  The left and right triggers control the
 * left and right grabber motors respectively.
 */
@TeleOp(name="TestGrabber", group="Robot")
@Disabled
public class TestGrabber extends OpMode {

    private Grabber grabber;
    @Override
    public void init()  {
        grabber = new Grabber(this);
        try{
            grabber.init(hardwareMap);
        }
        catch(Exception e){
            telemetry.addData("Error initializing grabber:"+e.getMessage(),0);
            telemetry.update();
        }
    }

    @Override
    public void loop() {
        grabber.moveGrabber(gamepad1.left_bumper,gamepad1.right_bumper, gamepad1.left_trigger,gamepad1.right_trigger);
    }

    @Override
    public void stop() {
        grabber.stop();
    }
}
