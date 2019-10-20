package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.configuration.ExpansionHubMotorControllerParamsState;

import org.firstinspires.ftc.teamcode.arm.FourBarArm;

/**
 * OpMode to test the arm using the gamepad.  The left stick up and down raises and lowers the arm.
 */
@TeleOp(name="TestArm", group="Robot")
public class TestArm extends OpMode {

    private FourBarArm arm;
    @Override
    public void init() {
        arm = new FourBarArm(this);
        try{
            arm.init(hardwareMap);
        }
        catch(Exception e){
            telemetry.addData("Error initializing arm:"+e.getMessage(),0);
            telemetry.update();
        }
    }

    @Override
    public void loop() {
        double xleft = gamepad1.left_stick_x;

        double position = xleft * 90;

        arm.setPosition(position);

        telemetry.addData("Commanded Position",position);
        telemetry.addData("current position",arm.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        arm.stop();
    }
}
