package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.arm.FourBarArm;

/**
 * OpMode to test the arm using the gamepad.  The left stick up and down raises and lowers the arm.
 */
@TeleOp(name="TestArm", group="Robot")
//@Disabled
public class TestArm extends OpMode {

    private FourBarArm arm;
    @Override
    public void init() {
        arm = new FourBarArm(this, true);
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
        double power = gamepad1.left_stick_y;
/*
        double position = power * 90;

        arm.setPower(1.0);
        arm.gotoAngle(position);
       telemetry.addData("Commanded degree position",position);
        telemetry.addData("current degree position",arm.getCurrentAngle());
        telemetry.addData("counts per degree",FourBarArm.COUNTS_PER_DEGREE);
*/
        if (!arm.isResetToRetractInProgress()) {
            if (Math.abs(power) > 0.1) {
                boolean up = true;
                if (Math.signum(power) < 0) {
                    up = false;
                }
                arm.moveArm(up);
//                telemetry.addData("Commanded power", power);
//                telemetry.update();
            } else {
                arm.stop();
            }
        }
        boolean status = false;
        if (gamepad1.a) {
             status = arm.resetToRetractPosition();
        }
        else if (arm.isResetToRetractInProgress()) {
            status = arm.resetToRetractPosition();
        }
        telemetry.addData("Arm status",status);
        telemetry.update();
    }

    @Override
    public void stop() {
        arm.stop();
    }
}
