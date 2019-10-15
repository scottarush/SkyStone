package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TestRobot", group="Robot")
public class TestRobot extends OpMode {

    private Robot robot = null;

    private double lfPower = 0.0;
    private double rfPower = 0.0;
    private double lrPower = 0.0;
    private double rrPower = 0.0;

    @Override
    public void init() {
        robot = new Robot(Robot.DriveTrainStyle.MECANUM, this);
        robot.init();
    }

    @Override
    public void loop() {
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        double xleft = gamepad1.left_stick_x;
        double yleft = -gamepad1.left_stick_y;
        double xright = gamepad1.right_stick_x;
        double yright = -gamepad1.right_stick_y;

        // the speeds with the new gamepad inputs
        computeMotorPower(xleft,yleft,xright,yright);

        // Set the motor power to the speeds
        robot.drivetrain.setPower(lfPower, rfPower, lrPower, rrPower);

        telemetry.update();
    }

    @Override
    public void stop() {
        robot.drivetrain.stop();
    }

    private void computeMotorPower(double xleft, double yleft, double xright, double yright) {

        lfPower = yleft+(xleft+xright)/2;
        rfPower = yright - (xleft+xright)/2;
        lrPower = yleft-(xleft + xright)/2;
        rrPower = yright + (xleft+xright)/2;

        /**
         * Now normalize the wheel speed commands:
         * Let speedmax be the maximum absolute value of the four wheel speed commands.
         * If speedmax is greater than 1, then divide each of the four wheel speed commands by speedmax.
         **/
        double speedmax = Math.abs(lfPower + rfPower + lrPower + rrPower);
        if (speedmax > 4.0){
            lfPower = lfPower /speedmax;
            rfPower = rfPower / speedmax;
            lrPower = lrPower / speedmax;
            rrPower = rrPower / speedmax;
        }
    }

}
