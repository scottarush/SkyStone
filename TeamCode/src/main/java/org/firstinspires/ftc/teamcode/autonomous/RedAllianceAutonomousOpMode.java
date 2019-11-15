package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonomous.SkystoneAutonomousOpMode;

@Autonomous(name="RedAllianceAutonomousMode", group="Robot")
//@Disabled
public class RedAllianceAutonomousOpMode extends OpMode {
    private SkystoneAutonomousOpMode mRealOpmode = null;
    public RedAllianceAutonomousOpMode(){
        mRealOpmode = new SkystoneAutonomousOpMode(this, false);
    }

    @Override
    public void init() {
        mRealOpmode.init();
    }

    @Override
    public void loop() {
        mRealOpmode.loop();
    }}


