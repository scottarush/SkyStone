package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="BlueAllianceAutonomousMode", group="Robot")
//@Disabled
public class BlueAllianceAutonomousOpMode extends OpMode{

    private SkystoneAutonomousOpMode mRealOpmode = null;
    public BlueAllianceAutonomousOpMode(){
        mRealOpmode = new SkystoneAutonomousOpMode(this, true);
    }

    @Override
    public void init() {
        mRealOpmode.init();
    }

    @Override
    public void loop() {
        mRealOpmode.loop();
    }


}


