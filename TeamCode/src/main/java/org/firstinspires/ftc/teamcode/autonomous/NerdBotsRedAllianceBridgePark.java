package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="NerdBotsRedAllianceBridgePark", group="Robot")
@Disabled
public class NerdBotsRedAllianceBridgePark extends OpMode{

    private SkystoneAutonomousOpMode mRealOpmode = null;
    public NerdBotsRedAllianceBridgePark(){
        mRealOpmode = new SkystoneAutonomousOpMode(this, false,AutonomousController.SEQUENCE_NERDBOTS_BRIDGE_PARK);
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


