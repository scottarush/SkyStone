package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonomous.SkystoneAutonomousOpMode;

@Autonomous(name="RedAllianceDragFoundation", group="Robot")
//@Disabled
public class AutoDragFoundationRedAlliance extends OpMode {
    private SkystoneAutonomousOpMode mRealOpmode = null;
    public AutoDragFoundationRedAlliance(){
        mRealOpmode = new SkystoneAutonomousOpMode(this, false,AutonomousController.SEQUENCE_DRAG_FOUNDATION);
    }

    @Override
    public void init() {
        mRealOpmode.init();
    }

    @Override
    public void loop() {
        mRealOpmode.loop();
    }}


