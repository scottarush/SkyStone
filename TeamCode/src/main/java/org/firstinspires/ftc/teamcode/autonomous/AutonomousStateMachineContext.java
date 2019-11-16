/*
 * ex: set ro:
 * DO NOT EDIT.
 * generated by smc (http://smc.sourceforge.net/)
 * from file : AutonomousStateMachine.sm
 */

package org.firstinspires.ftc.teamcode.autonomous;


public class AutonomousStateMachineContext
    extends statemap.FSMContext
{
//---------------------------------------------------------------
// Member methods.
//

    public AutonomousStateMachineContext(AutonomousController owner)
    {
        this (owner, AutonomousStateMachine.Idle);
    }

    public AutonomousStateMachineContext(AutonomousController owner, AutonomousControllerState initState)
    {
        super (initState);

        _owner = owner;
    }

    @Override
    public void enterStartState()
    {
        getState().entry(this);
        return;
    }

    public void evDriveComplete()
    {
        _transition = "evDriveComplete";
        getState().evDriveComplete(this);
        _transition = "";
        return;
    }

    public void evHookTimeout()
    {
        _transition = "evHookTimeout";
        getState().evHookTimeout(this);
        _transition = "";
        return;
    }

    public void evNoStoneFound()
    {
        _transition = "evNoStoneFound";
        getState().evNoStoneFound(this);
        _transition = "";
        return;
    }

    public void evRotationComplete()
    {
        _transition = "evRotationComplete";
        getState().evRotationComplete(this);
        _transition = "";
        return;
    }

    public void evStart()
    {
        _transition = "evStart";
        getState().evStart(this);
        _transition = "";
        return;
    }

    public AutonomousControllerState getState()
        throws statemap.StateUndefinedException
    {
        if (_state == null)
        {
            throw(
                new statemap.StateUndefinedException());
        }

        return ((AutonomousControllerState) _state);
    }

    protected AutonomousController getOwner()
    {
        return (_owner);
    }

    public void setOwner(AutonomousController owner)
    {
        if (owner == null)
        {
            throw (
                new NullPointerException(
                    "null owner"));
        }
        else
        {
            _owner = owner;
        }

        return;
    }

//---------------------------------------------------------------
// Member data.
//

    transient private AutonomousController _owner;

    //-----------------------------------------------------------
    // Constants.
    //

    private static final long serialVersionUID = 1L;

//---------------------------------------------------------------
// Inner classes.
//

    public static abstract class AutonomousControllerState
        extends statemap.State
    {
    //-----------------------------------------------------------
    // Member methods.
    //

        protected AutonomousControllerState(String name, int id)
        {
            super (name, id);
        }

        protected void entry(AutonomousStateMachineContext context) {}
        protected void exit(AutonomousStateMachineContext context) {}

        protected void evDriveComplete(AutonomousStateMachineContext context)
        {
            Default(context);
        }

        protected void evHookTimeout(AutonomousStateMachineContext context)
        {
            Default(context);
        }

        protected void evNoStoneFound(AutonomousStateMachineContext context)
        {
            Default(context);
        }

        protected void evRotationComplete(AutonomousStateMachineContext context)
        {
            Default(context);
        }

        protected void evStart(AutonomousStateMachineContext context)
        {
            Default(context);
        }

        protected void Default(AutonomousStateMachineContext context)
        {
            throw (
                new statemap.TransitionUndefinedException(
                    "State: " +
                    context.getState().getName() +
                    ", Transition: " +
                    context.getTransition()));
        }

    //-----------------------------------------------------------
    // Member data.
    //

        //-------------------------------------------------------
    // Constants.
    //

        private static final long serialVersionUID = 1L;
    }

    /* package */ static abstract class AutonomousStateMachine
    {
    //-----------------------------------------------------------
    // Member methods.
    //

    //-----------------------------------------------------------
    // Member data.
    //

        //-------------------------------------------------------
        // Constants.
        //

        public static final AutonomousStateMachine_Idle Idle =
            new AutonomousStateMachine_Idle("AutonomousStateMachine.Idle", 0);
        public static final AutonomousStateMachine_DriveToStones DriveToStones =
            new AutonomousStateMachine_DriveToStones("AutonomousStateMachine.DriveToStones", 1);
        public static final AutonomousStateMachine_LocateSkystone LocateSkystone =
            new AutonomousStateMachine_LocateSkystone("AutonomousStateMachine.LocateSkystone", 2);
        public static final AutonomousStateMachine_StrafeToStone StrafeToStone =
            new AutonomousStateMachine_StrafeToStone("AutonomousStateMachine.StrafeToStone", 3);
        public static final AutonomousStateMachine_DriveForwardToIntake DriveForwardToIntake =
            new AutonomousStateMachine_DriveForwardToIntake("AutonomousStateMachine.DriveForwardToIntake", 4);
        public static final AutonomousStateMachine_IntakeStone IntakeStone =
            new AutonomousStateMachine_IntakeStone("AutonomousStateMachine.IntakeStone", 5);
        public static final AutonomousStateMachine_BackupToBeginDrag BackupToBeginDrag =
            new AutonomousStateMachine_BackupToBeginDrag("AutonomousStateMachine.BackupToBeginDrag", 6);
        public static final AutonomousStateMachine_BlueAllianceRotateTowardBridge BlueAllianceRotateTowardBridge =
            new AutonomousStateMachine_BlueAllianceRotateTowardBridge("AutonomousStateMachine.BlueAllianceRotateTowardBridge", 7);
        public static final AutonomousStateMachine_RedAllianceRotateTowardBridge RedAllianceRotateTowardBridge =
            new AutonomousStateMachine_RedAllianceRotateTowardBridge("AutonomousStateMachine.RedAllianceRotateTowardBridge", 8);
        public static final AutonomousStateMachine_DragStone DragStone =
            new AutonomousStateMachine_DragStone("AutonomousStateMachine.DragStone", 9);
        public static final AutonomousStateMachine_ReleaseStone ReleaseStone =
            new AutonomousStateMachine_ReleaseStone("AutonomousStateMachine.ReleaseStone", 10);
        public static final AutonomousStateMachine_Complete Complete =
            new AutonomousStateMachine_Complete("AutonomousStateMachine.Complete", 11);
    }

    protected static class AutonomousStateMachine_Default
        extends AutonomousControllerState
    {
    //-----------------------------------------------------------
    // Member methods.
    //

        protected AutonomousStateMachine_Default(String name, int id)
        {
            super (name, id);
        }

    //-----------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class AutonomousStateMachine_Idle
        extends AutonomousStateMachine_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private AutonomousStateMachine_Idle(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void evStart(AutonomousStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(AutonomousStateMachine.DriveToStones);
            (context.getState()).entry(context);
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class AutonomousStateMachine_DriveToStones
        extends AutonomousStateMachine_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private AutonomousStateMachine_DriveToStones(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(AutonomousStateMachineContext context)
            {
                AutonomousController ctxt = context.getOwner();

            ctxt.setLogMessage("DriveToStones");
            ctxt.openHook();
            ctxt.linearDrive(30d);
            return;
        }

        @Override
        protected void evDriveComplete(AutonomousStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(AutonomousStateMachine.LocateSkystone);
            (context.getState()).entry(context);
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class AutonomousStateMachine_LocateSkystone
        extends AutonomousStateMachine_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private AutonomousStateMachine_LocateSkystone(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(AutonomousStateMachineContext context)
            {
                AutonomousController ctxt = context.getOwner();

            ctxt.setLogMessage("LocateSkystone");
            ctxt.checkForStones();
            return;
        }

        @Override
        protected void evNoStoneFound(AutonomousStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(AutonomousStateMachine.DriveForwardToIntake);
            (context.getState()).entry(context);
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class AutonomousStateMachine_StrafeToStone
        extends AutonomousStateMachine_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private AutonomousStateMachine_StrafeToStone(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(AutonomousStateMachineContext context)
            {
                AutonomousController ctxt = context.getOwner();

            ctxt.setLogMessage("SrafeToStone");
            return;
        }

        @Override
        protected void evDriveComplete(AutonomousStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(AutonomousStateMachine.DriveForwardToIntake);
            (context.getState()).entry(context);
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class AutonomousStateMachine_DriveForwardToIntake
        extends AutonomousStateMachine_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private AutonomousStateMachine_DriveForwardToIntake(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(AutonomousStateMachineContext context)
            {
                AutonomousController ctxt = context.getOwner();

            ctxt.setLogMessage("DriveForwardToIntake");
            ctxt.linearDrive(6d);
            return;
        }

        @Override
        protected void evDriveComplete(AutonomousStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(AutonomousStateMachine.IntakeStone);
            (context.getState()).entry(context);
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class AutonomousStateMachine_IntakeStone
        extends AutonomousStateMachine_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private AutonomousStateMachine_IntakeStone(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(AutonomousStateMachineContext context)
            {
                AutonomousController ctxt = context.getOwner();

            ctxt.setLogMessage("IntakeStone");
            ctxt.closeHook();
            ctxt.startHookTimer();
            ctxt.startGrabber(true, 1000);
            return;
        }

        @Override
        protected void evHookTimeout(AutonomousStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(AutonomousStateMachine.BackupToBeginDrag);
            (context.getState()).entry(context);
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class AutonomousStateMachine_BackupToBeginDrag
        extends AutonomousStateMachine_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private AutonomousStateMachine_BackupToBeginDrag(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(AutonomousStateMachineContext context)
            {
                AutonomousController ctxt = context.getOwner();

            ctxt.setLogMessage("BackupToBeginDrag");
            ctxt.linearDrive(-18d);
            return;
        }

        @Override
        protected void evDriveComplete(AutonomousStateMachineContext context)
        {
            AutonomousController ctxt = context.getOwner();

            if (ctxt.isBlueAlliance() == true)
            {
                (context.getState()).exit(context);
                // No actions.
                context.setState(AutonomousStateMachine.BlueAllianceRotateTowardBridge);
                (context.getState()).entry(context);
            }
            else if (ctxt.isBlueAlliance() == false)
            {
                (context.getState()).exit(context);
                // No actions.
                context.setState(AutonomousStateMachine.RedAllianceRotateTowardBridge);
                (context.getState()).entry(context);
            }            else
            {
                super.evDriveComplete(context);
            }

            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class AutonomousStateMachine_BlueAllianceRotateTowardBridge
        extends AutonomousStateMachine_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private AutonomousStateMachine_BlueAllianceRotateTowardBridge(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(AutonomousStateMachineContext context)
            {
                AutonomousController ctxt = context.getOwner();

            ctxt.setLogMessage("BlueAllianceRotateTowardBridge");
            ctxt.rotate(-90);
            return;
        }

        @Override
        protected void evRotationComplete(AutonomousStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(AutonomousStateMachine.DragStone);
            (context.getState()).entry(context);
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class AutonomousStateMachine_RedAllianceRotateTowardBridge
        extends AutonomousStateMachine_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private AutonomousStateMachine_RedAllianceRotateTowardBridge(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(AutonomousStateMachineContext context)
            {
                AutonomousController ctxt = context.getOwner();

            ctxt.setLogMessage("RedAllianceRotateTowardBridge");
            ctxt.rotate(+90);
            return;
        }

        @Override
        protected void evRotationComplete(AutonomousStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(AutonomousStateMachine.DragStone);
            (context.getState()).entry(context);
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class AutonomousStateMachine_DragStone
        extends AutonomousStateMachine_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private AutonomousStateMachine_DragStone(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(AutonomousStateMachineContext context)
            {
                AutonomousController ctxt = context.getOwner();

            ctxt.setLogMessage("DragStone");
            ctxt.linearDrive(72d);
            return;
        }

        @Override
        protected void evDriveComplete(AutonomousStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(AutonomousStateMachine.ReleaseStone);
            (context.getState()).entry(context);
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class AutonomousStateMachine_ReleaseStone
        extends AutonomousStateMachine_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private AutonomousStateMachine_ReleaseStone(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(AutonomousStateMachineContext context)
            {
                AutonomousController ctxt = context.getOwner();

            ctxt.setLogMessage("ReleaseStone");
            ctxt.openHook();
            ctxt.startGrabber(false, 3000);
            ctxt.linearDrive(-37d);
            return;
        }

        @Override
        protected void evDriveComplete(AutonomousStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(AutonomousStateMachine.Complete);
            (context.getState()).entry(context);
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class AutonomousStateMachine_Complete
        extends AutonomousStateMachine_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private AutonomousStateMachine_Complete(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(AutonomousStateMachineContext context)
            {
                AutonomousController ctxt = context.getOwner();

            ctxt.setLogMessage("Complete");
            ctxt.stopGrabber();
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }
}

/*
 * Local variables:
 *  buffer-read-only: t
 * End:
 */
