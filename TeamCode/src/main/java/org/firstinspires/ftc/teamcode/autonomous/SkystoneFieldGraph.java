package org.firstinspires.ftc.teamcode.autonomous;

/**
 * SkystoneFile specific graph.  Layout is as follows:
 *
 */

public class SkystoneFieldGraph extends FieldGraph {

    /** size of each block in inches. **/
    public static final double TILE_WIDTH = 24.0d;
    public static final double TILE_HEIGHT = 24.0d;

    public static final int NUM_COLS = 6;
    public static final int NUM_ROWS = 6;

    public static final double FIELD_WIDTH = TILE_WIDTH * NUM_COLS;
    public static final double FIELD_HEIGHT = TILE_HEIGHT * NUM_ROWS;

    private GraphPosition[][] mTileLocations;

    private Route mWhiteRouteBlue = new Route("BLUE_WHITE", this);

    public SkystoneFieldGraph(){
        super(NUM_ROWS, NUM_COLS,TILE_WIDTH,TILE_HEIGHT);

        // Add all the routes
        addBlueWhiteRoute(mWhiteRouteBlue);
    }

    /**
     * Sets the robot location on the field in absolute skystone coordinates which will
     * be translated into absolute coords.
     * @return true if location was valid, false otherwise
     */
    public boolean setSkystoneFieldPosition(double x, double y){
        // Translate coords
        double xtrans = x+FIELD_WIDTH/2d;
        double ytrans = y+FIELD_HEIGHT/2d;
        setRobotPosition(xtrans,ytrans,null);
        return true;
    }



    /**
     *  Blue White route.
     */
    private void addBlueWhiteRoute(Route r){
        r.setStartTile(7);
        r.addRouteTransition(1,null);
        r.addRouteTransition(2,null);
        r.addRouteTransition(1,null);
        r.addRouteTransition(7,null);
        r.addRouteTransition(13,null);
        r.addRouteTransition(19,null);
        r.addRouteTransition(25,null);
        r.addRouteTransition(31,null);
        r.addRouteTransition(32,null);
        r.addRouteTransition(31,null);
        r.addRouteTransition(25,null);
        r.addRouteTransition(19,null);
        r.setEndTile(19);
    }


    public static void main(String[] args){
        SkystoneFieldGraph graph = new SkystoneFieldGraph();
        boolean retcode = graph.setSkystoneFieldPosition(-55,-36);
        GraphPosition position = graph.getRobotPosition();

        Maneuver maneuver = graph.getNextManeuver();

    }
}
