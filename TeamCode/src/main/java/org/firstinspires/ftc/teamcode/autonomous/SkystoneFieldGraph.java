package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.Point;

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

    public Route mWhiteRouteBlue = new Route("BLUE_WHITE", this);

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
    public boolean setSkystoneFieldPosition(double x, double y,Route startingRoute){
        // Translate coords
        double xtrans = x+FIELD_WIDTH/2d;
        double ytrans = y+FIELD_HEIGHT/2d;
        resetRobotPosition(xtrans,ytrans,startingRoute);
        return true;
    }

    public FieldGraph.GraphPosition convertSkystonePosition(double x, double y){
        GraphPosition position = getGraphPosition(x,y);
        return position;
    }

    public static PointD translateSkystonePosition(double x, double y){
        double xtrans = x+FIELD_WIDTH/2d;
        double ytrans = y+FIELD_HEIGHT/2d;
        return new PointD(xtrans,ytrans);
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
    }


    public static void main(String[] args){
        SkystoneFieldGraph graph = new SkystoneFieldGraph();
        double robotx = -55;
        double roboty = -36;
        boolean retcode = graph.setSkystoneFieldPosition(-55,-36,graph.mWhiteRouteBlue);
        GraphPosition position = graph.getRobotPosition();


        while(true){
            MovementManeuver maneuver = (MovementManeuver)graph.getNextManeuver();
            if (maneuver == null) {
                break;
            }
            // Prinout the maneuver
            System.out.println(maneuver.toString());
            robotx += maneuver.xDelta;
            roboty += maneuver.yDelta;
            PointD point = translateSkystonePosition(robotx,roboty);
            graph.updateRobotPosition(point.x,point.y,graph.mWhiteRouteBlue);
            if (graph.mWhiteRouteBlue.isLastTransition()){
                break;
            }
            graph.mWhiteRouteBlue.incrementTransitionIndex();
        }

    }
}
