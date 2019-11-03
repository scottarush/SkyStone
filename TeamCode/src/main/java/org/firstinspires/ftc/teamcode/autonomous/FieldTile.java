package org.firstinspires.ftc.teamcode.autonomous;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.HashMap;

/**
 * This is a data structure class used by the FieldGraph to represent the
 * each block.
 */
public class FieldTile {

    public static final int EDGE_INVALID = -1;

    public static final int EDGE_TOP = 0;
    public static final int EDGE_RIGHT = 1;
    public static final int EDGE_BOTTOM = 2;
    public static final int EDGE_LEFT = 3;


    private FieldTile mNeighbors[] = new FieldTile[4];

    /**
     * List of routes that start on this tile. Routes in this
     * list will also be in the outgoing routes
     */
    private ArrayList<Route>mStartingRoutes = new ArrayList<>();
    /**
     * List of routes that end on this tile.  Routes in this
     * list will also be in the incoming routes.
     */
    private ArrayList<Route>mEndingRoutes = new ArrayList<>();

    /**
     * Array of Lists of incoming routes on each edges
     */
    private ArrayList<Route> mIncomingRoutesArray[] = new ArrayList[4];
    /**
     * Array of Lists of outgoing routes on each edges.
     */
    private ArrayList<Route> mOutgoingRoutesArray[] =  new ArrayList[4];

    private HashMap<Route,Action> mRouteEntryActionMap = new HashMap<>();

    private TileLocation mTileLocation = null;

    public FieldTile(int tileNum,double width, double height, double xbase, double ybase){
        mTileLocation = new TileLocation(tileNum,xbase,ybase);
        for(int i=0;i <= 3;i++){
            mIncomingRoutesArray[i] = new ArrayList<>();
            mOutgoingRoutesArray[i] = new ArrayList<>();
        }
     }

    public static class TileLocation {
        TileLocation(int tileNum, double x, double y){
            this.x = x;
            this.y = y;
            this.mTileNum = tileNum;

        }
        /** left edge of tile  **/
        public double x = 0;
        /** lower edge of tile  **/
        public double y = 0;
        public int mTileNum = 0;
    }

    /**
     *
     * @return the number of this tile.
     *
     */
    public int getTileNum(){
       return mTileLocation.mTileNum;
    }

    public TileLocation getTileLocation(){
        return mTileLocation;
    }
    /**
     * Sets a neighboor
     */
    public boolean setNeighbor(int edge, FieldTile block){
        if (!isEdgeValid(edge)){
            return false;
        }
        mNeighbors[edge] = block;
        return true;
    }


    public static boolean isEdgeValid(int edge){
        if ((edge < 0) || (edge > 3)) {
            return false;
        }
        return true;
    }

    /**
     * @return the neighboring tile on the selected edge or null if no
     * neighbor on that edge
     * @param edge
     */

    public FieldTile getNeighbor(int edge){
        if (!isEdgeValid(edge))
            return null;
        return mNeighbors[edge];
    }

    /**
     * Called to set this tile as the start of a route.
     * @param r the route to set as start
     */
    public void setStartOfRoute(Route r){
        mStartingRoutes.add(r);
    }
    /**
     * Called to set this tile as the end of a route.
     * @param r
     * @return true on success, false if this route is not already in incoming connections
     */
    public boolean setEndOfRoute(Route r){
        boolean found = false;
        for(int i=0;i <= 3;i++) {
            if (mIncomingRoutesArray[i].contains(r)){
                found = true;
                break;
            }
        }
        if (!found)
            return false;
        mEndingRoutes.add(r);
        return true;
    }

    /**
     * Called to add this route as an outgoing connection on this tile.
     * @param r
     * @param destTile
     * @param action Action to be taken on entry to destTile or null if no action
     * @return RouteTransition or null if destTile is not a neigher
     */
    public RouteTransition addRouteTransition(Route r, FieldTile destTile,Action action){
        // Find the edge where the route goes.
        for(int edge=0;edge <= 3;edge++){
            FieldTile tile = mNeighbors[edge];
            if (tile == destTile){
                // This is it.  add it to the correct edge array
                mOutgoingRoutesArray[edge].add(r);
                // And add this tile to the mIncomingRoutesArray the destTiles correct edge
                int destEdge = 0;
                switch(edge){
                    case EDGE_LEFT:
                        destEdge = EDGE_RIGHT;
                        break;
                    case EDGE_RIGHT:
                        destEdge = EDGE_LEFT;
                        break;
                    case EDGE_BOTTOM:
                        destEdge = EDGE_TOP;
                        break;
                    case EDGE_TOP:
                        destEdge = EDGE_BOTTOM;
                        break;
                }
                destTile.mIncomingRoutesArray[destEdge].add(r);
                if (action != null){
                    destTile.mRouteEntryActionMap.put(r,action);
                }
                return new RouteTransition(this,destTile,0);
            }
        }
        // Must not have been a neighbor
        return null;
    }

    /**
     * Returns an entry action for the provided route or null if no action has been set on this
     * route for this tile.
     */
    public Action getEntryAction(Route r){
        Action action = mRouteEntryActionMap.get(r);
        return action;
    }

    /**
     * @return the outgoing edge of the supplied route or EDGE_INVALID if the route is not
     * an outgoing route on this tile.
     */
    public int getOutgoingEdge(Route r){
        for(int edge=0;edge <= 3;edge++){
                return edge;
        }
        return EDGE_INVALID;
    }

    /**
     * @return list of starting routes on this tile.
     *
     */
    public ArrayList<Route> getStartingRoutes(){
        return mStartingRoutes;
    }

    /** @return list of incoming routes to this tile as an array indexed EDGE. **/
    public ArrayList<Route>[] getIncomingRoutes(){
        return mIncomingRoutesArray;
    }
    /** @return list of outgoing routes to this tile as an array indexed EDGE. **/
    public ArrayList<Route>[] getOutgoingRoutes(){
        return mOutgoingRoutesArray;
    }
}
