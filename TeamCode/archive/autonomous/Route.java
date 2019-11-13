package org.firstinspires.ftc.teamcode.archive.autonomous;

import java.util.ArrayList;
import java.util.Iterator;

/**
 * This the class for a route.
 */
public class Route {
    private String mName;
    private FieldGraph mGraph;

    FieldTile mCurrentBuildTile = null;

    private ArrayList<RouteTransition> mRouteTransitions = new ArrayList<>();

    private int mNextTransitionIndex = 0;

    public Route(String name,FieldGraph graph){
        mName = name;
        mGraph = graph;
    }

    public String getName(){
        return mName;
    }

    /**
     * Sets the starting tile of the route.
     * @return false if tileNum invalid, false otherwise
     */
    public boolean setStartTile(int tileNum){
        FieldTile tile = mGraph.getTile(tileNum);
        if (tile == null)
            return false;
        tile.setStartOfRoute(this);
        mCurrentBuildTile = tile;
        return true;
    }

    public boolean addRouteTransition(int destTileNum,Action entryAction){
        FieldTile tile = mGraph.getTile(destTileNum);
        if (tile == null)
            return false;
        RouteTransition transition = mCurrentBuildTile.addRouteTransition(this,tile,entryAction);
        if (transition != null){
            mCurrentBuildTile = tile;
            // Set the transition number to the next avalaible index
            transition.transitionIndex = mRouteTransitions.size();
            mRouteTransitions.add(transition);
        }
        return (transition != null);
    }

    /**
     * @return number of transitions on the route.
     */
    public int getNumTransitions(){
        return mRouteTransitions.size();
    }

    /**
     * @return the RouteTransition at the current transition index or null if no transitions have been added yet.
     */
    public RouteTransition getNextRouteTransition(){
        return mRouteTransitions.get(mNextTransitionIndex);
    }
    /**
     * @return true we have reached the end of the route
     */
    public boolean isLastTransition(){
        if (mNextTransitionIndex == mRouteTransitions.size()-1){
            return true;
        }
        return false;
    }
    /**
     * Increments the transition index.
     * @return the current transition index.
     */
    public int incrementTransitionIndex(){
        if (mNextTransitionIndex < mRouteTransitions.size()-1){
            mNextTransitionIndex++;
        }
        return mNextTransitionIndex;
    }
    /**
     * Retrieves the transition index for the provided tile.  If there are multiple transitions
     * on this tile, the start or first incoming transition will be selected.
     * @return index or -1 if tile is not on this route
     */
    private int getTransitionIndex(FieldTile tile){
        int index = 0;
        for(Iterator<RouteTransition> iter = mRouteTransitions.iterator(); iter.hasNext();){
            RouteTransition transition = iter.next();
            if (transition.startTile == tile){
                return index;
            }
            index++;
        }
        return -1;
    }

    /**
     * Resets the route to run on the provided tile
     * @return true if start, false if tile was invalid.
     */
    public boolean resetRouteOnTile(FieldTile tile){
        int index =  getTransitionIndex(tile);
        if (index < 0)
            return false;
        mNextTransitionIndex = index;
        return true;
    }



}
