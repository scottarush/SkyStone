package org.firstinspires.ftc.teamcode.autonomous;

import java.lang.reflect.Field;

/**
 * This is a data structure class used by the FieldGraph to represent the
 * each block.
 */
public class FieldBlock {

    public static final int TOP = 0;
    public static final int RIGHT = 1;
    public static final int BOTTOM = 2;
    public static final int LEFT = 3;

    public static final int NO_CONNECT = 0;
    public static final int IN_CONNECT = 1;
    public static final int OUT_CONNECT = 2;
    public static final int BI_CONNECT = 3;

    private FieldBlock mNeighbors[] = new FieldBlock[4];

    private int mBlockNum;

    public FieldBlock(int blockNum){
        mBlockNum = blockNum;
     }

    public int getBlockNum(){
        return mBlockNum;
    }
    /**
     * Sets a neighboor
     */
    public boolean setNeighbor(int edge,FieldBlock block){
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

    public boolean hasNeighbor(int edge){
        if (!isEdgeValid(edge))
            return false;
        return mNeighbors[edge] != null;
    }
}
