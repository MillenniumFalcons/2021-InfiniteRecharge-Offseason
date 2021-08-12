/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.lib.util;

/**
 * Add your docs here.
 */
public class VisionTarget {
    private final int valid;
    private final double x;
    private final double y;
    private final double area;
    private final double skew;
    private final double latency;
    private final int pipeline;
    private final double distance;

    public VisionTarget(int valid, double x, double y, double area, double skew, double latency,
            int pipeline, double distance) {
        this.valid = valid;
        this.x = x;
        this.y = y;
        this.area = area;
        this.skew = skew;
        this.latency = latency;
        this.pipeline = pipeline;
        this.distance = distance;
    }

    /**
     * @return the distance
     */
    public double getDistance() {
        return distance;
    }

    /**
     * @return the pipeline
     */
    public int getPipeline() {
        return pipeline;
    }

    /**
     * @return the latency
     */
    public double getLatency() {
        return latency;
    }

    /**
     * @return the skew
     */
    public double getSkew() {
        return skew;
    }

    /**
     * @return the area
     */
    public double getArea() {
        return area;
    }

    /**
     * @return the y
     */
    public double getY() {
        return y;
    }

    /**
     * @return the x
     */
    public double getX() {
        return x;
    }

    /**
     * @return the valid
     */
    public int getValid() {
        return valid;
    }

}
