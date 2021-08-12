/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team3647.lib.util;

public class RollingAverage {

    private int size;
    private double total = 0d;
    private int index = 0;
    private double samples[];

    public RollingAverage(int size) {
        this.size = size;
        samples = new double[size];
        for (int i = 0; i < size; i++)
            samples[i] = 0d;
    }

    public RollingAverage() {
        this(4);
    }

    public void add(double x) {
        total -= samples[index];
        samples[index] = x;
        total += x;
        if (++index == size)
            index = 0;
    }

    public double getAverage() {
        return total / size;
    }
}