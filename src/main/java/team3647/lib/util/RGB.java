/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team3647.lib.util;

import static team3647.frc2020.robot.Constants.cPPSpinner;

import team3647.lib.wpi.HALMethods;

/**
 * Add your docs here.
 */
public class RGB {
    private double[] rgbArr = new double[3];
    public static RGB NONE = new RGB(new double[3]);

    public RGB(double[] rgbArr) {
        if (rgbArr != null) {
            this.rgbArr = rgbArr;
        } else {
            throw new IllegalArgumentException("The color percentage array was null!");
        }

        if (rgbArr.length != 3) {
            throw new IllegalArgumentException("The color percentage array was not exactly 3 elements");
        }
    }

    public boolean check(double[] rgbToCheck, double threshold) {
        if (rgbToCheck == null) {
            HALMethods.sendDSError("checking rgb values failed!\nthe array to check against was null");
            return false;
        } else if (rgbToCheck.length != 3) {
            HALMethods.sendDSError("checking rgb values failed\nthe array was not exactly 3 elements");
            return false;
        }
        boolean isEqual = true;
        for (int i = 0; i < rgbArr.length; i++) {
            isEqual = isEqual && isInThrehsold(rgbToCheck[i], rgbArr[i], threshold);
        }
        return isEqual;
    }

    private boolean isInThrehsold(double value, double constant, double threshold) {
        return value < constant + threshold && value > constant - threshold;
    }

    public static RGB fromString(String color) {
        switch (color) {
        case "R":
            return cPPSpinner.red;
        case "G":
            return cPPSpinner.green;
        case "B":
            return cPPSpinner.blue;
        case "Y":
            return cPPSpinner.yellow;
        // case "T":
        // return Constants.PPSpinner.test;
        default:
            return NONE;
        }
    }
}