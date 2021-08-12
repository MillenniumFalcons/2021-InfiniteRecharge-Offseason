/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.inputs;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import team3647.lib.wpi.JoystickTrigger;

/**
 * Add your docs here.
 */
public class Joysticks {
    public final JoystickTrigger leftTrigger;
    public final JoystickTrigger rightTrigger;

    public final JoystickButton rightJoyStickPress;
    public final JoystickButton leftJoyStickPress;
    public final JoystickButton leftMidButton;
    public final JoystickButton rightMidButton;

    public final JoystickButton rightBumper;
    public final JoystickButton leftBumper;
    public final JoystickButton buttonA;
    public final JoystickButton buttonB;
    public final JoystickButton buttonY;
    public final JoystickButton buttonX;
    public final POVButton dPadDown;
    public final POVButton dPadLeft;
    public final POVButton dPadRight;
    public final POVButton dPadUp;

    /**
     * XboxController Object for Controller; contains all Xbox Controller Functions
     */
    private final XboxController controller;

    private final int controllerPin;


    public Joysticks(int controllerPin) {
        controller = new XboxController(controllerPin);
        this.controllerPin = controllerPin;

        leftTrigger = new JoystickTrigger(controller, XboxController.Axis.kLeftTrigger.value, .15);
        rightTrigger =
                new JoystickTrigger(controller, XboxController.Axis.kRightTrigger.value, .15);

        rightJoyStickPress = new JoystickButton(controller, 10);
        leftJoyStickPress = new JoystickButton(controller, 9);
        leftMidButton = new JoystickButton(controller, XboxController.Button.kBack.value);
        rightMidButton = new JoystickButton(controller, XboxController.Button.kStart.value);

        rightBumper = new JoystickButton(controller, XboxController.Button.kBumperRight.value);
        leftBumper = new JoystickButton(controller, XboxController.Button.kBumperLeft.value);
        buttonA = new JoystickButton(controller, XboxController.Button.kA.value);
        buttonB = new JoystickButton(controller, XboxController.Button.kB.value);
        buttonY = new JoystickButton(controller, XboxController.Button.kY.value);
        buttonX = new JoystickButton(controller, XboxController.Button.kX.value);

        dPadDown = new POVButton(controller, 180);
        dPadLeft = new POVButton(controller, 270);
        dPadRight = new POVButton(controller, 90);
        dPadUp = new POVButton(controller, 0);
    }

    public double getLeftStickX() {
        return applyDeadband(controller.getRawAxis(XboxController.Axis.kLeftX.value));
    }

    public double getLeftStickY() {
        return applyDeadband(-controller.getRawAxis(XboxController.Axis.kLeftY.value));
    }

    public double getRightStickX() {
        return applyDeadband(controller.getRawAxis(XboxController.Axis.kRightX.value));
    }

    public double getRightStickY() {
        return applyDeadband(-controller.getRawAxis(XboxController.Axis.kRightY.value));
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value    value to clip
     * @param deadband range around zero
     */
    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value    value to clip
     * @param deadband range around zero
     */
    private double applyDeadband(double value) {
        return applyDeadband(value, 0.09);
    }

    public int getControllerPin() {
        return controllerPin;
    }
}

