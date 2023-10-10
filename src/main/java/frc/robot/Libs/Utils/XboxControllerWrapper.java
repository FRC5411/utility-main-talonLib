package frc.robot.Libs.Utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxControllerWrapper {
    private CommandXboxController commandController;
    private XboxController controller;

    private axisConstraints leftXConstraints;
    private axisConstraints leftYConstraints;
    private axisConstraints rightYConstraints;
    private axisConstraints rightXConstraints;

    XboxControllerWrapper(int port, axisConstraints[] joyStickConstraints) {
        this.commandController = new CommandXboxController(port);
        this.controller = commandController.getHID();

        this.leftXConstraints = joyStickConstraints[0];
        this.leftYConstraints = joyStickConstraints[1];
        this.rightXConstraints = joyStickConstraints[2];
        this.rightYConstraints = joyStickConstraints[3];
    }

    public CommandXboxController getCommandController() { return commandController; }

    public XboxController getController() { return controller; }

    public double getAdjustedLeftX() { return - adjustJoystick(leftXConstraints, commandController.getLeftX()); }

    public double getAdjustedLeftY() { return adjustJoystick(leftYConstraints, commandController.getLeftY()); }

    public double getAdjustedRightX() { return adjustJoystick(rightXConstraints, commandController.getRightX()); }

    public double getAdjustedRightY() { return adjustJoystick(rightYConstraints, commandController.getRightY()); }

    public void onClick(Trigger trigger, Command trueCommand, Command falseCommand) {
        trigger
            .onTrue(trueCommand)
            .onFalse(falseCommand);
    }

    public void whileClick(Trigger trigger, Command whileCommand, Command falseCommand) {
        trigger
            .whileTrue(whileCommand)
            .onFalse(falseCommand);
    }

    public void onWhileClick(Trigger trigger, Command trueCommand, Command whileCommand, Command falseCommand) {
        trigger
            .onTrue(trueCommand)
            .whileTrue(whileCommand)
            .onFalse(falseCommand);
    }

    public double adjustJoystick(axisConstraints constraints, double value) {
        return adjustJoystick(
            constraints.inversion, 
            constraints.maxSpeed, 
            constraints.deadBand, 
            constraints.rateLimiter, 
            value);
    }

    public double adjustJoystick(double inversion, double maxSpeed, double deadBand, SlewRateLimiter rateLimiter, double value) {
        value = MathUtil.applyDeadband(value, deadBand);
        value *= Math.signum(inversion);
        value *= maxSpeed;
        value = rateLimiter.calculate(value);
        return value;
    }

    public class axisConstraints {
        public double inversion;
        public double maxSpeed;
        public double deadBand;
        public SlewRateLimiter rateLimiter;

        public axisConstraints(double inversion, double maxSpeed, double deadBand, SlewRateLimiter rateLimiter) {
            this.inversion = inversion;
            this.maxSpeed = maxSpeed;
            this.deadBand = deadBand;
            this.rateLimiter = rateLimiter;
        }
    }
}