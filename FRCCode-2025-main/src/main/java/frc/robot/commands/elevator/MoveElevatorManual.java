package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.PortConstants.Controller;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.wpilibj.XboxController;


public class MoveElevatorManual extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    XboxController joystick;

    public MoveElevatorManual(ElevatorSubsystem elevatorSubsystem, XboxController joystick) {
        this.joystick = joystick;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.moveAtSpeed(0); // Makes sure it does not keep on moving.
    }

    @Override
    public void execute() {
        elevatorSubsystem.moveAtSpeed(joystick.getLeftY());
        // elevatorSubsystem.setMotorVoltage(-.2);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.moveAtSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}