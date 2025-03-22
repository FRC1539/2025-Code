package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.RunAtVelocity;
import frc.robot.subsystems.drive.DriveSubsystem;

public class testAuto extends SequentialCommandGroup {
    public testAuto(DriveSubsystem driveSubsystem){
        addCommands(new RunAtVelocity(driveSubsystem, 0.2, 0, 0));
    }
}