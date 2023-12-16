package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SubsystemsInstance;

public class ChangeFieldRelativeCommand extends Command {
    
public ChangeFieldRelativeCommand() {

}

@Override
public void execute() {
    SubsystemsInstance.getInstance().drivetrain.fieldRelative = !SubsystemsInstance.getInstance().drivetrain.fieldRelative;
}

@Override
  public boolean isFinished() {
    return true;
  }

}
