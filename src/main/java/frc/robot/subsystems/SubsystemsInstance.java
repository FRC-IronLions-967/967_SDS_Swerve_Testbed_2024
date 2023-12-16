package frc.robot.subsystems;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class SubsystemsInstance {
    public Drivetrain drivetrain;
   
    private static SubsystemsInstance inst;

    private final SendableChooser<Command> autoChooser;

    private SubsystemsInstance() {
        drivetrain = new Drivetrain();
        autoChooser = AutoBuilder.buildAutoChooser("Default_Auto");

        CommandScheduler.getInstance().registerSubsystem(drivetrain);
        
        
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }
    public static SubsystemsInstance getInstance () {
        if(inst == null) inst = new SubsystemsInstance();

        return inst;

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}