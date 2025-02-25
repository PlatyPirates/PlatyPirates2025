package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.MAXSwerveModule;
import frc.robot.subsystems.DriveSubsystem;

public class DriveRobotFromLimelight extends Command {

    private DriveSubsystem _DriveSubsystem;

    public DriveRobotFromLimelight(DriveSubsystem DriveSubsystem) {
        _DriveSubsystem = DriveSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {    
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double kP = .035;
        double targetingAngularVelocity = LimelightHelpers.getTX("")*kP;
        targetingAngularVelocity *= -1.0*DriveConstants.kMaxAngularSpeed;

        boolean hasTarget = LimelightHelpers.getTV("");
        if (hasTarget && (LimelightHelpers.getTX("") != 0)) {
             _DriveSubsystem.drive(0.03*LimelightHelpers.getTX(""), 0, targetingAngularVelocity, true);
        } else if(hasTarget && (LimelightHelpers.getTX("") == 0)){
            _DriveSubsystem.drive(0, 0, 0, true);
        }
    }
       
    

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _DriveSubsystem.drive(0.0, 0.0, 0.0, true);
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}