package frc.robot.commands;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;

/*public class MoveTowardsReefTest extends Command {

    private DriveSubsystem _drive;
    private final DoubleTopic centerTagXTopic;
    private final IntegerTopic centerImageXTopic;
    private final IntegerTopic numTargetsTopic;
    private final DoubleSubscriber tagSub;
    private final IntegerSubscriber imageSub;
    private final IntegerSubscriber numTargetsSub;
    private final long tolerance = 12;

    public MoveTowardsReefTest(DriveSubsystem drive, NetworkTableInstance inst) {
        _drive = drive;
        centerTagXTopic = inst.getDoubleTopic("/datatable/center_of_reef_X");
        centerImageXTopic = inst.getIntegerTopic("/datatable/center_of_image_X");
        numTargetsTopic = inst.getIntegerTopic("/datatable/num_targets_detected");
        tagSub = centerTagXTopic.subscribe(-1);
        imageSub = centerImageXTopic.subscribe(-1);
        numTargetsSub = numTargetsTopic.subscribe(-1);

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize(){
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
        double centerTagX = tagSub.get();
        long centerImageX = imageSub.get();
        long numTargets = numTargetsSub.get(); 

        // these values are probably incorrect because this is code taken from last year's robot with zero testing :)
        if (numTargets <= 0 || centerTagX <= 0) { // if there is no tag detected
            // Robot does not move
            _drive.drive(0,0,0,true, true);
        } else if (Math.abs(centerImageX-(long)centerTagX) <= tolerance) { // if there is a tag in the center of video
            // Robot drives forward
            _drive.drive(0.25, 0, 0, true, true);
        } else if (centerTagX < (long)centerImageX) { // if the tag is left of center
            // Robot drives to the left
            _drive.drive(0, -0.35, 0, true, true);
        } else if((long)centerTagX > centerImageX) { // if the tag is right of center
            // Robot drives to the right
            _drive.drive(0,0.35,0,true,true);
        } else {
            System.out.println("Undefined autonomous drive behavior");
        }
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _drive.drive(0,0,0,true,true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
    */