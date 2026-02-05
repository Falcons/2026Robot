package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb.Climb;

public class ClimbCommand extends Command {
  private final Climb climb;
  private double speed;
  
  /** Creates a new Climb. */
  public ClimbCommand(Climb climb, double speed) {
    this.climb = climb;
    this.speed = speed;
    addRequirements(climb);
  
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(this.getName() + " start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.setClimb(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.stopClimb();
    System.out.println(this.getName() + " end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
    
}
