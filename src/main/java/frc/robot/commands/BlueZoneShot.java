package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class BlueZoneShot extends CommandGroup {
    
    public BlueZoneShot() {

        addSequential(new RevShooter());
        addParallel(new TimedShoot());
        addSequential(new LoadBallTimed(5));
        addSequential(new Space(3));

    }
}