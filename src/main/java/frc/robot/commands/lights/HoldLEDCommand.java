package frc.robot.commands.lights;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.BlinkinState;
import frc.robot.subsystems.LightControl;

public class HoldLEDCommand extends Command {
    
    private LightControl blinkin;
    private final BlinkinState holdPattern;
   

    private int loops;
    private long timer;

    public HoldLEDCommand(LightControl blinkin, BlinkinState holdPattern) {
        this.blinkin = blinkin;
        this.holdPattern = holdPattern;
        

        addRequirements(blinkin);
    }

    @Override
    public void initialize() {
        loops = 0;
        timer = 0;
    }

    @Override
    public void execute() {
        blinkin.setPattern(holdPattern);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        blinkin.turnOff();
    }
}