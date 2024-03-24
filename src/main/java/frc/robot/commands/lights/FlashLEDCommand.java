package frc.robot.commands.lights;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.BlinkinState;
import frc.robot.subsystems.LightControl;

public class FlashLEDCommand extends Command {
    
    private LightControl blinkin;
    private final BlinkinState flashPattern;
    private final BlinkinState holdPattern;
    private int flashTimeMillis;
    private int timesToFlash;

    private int loops;
    private long timer;

    public FlashLEDCommand(LightControl blinkin, BlinkinState flashPattern, BlinkinState holdPattern, int flashTimeMillis, int timesToFlash) {
        this.blinkin = blinkin;
        this.flashPattern = flashPattern;
        this.holdPattern = holdPattern;
        this.flashTimeMillis = flashTimeMillis;
        this.timesToFlash = timesToFlash;

        addRequirements(blinkin);
    }

    @Override
    public void initialize() {
        loops = 0;
        timer = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        if (!blinkin.isActive() && System.currentTimeMillis() > timer + flashTimeMillis) {
            timer = System.currentTimeMillis();
            blinkin.setPattern(flashPattern);
            loops++;
        } else if (blinkin.isActive() && System.currentTimeMillis() > timer + flashTimeMillis) {
            timer = System.currentTimeMillis();
            blinkin.turnOff();
        }
        if (loops > timesToFlash) blinkin.setPattern(holdPattern);
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