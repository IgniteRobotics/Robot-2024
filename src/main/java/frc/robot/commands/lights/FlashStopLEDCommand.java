package frc.robot.commands.lights;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.BlinkinState;
import frc.robot.subsystems.LightControl;

public class FlashStopLEDCommand extends Command {
    
    private LightControl blinkin;
    private final BlinkinState flashPattern;
    private int flashTimeMillis;
    private int timesToFlash;

    private int loops;
    private long timer;

    public FlashStopLEDCommand(LightControl blinkin, BlinkinState flashPattern, int flashTimeMillis, int timesToFlash) {
        this.blinkin = blinkin;
        this.flashPattern = flashPattern;
        this.flashTimeMillis = flashTimeMillis;
        this.timesToFlash = timesToFlash;

        addRequirements(blinkin);
    }

    @Override
    public void initialize() {
        blinkin.StartFlashPeriod();
        loops = 0;
        timer = 0;
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
        if (loops > timesToFlash) this.end(true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        blinkin.turnOff();
        blinkin.endPeriod();
    }
}