package frc.robot.commands.lights;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.BlinkinState;
import frc.robot.subsystems.LightControl;
import frc.utils.Timer;

public class FlashStopLEDCommand extends Command {
    
    private LightControl blinkin;
    private final BlinkinState flashPattern;
    private int flashTimeMillis;
    private int timesToFlash;

    private int loops;
    private Timer m_Timer;

    public FlashStopLEDCommand(LightControl blinkin, BlinkinState flashPattern, int flashTimeMillis, int timesToFlash) {
        this.blinkin = blinkin;
        this.flashPattern = flashPattern;
        this.flashTimeMillis = flashTimeMillis;
        this.timesToFlash = timesToFlash;

        addRequirements(blinkin);
    }

    @Override
    public void initialize() {
        loops = 0;
        m_Timer.resetTimer();;
    }

    @Override
    public void execute() {
        if (!blinkin.isActive() && m_Timer.timePassed(flashTimeMillis)) {
            m_Timer.resetTimer();
            blinkin.setPattern(flashPattern);
            loops++;
        } else if (blinkin.isActive() && m_Timer.timePassed(flashTimeMillis)) {
            m_Timer.resetTimer();
            blinkin.turnOff();
        }
    }

    @Override
    public boolean isFinished() {
        return loops > timesToFlash;
    }

    @Override
    public void end(boolean interrupted) {
        blinkin.turnOff();
    }
}