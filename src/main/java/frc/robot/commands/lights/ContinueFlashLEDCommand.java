package frc.robot.commands.lights;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.BlinkinState;
import frc.robot.subsystems.LightControl;
import frc.utils.Timer;

public class ContinueFlashLEDCommand extends Command {
    
    private LightControl blinkin;
    private final BlinkinState flashPatternFirst;
    private final BlinkinState flashPatternSecond;
    private int flashTimeMillis;

    private Timer m_Timer;

    public ContinueFlashLEDCommand(LightControl blinkin, BlinkinState flashPatternFirst, BlinkinState flashPatternSecond, int flashTimeMillis) {
        this.blinkin = blinkin;
        this.flashPatternFirst = flashPatternFirst;
        this.flashPatternSecond = flashPatternSecond;
        this.flashTimeMillis = flashTimeMillis;

        addRequirements(blinkin);
    }

    @Override
    public void initialize() {
        m_Timer.resetTimer();
    }

    @Override
    public void execute() {
        if (!blinkin.isActive() && m_Timer.timePassed(flashTimeMillis)){
            m_Timer.resetTimer();
            blinkin.setPattern(flashPatternFirst);
        } else if (blinkin.isActive() && m_Timer.timePassed(flashTimeMillis)) {
            m_Timer.resetTimer();
            blinkin.setPattern(flashPatternSecond);
        }
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