package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import monologue.Logged;
import monologue.Annotations.Log;

public class StateMachine<T extends Enum<T>> implements Logged {
    public EventLoop m_loop;
    public StateMachine(T init, T idle, EventLoop loop) {
      m_state = init;
      m_idleState = idle;
      m_loop = loop;
    }
    public StateMachine(T init, T idle) {
        this(init, init, CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    public StateMachine(T init) {
      this(init, init);
    }
    public StateMachine(T init, EventLoop loop) {
        this(init, init, loop);
    }
    @Log
    public T m_state;
    @Log.Once
    public T m_idleState;
    private Map<T, Trigger> stateTrgs;
    public Trigger trg(T state) {
      return stateTrgs.computeIfAbsent(state,st->new Trigger(m_loop, ()->m_state==st));
    }
    public Command setState(T state) {
      return Commands.runOnce(()->m_state = state).ignoringDisable(true);
    }
    public Command clearState() {
        return setState(m_idleState);
    }
  }
