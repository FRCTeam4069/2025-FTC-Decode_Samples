package org.firstinspires.ftc.teamcode.pedro.commands;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/** Runs independent commands and calls their cleanup when stopped. */
public class Scheduler {
    private final List<Command> commands = new ArrayList<>();
    public void schedule(Command command) { command.initialize(); commands.add(command); }
    public void run() {
        Iterator<Command> iterator = commands.iterator();
        while (iterator.hasNext()) {
            Command command = iterator.next();
            command.execute();
            if (command.isFinished()) { command.end(false); iterator.remove(); }
        }
    }
    public void cancelAll() {
        for (Command command : commands) command.end(true);
        commands.clear();
    }
}
