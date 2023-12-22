package org.firstinspires.ftc.teamcode.centerstage.util;

import org.firstinspires.ftc.robotcore.external.Function;

import java.util.ArrayList;
import java.util.Dictionary;
import java.util.List;

public class EventListener {
    private static List<Event> currentEvents;

    public static class Event {
        public Dictionary<String, Object> arguments;
        String name;

        public Event(String name, Dictionary<String, Object> arguments) {
            this.name = name;
            this.arguments = arguments;
        }

        public String getName() {
            return name;
        }
    }

    static {
        currentEvents = new ArrayList<>();
    }

    public static void sendEvent(String name, Dictionary<String, Object> args) {
        currentEvents.add(new Event(name, args));
    }

    public static void onEvent(String name, Function<Event, Event> doEvent) {
        for (Event event : currentEvents) {
            if (event.getName().equals(name)) {
                doEvent.apply(event);
            }
        }
    }
}
