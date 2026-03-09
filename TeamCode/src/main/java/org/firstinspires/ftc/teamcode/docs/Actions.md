# So... Actions
Actions were created as a way to solve the issues many programmers find in autonomous programming, or in general doing things sequentially while in a loop.

## Why this over a state machine
A state machine is a totally valid system to accomplish much of what our Actions system does.
Actions were made as a reaction to state machines because I ([Alex](https://github.com/26-AMoore)) dislikes state machines. In my opinion they are great for smaller projects, or big projects where you have everything pre-planned, they become burdensome in our usecase.
I have always found state machines hard to follow and hard to debug after they grow a little bit. Also, most state machines are implemented poorly. If a state machine is used it should use enums, and should be very clear about what each state does and have specific functions that change state.
Actions were made as a way to avoid a state machine, and allow for easy insertion or deletion of steps within the state machine, They were also made following the philosophy of do one thing and do it right.

## How to use
Actions are actually very very very simple to use.
In fact, there is only one (normally) method in each.
Each Action has a function called `run()` This function is called repeatedly until it returns false.
That's it. Actions are that simple.

### Main actions
Whenever you wish to create an action to use in your program, it is normal to just use the Action type. Unless you are using an action in a teleOp function then I would not recommend doing anything differently.
The main types of actions you will use when creating your program are:
1. Sequential Actions
2. Parallel Actions

Sequential Actions are used when you wish to do something in order. A Sequential Action will call the first action in its list until it returns false, then it will call the next until it returns false, until all are finished.
A Parallel Action is for when you want to do something at the same time. It calls every Action that was passed to it every time it is called. This happens until every action in the list returns false.

### Examples
Below is a simple example of what action syntax looks like.
```java
Action action;
boolean isRunning;

@Override
public void setup() {
    isRunning = true;

    action = // While not necessary I like using a newline for the first action, as it makes the "layers" more obvious
    new ParallelAction( // Think of actions as layers. This parallel action runs each of the actions within it
            new SequentialAction( // The first action run in the parallel action is this sequential action, which runs each action in order until it has completed
                   new FollowPathAction(follower, paths.Curve1), // This FollowPathAction makes the robot follow a pedropathing path, and finises once the robot completes the path
                   new SleepAction(1000), // After the robot has followed the path, the sequential action sleeps for one second **Note that when the sequential action is sleeping, the other actions in the parallel action still get run
                   new FollowPathAction(follower, paths.Straight2), // Then the robot follows another path, same way as the first
                   robot.doSomethingAction(), // After we finish the previous action we call robot.doSomethingAction() which presumably does something
                   new FollowPathAction(follower, paths.Wiggle3), // then we follow another path
                   new FollowPathAction(follower, paths.Straight4) // And to finish its one more path
            ),

            // These next parts are called every time run is called on the ParallelAction.
            robot.savePosition(), // We save the robots position
            new InstantAction(() -> telemetry.update()) // and this is a weird one but we update the telemetry
    );
}

@Overried
public void loop() {
    if (isRunning)
        isRunning = action.run(); // This is a rudimentary way to run the action until it is finished
}
```

### Notes
Do not turn all of your functions into actions. That gets really really annoying. In general something should be an action only if it must be run repeatedly until something is achieved.
If you have a function which does not meet this criteria, but still gets called a lot in an autonomous, it is recommended to either make a wrapper function for it.
```java
// Here is an example of a normal function that you wish to turn into an action
public void updateColors() {
    updateRed();
    updateGreen();
    updateBlue();
}

// And the wrapper function for it
public Action updateColorsAction() {
    return new Action() {
        @Override
            public boolean run() {
                updateColors(); // see how it **wraps** the normal function
                return false;
            }
    };
}
```
The problem with a wrapper like this, is that it is really ugly.
Thus a action was developed to make turning a normal function into an action much more ergonomic: The InstantAction
An Instant action easily wraps functions that return void or anything else, and turns them into an action.
```java
// The syntax is a little weird but like its much cleaner than the wrapper function if you only need to call it once or twice
new InstantAction(() -> updateColors());
```
