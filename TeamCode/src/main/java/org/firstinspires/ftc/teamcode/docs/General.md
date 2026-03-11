# This is a place for general and informal information and thoughts
It is very VERY informal
And kinda just rants and thoughts from [Alex](https://github.com/26-AMoore)

## Tips
### Git
- with git, make a new branch for every person and breaking changes
- use good commit messages (i might be hypocritical)
- learn to use the command line with git, it will pay off immensely
- if you have two people developing at the same time, two branches you repeatedly sync is a very easy way to not face merge conflicts, just make sure you never touch the same file
### Android studio
- Use the LSP (when you type code, the tab key should be one of your most pressed)
- Using the LSP also lets you read the docs and see all associated methods, you don't need to remember anything
- Setup sloth. It will save literally hours
### General Programming
- USE ENUMS. Enums are such a good way to model data. Using enums with an attached data type like our RobotPart class does is really powerful. Enums are so powerful.
- Do one thing and do it well. Make one function to do something, like turn a light on, and then only use that function
- READ DOCUMENTATION oh my god read documentation. I know it can be a little boring at times, but 99% of the time you can figure things out with the LSP tips mentioned earlier
- Keep things segmented. Try to take things as low level as possible to start. Make a class that just turns a light on and off. Then if you need to do that based on some other part of the robot, either pass in the other part to **a function** in the light class or make a class to combine them, like the Robot class.
- Do not be too abstract. This is ironic after the previous tip, but you do not need to make a wrapper class for everything. This goes hand in hand with the next tip
- Done is better than perfect. After you write any code it's really easy to rethink it and want to re-factor. You can give into this urge, but make sure it is actually useful, and will not only be used once.
- Write re-usable code. In general your code is going to be specific to whatever years robot. This is okay, but if you use any systems which are not directly tied to that robot, like a light try to make it usable for future teams.
