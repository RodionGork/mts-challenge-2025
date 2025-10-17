# MTS True Tech Champ 2025

Here I'll put, with the kind approval of challenge officials, original materials
for the problems - and later my solutions will be added for reference (after
the corresponding rounds are finished).

You can run simulator and solve problems locally - you'll need to download
`Weebots` and load the "world" provided in the task materials into it, then
write a robot control program in any language (it communnicates by network)
and watch your robot moving through the room/maze.

The first problem simply wants us to make robot moving through the fixed maze
of known geometry. We learn here that odometer and time-based solutions are not
reliable and learn to manage robot using lidar. Best solution works in 65 seconds.

The second problem requests us to make robot navigating the maze. It would be
simple but the robot is the same "physical" car as in the first problem so
it becomes quite challenging.

The third problem wants us to drive the robot over the platform of complex shape,
which has no walls so it is easy to fall-off into abyss. The main sensor here
is the "depth camera". Regretfully the problem was not well prepared and even
was given to contestants in incomplete state with controller lacking movement
part at all. Contestants were told to finalize it themselves. No clear statement
was specified either.