## University Rotics Project

This is a project for my final year at the University of York.

The goal is to design and build a skid steered differential drive mobile robot, designing all aspects from the chassis, to the sensor placement, to the code and algorithms implemented. 
This robot should then be able to navigate and plot out a maze, and then be able to calculate the shortest path back to the "start".

Currently, the robot looks like this:



It uses the front array of sensors to look into clear space, and influence the current heading based upon where it sees a bigger gap. The position goal of the maze is also known, and using a weighted "score" function based upon gaps ahead of sensors and the direction of the goal, the controller navigates the robot around obstacles

If an obstacle is detected (flagged by "hit" boolean for each sensor), the robot stops and turns a set amount based upon the sensors that trigger the AVOID behaviour. Once clear, it resumes its navigation through the maze towards the goal.

Currently it is around 70% succesful at finding its wayt through the maze, but many parameters need tweaking! I need to next implement an amount of mapping which will be done through the use of "Nodes" at junction initially, perhaps with some amount of an occupany graph. Once a map has been created, some form of pathfinding logic will be implemented to allow the robot to navigate back to the start point of the maze, completing the requirements for the assessment.
