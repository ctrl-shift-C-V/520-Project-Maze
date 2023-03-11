# EEP 520 Final Project Maze

## Project Goal
The project goal is to create a maze like game. A robot is wandering in the maze trying to find the exit and the user and help the robot by controlling it using the W, A, S, D keys on the keyboard.

## Game setup
1. Install [git](https://git-scm.com/) and [Docker](https://www.docker.com/)
2. Clone this repo into your workspace
    ```
    > git clone https://github.com/ctrl-shift-C-V/520-Project-Maze.git
    ```
3. Build up the docker environment. 

    Open a new terminal windown

    ```
    > cd <your_workspace>/520-Project-Maze
    > docker run -p80:80 -p8765:8765 -v $PWD:/source -it klavins/enviro:v1.4 bash
    ```
4. Compile the code
    ```
    > esm start
    > make
    ```
5. Open your browser and go to http://localhost/. You should see something like this: ![Game image](studio/game_example.png)

6. Go back to the terminal and start the server
    ```
    enviro
    ```
7. Now go to the browser and you should see the game started. 
8. Play and Have Fun!
9. You can press `Ctrl-C` to stop the enviro server

## Game design & User Guide
The maze in this game is discretized into a 7x7 grid, with each block assigned a location `(x, y)`. The blocks are arranged in a left-to-right and top-to-bottom sequence, starting from `(0,0)` in the top left corner and ending with `(6,6)` in the bottom right corner. The robot starts from the `(0,0)` position and the exit is located at `(6,6)`.

To keep track of the number of times the robot visits each block, a 7x7 vector is used, with each value initialized to 0.

The robot moves by checking the surrounding blocks (block in front, block on the right, and block on the left) and selecting the block with the lowest number of visits.

Users can control the robot by clicking on the `Control Robot` button located in the upper right corner and using the W, A, S, and D keys to move the robot.

By clicking the `Self Wander` button, the robot will switch to wander mode, and users will no longer be able to control it.

The `Restart` button can be used to restart the game, with the robot respawning at the starting position `(0,0)` and the values of each block reset to 0.

Once the robot exits the maze, the game will automatically restart.

## Game Demo
![](studio/game_demo.gif)

## Challenges
### Robot not guaranteed to find the exit

As the robot wanders through the maze, there is no guarantee that it will find the exit. To address this issue, the maze is divided into a 7x7 grid, and a 7x7 matrix is used to track the number of times each block has been visited. The robot's movement is based on a simple algorithm that directs it to the block with the lowest number of visits among its surrounding blocks. This approach allows the robot to explore the maze more efficiently by prioritizing the unvisited blocks.

### Robot cannot turn precisely 90 degrees

The robot's movement is controlled by a series of rotations and translations. However, making the robot turn precisely 90 degrees is impossible. Instead, the solution is to teleport the robot to the desired angle, which provides a more accurate and consistent movement.

### Passing values between two controllers

Transferring values between a Process controller and a StateMachine controller can be challenging, as the two controllers operate independently and may not have direct access to each other's variables. To solve this problem, events are emitted between the two controllers, carrying essential information. This approach ensures that the controllers can communicate effectively without compromising their independence. However, it requires careful planning and coordination to ensure that the events are sent and received correctly.

## Accknowledgement

ENVIRO: The multi-agent, multi-user, multi-everything simulator
- https://github.com/klavinslab/enviro

[Elma](http://klavinslab.org/elma/):
- https://github.com/klavinslab/elma_project
- http://klavinslab.org/elma/functions_func.html

Course Material:
- https://github.com/sosper30/eep520

## LICENSE
[MIT License](/LICENSE)