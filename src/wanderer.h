#ifndef __WANDERER_AGENT__H
#define __WANDERER_AGENT__H

#include <string>
#include <math.h>
#include "enviro.h"
#include <vector>
#include <json/json.h>
#include <climits>

namespace
{

    using namespace enviro;

    //! A static class that represents a 7x7 matrix that keeps track of how many times each block has been visited.

    //! An example shows the robot has visited (0,0) 1 time, (0,1) 1 time, (0,2) 1 time and (0,3) one time
    //! 1, 1, 1, 1, 0, 0, 0
    //! 0, 0, 0, 0, 0, 0, 0
    //! 0, 0, 0, 0, 0, 0, 0
    //! 0, 0, 0, 0, 0, 0, 0
    //! 0, 0, 0, 0, 0, 0, 0
    //! 0, 0, 0, 0, 0, 0, 0
    //! 0, 0, 0, 0, 0, 0, 0
    class SeenMap 
    {
    public:
        //! Get the value of the block located in the given (x, y) position of the matrix.
        //! \param x (int) the column of the 7x7 matrix.
        //! \param y (int) the row of the 7x7 matrix.
        //! \return value of the block, if the (x, y) is out of boundary, it returns the maximum integer value.
        static int get_value(int x, int y) {
            if (x > 6 || x < 0 || y < 0 || y > 6) {
                return INT_MAX;
            }
            return map[y][x];
        }

        //! Increases the value of the block located in the given (x, y) position of the matrix by one.
        //! \param x (int) the column of the 7x7 matrix.
        //! \param y (int) the row of the 7x7 matrix.
        static void set_to_visited(int x, int y) {
            map[x][y] = map[x][y] + 1;
        }

        //! Resets all the values in the matrix to 0.
        static void reset_map(){
            for(int i = 0; i<7; i++){
                for(int j = 0; j<7; j++) {
                    map[i][j] = 0;
                }
            }
        }

        //! Prints the current state of the matrix to the standard output in the form of comma-separated values.
        static void draw() {
            for(int i = 0; i<7; i++){
                for(int j = 0; j<7; j++) {
                    std::cout << map[i][j] << ",";
                }
                std::cout<<std::endl;
            }
        }

    private:
    //! 2D vector that holds the values of the blocks in the matrix.
    static std::vector<std::vector<int>> map;
    };

    //! Initialize the matrix, the values are initialized to 0 by default, the size is 7x7.
    std::vector<std::vector<int>> SeenMap::map(7, std::vector<int>(7, 0));


    //! Moving state, the robot should move to a specific location
    class Moving : public State, public AgentInterface
    {
    public:
        //! A method that derived instances should define. It is called when the state is
        //! entered by the state machine either when the machine starts or when a transition
        //! to the state is fired.
        //! \param e The event that led to the transition into the state, the value of the event is of form {x, y}
        void entry(const Event &e) {
            pos = e.value();
            goal_x = 50.0+pos[0].get<int>()*100.0-350.0; // The x position the robot should move to
            goal_y = 50.0+pos[1].get<int>()*100.0-350.0; // The y position the robot should move to
            if (goal_x == 300 && goal_y == 400) {
                goal_y = 350.0; // The boundry of the maze is 350.
            }
            int cur_column = (position().x + 350.0) / 100.0;
            int cur_row = (position().y + 350.0) / 100.0;
            SeenMap::set_to_visited(cur_row, cur_column); // Set current block to visited
            // SeenMap::draw();
        }

        void during()
        {
            cur_x = position().x;
            cur_y = position().y;
            // Keep robot moving toward the goal position.
            if (std::fabs(cur_x - goal_x) >= 3 || std::fabs(cur_y - goal_y) >= 3){
                move_toward(goal_x, goal_y, 30, 0);
            } else {
                // teleport the robot to the goal position when it is close enough
                teleport(goal_x, goal_y, angle());
                if(goal_x == 300.0 && goal_y == 350.0){
                    emit(Event("restart")); // Robot at maze exit
                } else {
                    emit(Event("check"));
                }
            }
            
        }
        void exit(const Event &e) {
        }
        void set_tick_name(std::string s) { tick_name = s; }
        std::string tick_name;
        double goal_x;
        double goal_y;
        double cur_x;
        double cur_y;
        json pos;
    };

    //! Rotating state, the robot should rotate 90 degrees
    class Rotating : public State, public AgentInterface
    {
    public:
        //! A method that derived instances should define. It is called when the state is
        //! entered by the state machine either when the machine starts or when a transition
        //! to the state is fired.
        //! \param e The event that led to the transition into the state, the value of the event is -1 or 0 or 1
        //! the robot turns clockwise when value is 1, turns counter-clockwise when value is -1
        void entry(const Event &e) { 
            rate = e.value();
            int degree_counter = angle() / 1.57;
            // Calculate the goal angle
            if (degree_counter >= 0) {
                goal_angle = angle() - degree_counter * 1.57 >= 1.57/2 ? (degree_counter + 1) * 1.57 : degree_counter * 1.57;
            } else {
                goal_angle = std::fabs(angle() - degree_counter * 1.57) >= 1.57/2 ? (degree_counter-1) * 1.57 : degree_counter * 1.57;
            }
            if (rate != 0) {
                goal_angle = goal_angle + rate * 1.57;
            }
        }
        void during()
        {
            // Set goal angle to 0 if the robot's angle is over a full circle (6.28)
            if (goal_angle == 6.28 || goal_angle == -6.28) {
                teleport(position().x, position().y, 0);
            } else {
                teleport(position().x, position().y, goal_angle);
            }
            emit(Event("check"));

        }
        void exit(const Event &e) {}
        double rate;
        void set_tick_name(std::string s) { tick_name = s; }
        std::string tick_name;
        double goal_angle;
    };

    //! Checking state, analyze current state and figure out next movement
    //!
    //! The robot will figure out its current heading direction and decided where to go
    class Checking : public State, public AgentInterface
    {
    public:
        void entry(const Event &e) {
        }
        void during()
        {
            pos = position();
            int cur_x = (pos.x + 350.0) / 100.0;
            int cur_y = (pos.y + 350.0) / 100.0;
            // Calculate where the robot is currently heading
            double cur_angle = remainder(angle(), 1.57*4) >= 0 ? remainder(angle(), 1.57*4) : 6.28 + remainder(angle(), 1.57*4);
            double direction = cur_angle/(1.57/2);
            // robot at maze exit
            if (cur_x == 6 and cur_y == 6) {
                if(direction < 1|| direction >=7) {
                    emit(Event("rotate", 1)); // Turn right to face the exit
                } else if (direction >= 1 && direction < 3) {
                    emit(Event("move", {cur_x, cur_y+1})); // Move to the boundry (exit)
                } else if (direction >= 3 && direction < 5) {
                    emit(Event("rotate", -1));
                } else {
                    emit(Event("rotate", 1));
                }
            } else {
                if (direction <1 || direction >=7){ // Heading right
                    // No obstacles at all three directions, rigt, front, left
                    if(sensor_value(0) > 100 && sensor_value(1) > 100 && sensor_value(2) > 100){
                        // Block in front has the lowest visited times compare to block on the right and block on the left
                        if(SeenMap::get_value(cur_x+1, cur_y) <= SeenMap::get_value(cur_x, cur_y-1) && SeenMap::get_value(cur_x+1, cur_y) <= SeenMap::get_value(cur_x, cur_y+1)) {
                            emit(Event("move", {cur_x+1, cur_y})); // Move forward
                        // Block on the right has the lowest visited times compare to block in front and block on the left
                        } else if(SeenMap::get_value(cur_x, cur_y+1) <= SeenMap::get_value(cur_x+1, cur_y) && SeenMap::get_value(cur_x, cur_y+1) <= SeenMap::get_value(cur_x, cur_y-1)) {
                            emit(Event("rotate", 1)); // Turn right
                        } else {
                            emit(Event("rotate", -1)); // Turn left
                        }
                    // No obstacles in front and on the right but on the left
                    } else if (sensor_value(0) > 100 && sensor_value(1) > 100){
                        // Block in front has lower visited times compare to the block on the right
                        if(SeenMap::get_value(cur_x+1, cur_y) <= SeenMap::get_value(cur_x, cur_y+1)){
                            emit(Event("move", {cur_x+1, cur_y})); // Move forward
                        }else{
                            emit(Event("rotate", 1)); // Turn right
                        }
                    // No obstacles in front and on the left but on the right
                    } else if (sensor_value(0) > 100 && sensor_value(2) > 100){
                        // Block in front has lower visited times compare to the block on the left
                        if(SeenMap::get_value(cur_x+1, cur_y) <= SeenMap::get_value(cur_x, cur_y-1)){
                            emit(Event("move", {cur_x+1, cur_y})); // Move forward
                        }else{
                            emit(Event("rotate", -1)); // Turn left
                        }
                    // No obstacles on the right and on the left but in front
                    } else if (sensor_value(1) > 100 && sensor_value(2) > 100) {
                        // Block on the right has lower visited times compare to block on the left
                        if(SeenMap::get_value(cur_x, cur_y+1) <= SeenMap::get_value(cur_x, cur_y-1)){
                            emit(Event("rotate", 1)); // Turn right
                        } else {
                            emit(Event("rotate", -1)); // Tuen left
                        }
                    } else if (sensor_value(0) > 100){
                        emit(Event("move", {cur_x+1, cur_y}));
                    } else if (sensor_value(1) > 100) {
                        emit(Event("rotate", 1));
                    } else if (sensor_value(2) > 100) {
                        emit(Event("rotate", -1));
                    } else {
                        emit(Event("rotate", 1));
                    }
                } else if (direction >= 1 && direction <3 ){ // Heading down
                    // No obstacles in all three directions, front, right, left
                    if(sensor_value(0) > 100 && sensor_value(1) > 100 && sensor_value(2) > 100){
                        // Block in front has the lowest visited times compare to block on the right and block on the left
                        if(SeenMap::get_value(cur_x, cur_y+1) <= SeenMap::get_value(cur_x+1, cur_y) && SeenMap::get_value(cur_x, cur_y+1) <= SeenMap::get_value(cur_x-1, cur_y)) {
                            emit(Event("move", {cur_x, cur_y+1})); // Move forward
                        // Block on the right has the lowest visited times compare to block in front and block on the left
                        } else if(SeenMap::get_value(cur_x-1, cur_y) <= SeenMap::get_value(cur_x, cur_y+1) && SeenMap::get_value(cur_x-1, cur_y) <= SeenMap::get_value(cur_x+1, cur_y)) {
                            emit(Event("rotate", 1)); // Turn right
                        } else {
                            emit(Event("rotate", -1)); // Turn left
                        }
                    // No obstacles in front and on the right but on the left
                    } else if (sensor_value(0) > 100 && sensor_value(1) > 100){
                        // Block in front has lower visited times compare to the block on the right
                        if(SeenMap::get_value(cur_x, cur_y+1) <= SeenMap::get_value(cur_x-1, cur_y)){
                            emit(Event("move", {cur_x, cur_y+1})); // Move forward
                        }else{
                            emit(Event("rotate", 1)); // Turn right
                        }
                    //No obstacles in front and on the left but on the right
                    } else if (sensor_value(0) > 100 && sensor_value(2) > 100){
                        // Block in front has lower visited times compare to the block on the left
                        if(SeenMap::get_value(cur_x, cur_y+1) <= SeenMap::get_value(cur_x+1, cur_y)){
                            emit(Event("move", {cur_x, cur_y+1})); // Move forward
                        }else{
                            emit(Event("rotate", -1)); // Turn left
                        }
                    // No obstacles on the right and on the left but in front
                    } else if (sensor_value(1) > 100 && sensor_value(2) > 100) {
                        // Block on the right has lower visited times compare to block on the left
                        if(SeenMap::get_value(cur_x-1, cur_y) <= SeenMap::get_value(cur_x+1, cur_y)){
                            emit(Event("rotate", 1)); // Turn right
                        } else {
                            emit(Event("rotate", -1)); // Turn left
                        }
                    } else if (sensor_value(0) > 100){
                        emit(Event("move", {cur_x, cur_y+1}));
                    } else if (sensor_value(1) > 100) {
                        emit(Event("rotate", 1));
                    } else if (sensor_value(2) > 100) {
                        emit(Event("rotate", -1));
                    } else {
                        emit(Event("rotate", 1));
                    }
                } else if (direction >= 3 && direction < 5){ // Heading left
                    // No obstacles in all three directions, front, right, left
                    if(sensor_value(0) > 100 && sensor_value(1) > 100 && sensor_value(2) > 100){
                        // Block in front has the lowest visited times compare to block on the right and block on the left
                        if(SeenMap::get_value(cur_x-1, cur_y) <= SeenMap::get_value(cur_x, cur_y+1) && SeenMap::get_value(cur_x-1, cur_y) <= SeenMap::get_value(cur_x, cur_y-1)) {
                            emit(Event("move", {cur_x-1, cur_y})); // Move forward
                        // Block on the right has the lowest visited times compare to block in front and block on the left
                        } else if(SeenMap::get_value(cur_x, cur_y-1) <= SeenMap::get_value(cur_x-1, cur_y) && SeenMap::get_value(cur_x, cur_y-1) <= SeenMap::get_value(cur_x, cur_y+1)) {
                            emit(Event("rotate", 1)); // Turn right
                        } else {
                            emit(Event("rotate", -1)); // Turn left
                        }
                    // No obstacles in front and on the right but on the left
                    } else if (sensor_value(0) > 100 && sensor_value(1) > 100){
                        // Block in front has lower visited times compare to the block on the right
                        if(SeenMap::get_value(cur_x-1, cur_y) <= SeenMap::get_value(cur_x, cur_y-1)){
                            emit(Event("move", {cur_x-1, cur_y})); // Move forward
                        }else{
                            emit(Event("rotate", 1)); // Turn right
                        }
                    // No obstacles in front and on the left but on the right
                    } else if (sensor_value(0) > 100 && sensor_value(2) > 100){
                        // Block in front has lower visited times compare to the block on the left
                        if(SeenMap::get_value(cur_x-1, cur_y) <= SeenMap::get_value(cur_x, cur_y+1)){
                            emit(Event("move", {cur_x-1, cur_y})); // Move forward
                        }else{
                            emit(Event("rotate", -1)); // Turn left
                        }
                    // No obstacles on the left and on the right but in front
                    } else if (sensor_value(1) > 100 && sensor_value(2) > 100) {
                        // Block on the right has lower visited times compare to the block on the left
                        if(SeenMap::get_value(cur_x, cur_y-1) <= SeenMap::get_value(cur_x, cur_y+1)){
                            emit(Event("rotate", 1)); // Turn right
                        } else {
                            emit(Event("rotate", -1)); // Turn left
                        }
                    } else if (sensor_value(0) > 100){
                        emit(Event("move", {cur_x-1, cur_y}));
                    } else if (sensor_value(1) > 100) {
                        emit(Event("rotate", 1));
                    } else if (sensor_value(2) > 100) {
                        emit(Event("rotate", -1));
                    } else {
                        emit(Event("rotate", 1));
                    }
                } else { // Heading up
                    // No obstacles in all three directions, front, right, left
                    if(sensor_value(0) > 100 && sensor_value(1) > 100 && sensor_value(2) > 100){
                        // Block in front has the lowest visited times compare to block on the right and block on the left
                        if(SeenMap::get_value(cur_x, cur_y-1) <= SeenMap::get_value(cur_x+1, cur_y) && SeenMap::get_value(cur_x, cur_y-1) <= SeenMap::get_value(cur_x-1, cur_y)) {
                            emit(Event("move", {cur_x, cur_y-1})); // Move forward
                        // Block on the right has the lowest visited times compare to block in front and block on the left
                        } else if(SeenMap::get_value(cur_x+1, cur_y) <= SeenMap::get_value(cur_x, cur_y-1) && SeenMap::get_value(cur_x+1, cur_y) <= SeenMap::get_value(cur_x-1, cur_y)) {
                            emit(Event("rotate", 1)); // Turn right
                        } else {
                            emit(Event("rotate", -1)); // Turn left
                        }
                    // No obstacles in front and on the right but on the left
                    } else if (sensor_value(0) > 100 && sensor_value(1) > 100){
                        // Block in front has lower visited times compare to the block on the right
                        if(SeenMap::get_value(cur_x, cur_y-1) <= SeenMap::get_value(cur_x+1, cur_y)){
                            emit(Event("move", {cur_x, cur_y-1})); // Move forward
                        }else{
                            emit(Event("rotate", 1)); // Turn right
                        }
                    // No obstacles in front and on the left but on the right
                    } else if (sensor_value(0) > 100 && sensor_value(2) > 100){
                        // Block in front has lower visited times compare to the block on the left
                        if(SeenMap::get_value(cur_x, cur_y-1) <= SeenMap::get_value(cur_x-1, cur_y)){
                            emit(Event("move", {cur_x, cur_y-1})); // Move forward
                        }else{
                            emit(Event("rotate", -1)); // Turn left
                        }
                    // No obstacles on the left and on the right but in front
                    } else if (sensor_value(1) > 100 && sensor_value(2) > 100) {
                        // Block on the right has lower visited times compare to the block on the left
                        if(SeenMap::get_value(cur_x+1, cur_y) <= SeenMap::get_value(cur_x-1, cur_y)){
                            emit(Event("rotate", 1)); // Turn right
                        } else {
                            emit(Event("rotate", -1)); // Turn left
                        }
                    } else if (sensor_value(0) > 100){
                        emit(Event("move", {cur_x, cur_y-1}));
                    } else if (sensor_value(1) > 100) {
                        emit(Event("rotate", 1));
                    } else if (sensor_value(2) > 100) {
                        emit(Event("rotate", -1));
                    } else {
                        emit(Event("rotate", 1));
                    }
                }
            }
            
        }
        void exit(const Event &e) {}
        double rate;
        void set_tick_name(std::string s) { tick_name = s; }
        std::string tick_name;
        cpVect pos;
    };

    //! Restart state, teleport the robot to the start position (x: -300, y:-300, theta: 0)
    class Restart : public State, public AgentInterface
    {
    public:
        //! A method that derived instances should define. It is called when the state is
        //! entered by the state machine either when the machine starts or when a transition
        //! to the state is fired.
        //! \param e The event that led to the transition into the state, the value of the event is 1 or 0
        //! event value is 1 means player is controlling the robot, otherwise the robot is wardering on its own.
        void entry(const Event &e) {
            playing = e.value();
        }
        void during()
        {
            teleport(-300, -300, 0); // Teleport the robot to start position
            SeenMap::reset_map(); // clear the record
            if (playing == 1) { // Player controlling
                emit(Event("play", {0, 0})); // Transit to Inplay state
            } else {
                emit(Event("check"));
            }
        }
        void exit(const Event &e) {
        }
        void set_tick_name(std::string s) { tick_name = s; }
        std::string tick_name;
        int playing;
    };

    //! Inplay state, player is controlling the robot
    class Inplay : public State, public AgentInterface
    {
    public:
        //! A method that derived instances should define. It is called when the state is
        //! entered by the state machine either when the machine starts or when a transition
        //! to the state is fired.
        //! \param e The event that led to the transition into the state, the value of the event 
        //! is of form {velocity, angular velocity}
        void entry(const Event &e) {
            v = e.value()[0].get<double>(); // Get the velocity
            av = e.value()[1].get<double>(); // Get the angular velocity
        }
        void during()
        {
            // Restart the game when robot gets to the exit of the maze.
            if (position().x >= 350 || position().y >= 350) {
                emit(Event("restart", 1));
            }
            track_velocity(v, av);
        }
        void exit(const Event &e) {
        }
        void set_tick_name(std::string s) { tick_name = s; }
        std::string tick_name;
        double v;
        double av;
    };

    //! GoToClosestCenter state, makes to robot go to the center of the current block
    class GoToClosestCenter : public State, public AgentInterface
    {
    public:
        void entry(const Event &e) {
            cur_x = position().x;
            cur_y = position().y;
            // Calculate which block the robot is current in.
            for(int i = 0; i < 7; i++){
                if (cur_x >= -350 + i*100 && cur_x < -350 + (i+1)*100) {
                    grid_x = 50.0+i*100.0-350.0;
                }
                if (cur_y >= -350 + i*100 && cur_y < -350 + (i+1)*100) {
                    grid_y = 50.0+i*100.0-350.0;
                }
            }
        }
        void during()
        {
            cur_x = position().x;
            cur_y = position().y;
            move_toward(grid_x, grid_y, 20, 10); // Move to the center of the current block
            if (std::fabs(cur_x - grid_x) < 1 && std::fabs(cur_y - grid_y) < 1){
                track_velocity(0, 0);
                teleport(grid_x, grid_y, angle()); // Teleport to the center when the robot is getting close
                emit(Event("rotate", 0));
            }
        }
        void exit(const Event &e) {
        }
        void set_tick_name(std::string s) { tick_name = s; }
        std::string tick_name;
        double cur_x;
        double cur_y;
        int grid_x;
        int grid_y;
        double goal_angle;
    };


    //! WandererController class, a state machine that is used to control the robot's states
    class WandererController : public StateMachine, public AgentInterface
    {

    public:
        WandererController() : StateMachine()
        {
            // Add all the transitions here
            set_initial(checking); // init with checking state
            tick_name = "tick_" + std::to_string(rand() % 1000); // use an agent specific generated
                                                                 // event name in case there are
                                                                 // multiple instances of this class
            add_transition(tick_name, moving, rotating);
            add_transition(tick_name, rotating, moving);
            add_transition("move", checking, moving);
            add_transition("check", moving, checking);
            add_transition("rotate", checking, rotating);
            add_transition("check", rotating, checking);
            add_transition("restart", moving,restarting);
            add_transition("restart", rotating, restarting);
            add_transition("restart", checking, restarting);
            add_transition("restart", restarting, restarting);
            add_transition("restart", playing, restarting);
            add_transition("check", restarting, checking);
            add_transition("rotate", gtcc, rotating);
            add_transition("play", restarting, playing);
            add_transition("play", moving, playing);
            add_transition("play", rotating, playing);
            add_transition("play", checking, playing);
            add_transition("play", playing, playing);
            add_transition("play", gtcc, playing);
            add_transition("gtcc", playing, gtcc);
            moving.set_tick_name(tick_name);
            rotating.set_tick_name(tick_name);
            checking.set_tick_name(tick_name);
        }
        
        Moving moving;
        Rotating rotating;
        Checking checking;
        Restart restarting;
        Inplay playing;
        GoToClosestCenter gtcc;
        std::string tick_name;

    };

    //! UserInputController class, listens to all the button click events and key press events
    class UserInputController : public Process, public AgentInterface {

        public:
        UserInputController() : Process(), AgentInterface() {}

        void init() {
            // Adds a label showing robot is being controlled by the player or self wandering
            label("wandering", -27, -20);
            playing = 0;
            watch("button_click", [&](Event& e) {
                if ( e.value()["value"] == "restart" ) {
                    emit(Event("restart", playing));
                }
                if ( e.value()["value"] == "player" ) { // Player in control
                    if (playing == 0){
                        playing = 1; 
                        clear_label();
                        label("user playing", -33, -20);
                        emit(Event("play", {0, 0}));
                    }
                }
                if ( e.value()["value"] == "selfWander" ) { // Self wandering
                    if (playing == 1){
                        playing = 0;
                        clear_label();
                        label("wandering", -27, -20);
                        emit(Event("gtcc")); // Go to the closest center first then starts wandering
                    }
                }
            });
            watch("keydown", [&](Event &e) {
                // Use A, W, S, D keys to control the robot
                if(position().x < 350 && position().y < 350){ // within the maze boundry
                    if (playing == 1) {
                        auto k = e.value()["key"].get<std::string>();
                        if ( k == "w" ) {
                            v = v_m;
                        } else if ( k == "s" ) {
                            v = -v_m;
                        } else if ( k == "a" ) {
                            omega = -omega_m;
                        } else if ( k == "d" ) {
                            omega = omega_m;
                        }
                        emit(Event("play", {v, omega}));
                    }
                }
            });
            watch("keyup", [&](Event &e) {
                if(position().x < 350 && position().y < 350){ // within the maze boundry 
                    if (playing == 1) {
                        auto k = e.value()["key"].get<std::string>();
                        if ( k == "w" || k == "s" ) {
                            v = 0;               
                        } else if ( k == "a" ) {
                            omega = 0;
                        } else if ( k == "d" ) {
                            omega = 0;
                        } 
                        emit(Event("play", {v, omega}));
                    }
                }
            });
        }

        void start() {}
        void update() {}
        void stop() {}

        double z = 1.0;
        int playing = 0;
        double v, omega;
        double const v_m = 5, omega_m = 0.5;

    };

    class Wanderer : public Agent
    {

    public:
        Wanderer(json spec, World &world) : Agent(spec, world)
        {
            // Adds two processes
            add_process(wc); 
            add_process(wpc);
        }

        WandererController wc;
        UserInputController wpc;
    };

    DECLARE_INTERFACE(Wanderer);

}

#endif