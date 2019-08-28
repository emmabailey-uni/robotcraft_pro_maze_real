#include <iostream>
#include <cstdlib>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <rosserial_arduino/Test.h>
#include <string>
#include <list>
#include <tuple>

#include <vector>
#include <algorithm>
#include <utility>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"

#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884L
#endif na



class PathFollowingController
{
private:

    //Making sort functions
    void new_sort(std::vector<std::tuple<int,int, int>> &path_finder, std::vector<std::tuple<int,int,int>> &list_neighbours, bool sort_flag){
        std::vector<std::tuple<int,int, int>> check;
        int i, j=0, k;
        int array_d[length(path_finder)];

        for (i=0; i<length(path_finder); i++){
          array_d[i] = std::get<2>(path_finder[i]);
        }

        for(i=0; i<4; i++){
          if (max_element(array_d[i].begin(), array_d[i].end()) == (std::get<2>(list_neighbours[i]))++){
            check[j] = (list_neighbours[i]);
            j++;
          }
        }

        int x_co, y_co, x_co_prev, y_co_prev;
        x_co = std::get<0>(path_finder.end());
        y_co = std::get<1>(path_finder.end());
        x_co_prev = std::get<0>(path_finder.end()[-2]);
        y_co_prev = std::get<1>(path_finder.end()[-2]);

        for (k=0; k<j; k++){

          if(x_co == x_co_prev && x_co == std::get<0>(check[k]) && y_co == y_co_prev +1 ){
                  list_neighbours[0] = check[k];
                  break;
          }

          if(x_co == x_co_prev && x_co == std::get<0>(check[k]) && y_co == y_co_prev -1 ){
                  list_neighbours[0] = check[k];
                  break;
          }

          if(y_co == y_co_prev && y_co == std::get<0>(check[k]) && x_co == x_co_prev -1 ){
                  list_neighbours[0] = check[k];
                  break;
          }

          if(y_co == y_co_prev && y_co == std::get<0>(check[k]) && x_co == x_co_prev +1 ){
                  list_neighbours[0] = check[k];
                  break;
          }

          else{
            sort_flag = true;
          }
        }
    }

    ros::NodeHandle n;

    //Publisher topics
    ros::Publisher cmd_vel_pub;

    //Subscriber topics
    ros::Subscriber map_sub;
    ros::Subscriber odom_sub;

    //Variables to discretize map into pixels
    int Width;
    int Height;
    float cell_size;

    //End position of robot - set from launch
    int endX, endY;

    //Start position of robot - set from launch
    int startX, startY, startD;

    float x, y, theta;

    bool robot_stopped;

    // Desired distance to the right wall - set from launch
    float desired_linear_velocity;
    float desired_side_wall_distance;
    float front_obstacle_distance_threshold;
    float desired_wall;

    // Wall distance controller parameters
    float v;
    float w;
    float K_omega; // Linear error
    float K_psi;    // Angular error
    float p;         // Recovery speed during line following
    float theta_ref;

    float x_offset, y_offset;

    //enum is a data type in c++ that contains a set of statis constants - not reinitialised on each run (they remember)
    //Essentially used here as a fancy switch_flag
    enum CONTROLLER_STATE
    {
        //Rotate means robot will be turning, follow line means it goes straight
        ROTATE = 1,
        FOLLOW_LINE = 2,
    };

    //Combines two values in a list - inextricably tied to each other in each list position, variable name is final_path
    //Final optimal path
     std::vector<std::tuple<int,int, int>> final_path;

     //Path we're currently running on to determine optimal
     std::tuple<int,int> my_p;


    bool init_flag = false;
    bool path_set = false;
    bool path_done = false;
    bool pos_set = false;


    geometry_msgs::Twist calculateCommand()
    {
        auto square_vel_msg = geometry_msgs::Twist();

        //Variable to store original position
        static float x_start , y_start, x_goal, y_goal;
        static bool init_flag = false;
        static bool done_flag = true;
        static int i = 0;
        static CONTROLLER_STATE ControllerState;

        if(!init_flag){
          //To keep track of where we are in map
          x_start = 0;
          y_start = 0;
          //Init flag changed - won't reinitilaise to start every time
          init_flag = true;
          ControllerState = ROTATE;
          my_p = final_path.front();
          final_path.erase(final_path.begin());
          //std::cout << "FIRST GOAL" << (final_path.front()).first << "\n";
        }

        //Will enter here as done_flag set to true at start of this function
        if(done_flag){
            //This conditon only entered when robot has arrived at the exit as we continually pop the queue
            if(final_path.empty()){
                //std::cout << "List is empty\n" ;
            }else{
                //This part determines the instructions to send to driver
                my_p = final_path.front();
                final_path.erase(final_path.begin());
                x_goal = Width*cell_size-(Width-(std::get<1>(my_p)))*cell_size+cell_size/2;
                y_goal = Height*cell_size-(std::get<0>(my_p))*cell_size-cell_size/2;
            }
            done_flag = false;
        }

        //std::cout<<"Theta current: "<<theta<<"\n";

        theta_ref = atan2((y_goal-y),(x_goal-x));
        //std::cout << "x_goal " << x_goal << " y_goal"  << y_goal <<"\n";
        //std::cout << "x_curr " << x << " y_curr"  << y <<"\n";
        //std::cout << "Theta_ref " << theta_ref << " Theta curr"  << theta <<"\n";


        switch (ControllerState) {
          //Rotate case - need to spin
          case 1 :
            //std::cout << "ROTATE" << "\n";
            //std::cout << "Diff" << (theta - theta_ref) <<"\n";
            //Angle not reduced enough - need to continue turning
            if (theta - theta_ref > 0.05 || theta - theta_ref < -0.05 ){
                v = K_omega * (cos(theta)*(x_start-x) + sin(theta)*(y_start-y));
                w = K_psi*(theta_ref-theta);
            } else{
              //Turn complete - want to stop and enter new case on next run
                w = 0;
                v = 0;
                ControllerState = FOLLOW_LINE;
                //Update position in map - need to always know what pixel we are in
                x_start = x;
                y_start = y;
            }
            break;

          //Want to move in a straight line to next positon
          case 2 :
            //std::cout << "FOLLOW LINE" << "\n";
            if(cos(theta)*(x_goal-x) + sin(theta)*(y_goal-y) > 0.05 || cos(theta)*(x_goal-x) + sin(theta)*(y_goal-y) < -0.05 ){
                w = K_psi*(sin(theta_ref)*(x + p*cos(theta)-x_start) - cos(theta_ref)*(y + p*sin(theta)- y_start));
                v = K_omega * (cos(theta)*(x_goal-x) + sin(theta)*(y_goal-y));
            } else{
                w = 0;
                v = 0;
                //Need to reset done_flag in order to continue sending info
                done_flag = true;
                //Update positon in map
                x_start = x;
                y_start = y;
                ControllerState = ROTATE;
            }
            break;
        }

        //Send the desired velocities to the robot driver based on current position and motion needs
        square_vel_msg.linear.x = v;
        square_vel_msg.angular.z = w;
        return square_vel_msg;
    }

    //Gets the map from the basic maze, discretizes it, all info in grids. Relates all pixels with distance value to compute cost function.
    //Wave Propagation algorithm - check gc video
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        std_msgs::Header header = msg->header;
        nav_msgs::MapMetaData info = msg->info;
        ROS_INFO("Got map %d %d", info.width, info.height);

        //Cell size determined by our desired resolution
        cell_size = info.resolution;
        //Know width and height of map from message - assigned to variable
        Width = info.width, Height = info.height;
        int cols = info.width, rows = info.height;
        auto p = [&](int x, int y) { return x * cols + y;  };

        //Need matrices with correct numbers of rows and columns to discretise entire map - all 2D mappings - remember pointers
        //Binary Map - keeps track of whether postion is an obstacle or not
        int** M = new int*[rows];
        for (int i = 0; i < rows; ++i){
            M[i] = new int[cols];
        }

        //Potential field - keeps track of each potential path through maze
        int** C = new int*[rows];
        for (int i = 0; i < rows; ++i){
            C[i] = new int[cols];
        }

        //Shortest path - will store the optimal path determined
        int** P = new int*[rows];
        for (int i = 0; i < rows; ++i){
            P[i] = new int[cols];
        }

        std::cout << "\n\n BINARY MAP \n\n";
        //Cycle through each row and column
        for (int i = (rows-1); i >= 0; --i) {   // for each row
            for (int j = 0; j < cols; ++j) { // for each column
                //If theres something in the cell (NOT empty space) - set value to 1
                if(msg->data[i*cols + j] > 0){
                    M[i][j] = 1;
                }
                //If its empty space set a zero
                else{
                    M[i][j] = 0;
                }
                std::cout << M[i][j] << " ";
            }
            std::cout << "\n";
        }

        //Changing obstacles to be -1 to make wave propagation easier - limits of map hard set to 0 too
        for(int x = (rows-1); x >= 0; --x){
            for(int y = 0; y < cols; y++){
                if(x == 0 || y == 0 || x == (cols -1) || y == (rows -1) || M[x][y] == 1){
                    C[rows-1-x][y] = -1;
                }else{
                    C[rows-1-x][y] = 0;
                }
            }
        }



        //New paired listing of three values for x, y and distance
        std::vector<std::tuple<int,int,int>> nodes;

        //Exit of the map pushed to the list first
        nodes.push_back({endX,endY,1});

        while(!nodes.empty()){
            //New paired listing of three values for x, y and distance
            std::vector<std::tuple<int,int,int>> new_nodes;

            for(auto &n : nodes){
                //This is the method to access the variable held in our tuple list
                int x = std::get<0>(n);
                int y = std::get<1>(n);
                int d = std::get<2>(n);

                C[x][y] = d;

                //Determining cost map - moving through maps possible paths and adding one to distance with each propgation move
                // Check south
                if((x+1) < rows && C[x+1][y] == 0){
                    new_nodes.push_back({x+1,y,d+1});
                }
                // Check north
                if((x-1) >= 0 && C[x-1][y] == 0){
                    new_nodes.push_back({x-1,y,d+1});
                }
                // Check east
                if((y+1) < cols && C[x][y+1] == 0){
                    new_nodes.push_back({x,y+1,d+1});
                }
                // Check west
                if((y-1) >= 0 && C[x][y-1] == 0){
                    new_nodes.push_back({x,y-1,d+1});
                }
                /*
                // Check south east
                if((x+1) < rows && (y+1) < cols && C[x+1][y+1] == 0){
                    new_nodes.push_back({x+1,y+1,d+1});
                }
                // Check north east
                if((x-1) >= 0 && (y+1) < cols && C[x-1][y+1] == 0){
                    new_nodes.push_back({x-1,y+1,d+1});
                }
                // Check south west
                if((x+1) < rows && (y-1) >= 0 && C[x+1][y-1] == 0){
                    new_nodes.push_back({x+1,y-1,d+1});
                }
                // Check north west
                if((x-1) >= 0 && (y-1) >= 0  && C[x-1][y-1] == 0){
                    new_nodes.push_back({x-1,y-1,d+1});
                }

                */
            }

            // Sort the nodes - This will stack up nodes that are similar: A, B, B, B, B, C, D, D, E, F, F
            new_nodes.sort([&](const std::tuple<int, int, int> &n1, const std::tuple<int, int, int> &n2)
            {
                // In this instance I dont care how the values are sorted, so long as nodes that
                // represent the same location are adjacent in the list. I can use the p() lambda
                // to generate a unique 1D value for a 2D coordinate, so I'll sort by that.
                return p(std::get<0>(n1), std::get<1>(n1)) < p(std::get<0>(n2), std::get<1>(n2));
            });

            // Use "unique" function to remove adjacent duplicates       : A, B, -, -, -, C, D, -, E, F -
            // and also erase them                                       : A, B, C, D, E, F
            new_nodes.unique([&](const std::tuple<int, int, int> &n1, const std::tuple<int, int, int> &n2)
            {
                return  p(std::get<0>(n1), std::get<1>(n1)) == p(std::get<0>(n2), std::get<1>(n2));
            });

            // We've now processed all the discoverd nodes, so clear the list, and add the newly
            // discovered nodes for processing on the next iteration
            nodes.clear();
            nodes.insert(nodes.begin(), new_nodes.begin(), new_nodes.end());
        }

        /*
        std::cout << "\n\n VISUALIZABLE BINARY \n\n";

        //Prints out the binary map
        for(int x = 0; x < rows; x++){
            for(int y = 0; y < cols; y++){
                //std::cout << C[x][y] << " ";
                if(C[x][y] == -1){
                    std::cout << "X ";
                }
                if(C[x][y] >= 0){
                    std::cout << "- ";
                }
            }
            std::cout << "\n";
        }
        */

        //Algorithm to move through the cost map - choosing optimal path
        //Pushing new x-y coord each time you move
        std::vector<std::tuple<int,int, int>> path;
        path.push_back({startX,startY, startD});
        //locx anf locy keep track of current location
        int locX = startX;
        int locY = startY;
        int locD = startD;
        bool no_path = false;

        while(!(locX == endX && locY == endY) && !no_path){
            std::vector<std::tuple<int,int,int>> listNeighbours;

            //This is the bit we want to optimise - we think
            // Check south
            if((locX+1) < rows && C[locX+1][locY] > 0){
                listNeighbours.push_back({locX+1,locY,C[locX+1][locY]});
            }
            // Check north
            if((locX-1) >= 0 && C[locX-1][locY] > 0){
                listNeighbours.push_back({locX-1,locY,C[locX-1][locY]});
            }
            // Check east
            if((locY+1) < cols && C[locX][locY+1] > 0){
                listNeighbours.push_back({locX,locY+1,C[locX][locY+1]});
            }
            // Check west
            if((locY-1) >= 0 && C[locX][locY-1] > 0){
                listNeighbours.push_back({locX,locY-1,C[locX][locY-1]});
            }
            /*
            // Check south east
            if((locX+1) < rows && (locY+1) < cols && C[locX+1][locY+1] > 0){
                listNeighbours.push_back({locX+1,locY+1,C[locX+1][locY+1]});
            }
            // Check north east
            if((locX-1) >= 0 && (locY+1) < cols && C[locX-1][locY+1] > 0){
                listNeighbours.push_back({locX-1,locY+1,C[locX-1][locY+1]});
            }
            // Check south west
            if((locX+1) < rows && (locY-1) >= 0 && C[locX+1][locY-1] > 0){
                listNeighbours.push_back({locX+1,locY-1,C[locX+1][locY-1]});
            }
            // Check north west
            if((locX-1) >= 0 && (locY-1) >= 0  && C[locX-1][locY-1] > 0){
                listNeighbours.push_back({locX-1,locY-1,C[locX-1][locY-1]});
            }
			*/
            bool sort_flag = true;

            if(sort_flag){
              new_sort(&path, &listNeighbours, sort_flag);
            }

            else{
              listNeighbours.sort([&](const std::tuple<int, int, int> &n1, const std::tuple<int, int, int> &n2)
              {
                  return std::get<2>(n1) < std::get<2>(n2); // Compare distances
              });
              sort_flag = false;

            }


            if (listNeighbours.empty()) // Neighbour is invalid or no possible path
                no_path = true;
            else
            {
                locX = std::get<0>(listNeighbours.front());
                locY = std::get<1>(listNeighbours.front());
                locD = std::get<2>(listNeighbours.front());
                path.push_back({ locX, locY, locD });
            }

        }
        P = C;
        final_path = path;
        int p_x, p_y;
        for (auto &a : path){
            p_x = std::get<0>(a);
            p_y = std::get<1>(a);
            std::cout << "X" << p_x << " Y " << p_y << "\n";
            P[p_x][p_y] = 0;
        }


        //Prints out map with shortest path
        std::cout << "\n\n SHORTEST PATH \n\n";

        for(int x = 0; x < rows; x++){
            for(int y = 0; y < cols; y++){
                if(P[x][y] == -1){
                    std::cout << "X ";
                }
                if(P[x][y] == 0){
                    std::cout << "O ";
                }
                if(P[x][y] > 0){
                    std::cout << "- ";
                }
            }
            std::cout << "\n";
        }

        final_path = path;
        path_set = true;

        /*

        nav_msgs::OccupancyGrid* newGrid = map.Grid();
        newGrid->header = header;
        newGrid->info = info;
        map_pub.publish(*newGrid);
        */

        //Clearing the memory
        for (int i = 0; i < rows; ++i){
            delete [] M[i];
        }
        delete [] M;

        for (int i = 0; i < rows; ++i){
            delete [] C[i];
        }
        delete [] C;


    }


    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        x = (msg->pose.pose.position.x)+x_offset; //0.2
        y = (msg->pose.pose.position.y)+y_offset; //5.175;
        theta = yaw;
        //std::cout << "YAW" << yaw << "\n";
        pos_set = true;
    }





public:
    PathFollowingController(){
        // Initialize ROS
        this->n = ros::NodeHandle();

        // Create a publisher object, able to push messages
        this->cmd_vel_pub = this->n.advertise<geometry_msgs::Twist>("reactive_vel", 10);

        // name_of_the_subscriber = n.subscribe("topic_name")
        this->odom_sub = n.subscribe("odom", 10, &PathFollowingController::odomCallback, this);
        this->map_sub = n.subscribe("map",10,&PathFollowingController::mapCallback,this);

        this->n.getParam("/startX", startX);
        this->n.getParam("/startY", startY);
        this->n.getParam("/startD", startD);
        this->n.getParam("/endX", endX);
        this->n.getParam("/endY", endY);
        this->n.getParam("/K_psi", K_psi);
        this->n.getParam("/K_omega", K_omega);
        this->n.getParam("/p", p);
        this->n.getParam("/x_offset", x_offset);
        this->n.getParam("/y_offset", y_offset);

    }

    void run(){

        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            if(path_set && pos_set){
            auto msg = calculateCommand();

            // Publish the new command
            this->cmd_vel_pub.publish(msg);
            }

            ros::spinOnce();

            // And throttle the loop
            loop_rate.sleep();
        }
    }

};


int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "path_following_controller");


    // Create our controller object and run it
    auto controller = PathFollowingController();
    controller.run();
}
