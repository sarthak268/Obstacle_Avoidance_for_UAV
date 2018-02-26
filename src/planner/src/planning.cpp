#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <vector>    
#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
   
using namespace ob = ompl::base;
using namespace og = ompl::geometric;
using namespace std;

vector<double> obstacles(3);
vector<double> waypoints(2);

void readObstacleFile()
{
   std::ifstream infile("/home/sarthak/Desktop/Aurora/obstacles.txt");
   double x, y, r;

   while(ifstream >> x >> y >> r)
   {
      obstacles[0].push_back(x);
      obstacles[1].push_back(y);
      obstacles[2].push_back(r);
   }
}

void readWaypointsFile()
{
   std::ifstream infile("/home/sarthak/Desktop/Aurora/waypoints.txt");
   double x, y;

   while(ifstream >> x >> y)
   {
      waypoints[0].push_back(x);
      waypoints[1].push_back(y);
   }
}

bool checkObstacleValidity(const ob::State* state) const
{
   const ob::RealVectorStateSpace::StateType* state2D = state->as<ob::RealVectorStateSpace::StateType>();
   double x = state2D->values[0];
   double y = state2D->values[1];

   bool safe = true;

   for (int i = 0; i < obstacles[0].size(); i++)
   {
      double obs_x = obstacles[0][i];
      double obs_y = obstacles[1][i];
      double obs_r = obstacles[2][i];

      double dis = pow((pow(obs_x - x, 2) + pow(obs_y - y, 2)), 0.5);
      if (dis < 1.1 * obs_r)
      {
         safe = false;
         // the state is valid only if it remains at a minimum distance of 1.1 * (radius of obstacle) from the (centre of obstacle) 
      }
   }
   return safe;
}
    
bool isStateValid(const ob::State *state)
{
   // cast the abstract state type to the type we expect
   const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
    
   // extract the first component of the state and cast it to what we expect
   const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
    
   // extract the second component of the state and cast it to what we expect
   const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);    
    
   // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
   if ((const void*)rot != (const void*)pos)
   {
      bool b = checkObstacleValidity();
      return b;
   }
   else 
   {
      return false
   }
}

void planWithSimpleSetup()
{
   // construct the state space we are planning in
   auto space(std::make_shared<ob::SE3StateSpace>());
   
   // set the bounds for the R^3 part of SE(3)
   ob::RealVectorBounds bounds(3);
   bounds.setLow(-100);
   bounds.setHigh(100);
   
   space->setBounds(bounds);
  
   // define a simple setup class
   og::SimpleSetup ss(space);

   // set state validity checking for this space
   ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });
   
   for (int wpno = 0; wpno < waypoints[0].size(); wpno ++)
   {
      if (wpno == 0)
      {
         ob::ScopedState<> start(space);
         start->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0;
         start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0;
      }
      else
      {
         double starting_x = waypoints[0][wpno - 1];
         double starting_y = waypoints[1][wpno - 1];
         ob::ScopedState<> start(space);
         start->as<ob::RealVectorStateSpace::StateType>()->values[0] = starting_x;
         start->as<ob::RealVectorStateSpace::StateType>()->values[1] = starting_y;  
      }

      double ending_x = waypoints[0][wpno];
      double ending_y = waypoints[1][wpno];
      ob::ScopedState<> goal(space);
      goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = ending_x;
      goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = ending_y;    

      // set the start and goal states
      ss.setStartAndGoalStates(start, goal);
      
      // this call is optional, but we put it in to get more output information
      ss.setup();
      ss.print();
      
      // attempt to solve the problem within one second of planning time
      ob::PlannerStatus solved = ss.solve(1.0);
      
      if (solved)
      {
         std::cout << "Found solution:" << std::endl;
         // print the path to screen         
         ss.simplifySolution();
         ss.getSolutionPath().print(std::cout);
      }
      else
      {
         std::cout << "No solution found" << std::endl;
      }
   }
   
}

int main(int argc, char** argv)
{
   cout << "OMPL version" << OMPL_VERSION << endl;
   planWithSimpleSetup();
}
































