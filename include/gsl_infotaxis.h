#include <gsl_algorithm.h>
#include <fstream>      
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <math.h>
#include <bits/stdc++.h>
#include <unordered_set>
#include <gmrf_wind_mapping/WindEstimation.h>
#include <visual_cpt.h>
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
enum class Infotaxis_state {WAITING_FOR_MAP, STOP_AND_MEASURE, MOVING};

class Cell {
    public:
        Cell(bool free, double x, double y, double weight);
        ~Cell(){};
        bool free;
        double x, y, weight, auxWeight, distance;        
};

struct WindVector {
    int i, j;
    double speed, angle;
};

class InfotaxisGSL:public GSLAlgorithm, public VisualCPT {
    public:
        InfotaxisGSL(ros::NodeHandle *nh);
        ~InfotaxisGSL();
        void getGasWindObservations();
        Infotaxis_state getState();
        void setGoal();

    protected:
        ros::Subscriber gas_sub_;                                           //! Gas readings subscriber
        ros::Subscriber wind_sub_;                                          //! Wind readings subscriber
        ros::Subscriber map_sub_;                                           //! Map subscriber.
        void gasCallback(const std_msgs::String::ConstPtr& msg); //override;
        void windCallback(const olfaction_msgs::anemometerPtr& msg);
        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result) override;
         
        Infotaxis_state previous_state, current_state;

        //Measurements
        ros::Time time_stopped; 
        ros::Time last_revisited;
        double stop_and_measure_time;                                       //! (seconds) time the robot is stopped while measuring the wind direction
        bool gasHit;
        double th_gas_present;
        double th_wind_present;

        std::vector<float> gas_vector;
        std::vector<float> wind_spd_vector;
        std::vector<float> wind_dir_vector;
        std::vector<float> entropy_gain_his;
        std::vector<float> entropy_gain_rate;

        double avg_concentration, avg_wind_dir, avg_wind_spd;
        float get_avg_wind_dir(std::vector<float> const &v);

        //Estimations
        double stdev_hit;
        double stdev_miss;
        void estimateProbabilities(std::vector<std::vector<Cell> >& map, bool hit, double wind_direction, Eigen::Vector2i robot_pos);
        void propagateProbabilities(std::vector<std::vector<Cell> >& map,
                                    std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int> > >& openSet,
                                    std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int> > >& closedSet,
                                    std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int> > >& activeSet);

        void calculateWeight(std::vector<std::vector<Cell> >& map,
                                int i, int j, std::pair<int,int> p,
                                std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int> > >& openSet,
                                std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int> > >& closedSet,
                                std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int> > >& activeSet);
        void normalizeWeights(std::vector<std::vector<Cell> >& map);

        
        //Movement
        bool infoTaxis;
        void moveTo(int i, int j);
        void cancel_navigation(); 
        
        Eigen::Vector2d previous_robot_pose;
        std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int>> > openMoveSet;
        std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int>> > visitedSet;
        
        void updateSets();

        //Infotaxis
        double entropy(int i, int j, Eigen::Vector2d wind);
        ros::ServiceClient clientW;
        std::vector<WindVector> estimateWind();

        //Cells
        double scale;
        int numCells;
        int planning_mode; //0 is infotaxis, 1 is dijstrak

        int number_revisited;

        std::vector<std::vector<Cell> > cells;
        ros::Publisher probability_markers;
        ros::Publisher entropy_reporter;
        Eigen::Vector2i currentPosIndex;
        
        //Auxiliary functions
        // visualization_msgs::Marker emptyMarker();
        void showWeights();
        Eigen::Vector2i coordinatesToIndex(double x, double y);
        Eigen::Vector2d indexToCoordinates(double i, double j);
        double gaussian(double distance, double sigma);
        
        //Termination condition
        double t1;
        double convergence_thr;
        double ground_truth_x, ground_truth_y;
};