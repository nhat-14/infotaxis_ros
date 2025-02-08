#include <gsl_infotaxis.h>
#include <boost/format.hpp>

Cell::Cell(bool f, double a, double b, double c) {
    x    = a;
    y    = b;
    free = f;
    weight    = c;
    auxWeight = 0;
    distance  = 0;
}

InfotaxisGSL::InfotaxisGSL(ros::NodeHandle *nh) : GSLAlgorithm(nh), VisualCPT(nh) {
    nh->param<double>("th_gas_present", th_gas_present, 0.3);
    nh->param<double>("th_wind_present", th_wind_present, 0.03);
    nh->param<double>("stop_and_measure_time", stop_and_measure_time, 5.0);
    nh->param<int>("cell_map_ratio", cell_map_ratio, 8);                      //scale for dynamic map reduction
    nh->param<double>("convergence_thr", convergence_thr, 0.5); //threshold for source declaration
    nh->param<double>("stdev_hit", stdev_hit, 1.0);             //standard deviation of hit and miss?
    nh->param<double>("stdev_miss", stdev_miss, 3.0);

    ROS_INFO("VLLLLLLLLLLLLLLLLLLLLLLL  %f",stop_and_measure_time);
    ROS_INFO("VLLLLLLLLLLLLLLLLLLLLLLL  %f",subenv_x[1]);
    
    // Subscribers & publisher 
    gas_sub_  = nh->subscribe(enose_topic,1,&InfotaxisGSL::gasCallback, this);
    wind_sub_ = nh->subscribe(anemometer_topic,1,&InfotaxisGSL::windCallback, this);
    map_sub_  = nh->subscribe(map_topic, 1, &InfotaxisGSL::mapCallback, this);

    probability_markers = nh->advertise<visualization_msgs::Marker>("probability_markers", 10);
    entropy_reporter    = nh->advertise<std_msgs::Float32>("entropy_reporter", 10);
    vel_pub             = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    // Init State
    gasHit           = false;
    number_revisited = 0;
    planning_mode    = "infotaxis";
    number_steps     = 0;
    previous_state   = Infotaxis_state::WAITING_FOR_MAP;
    current_state    = Infotaxis_state::WAITING_FOR_MAP; 
    moth_state       = 0;
    last_hit         = 0;
    tblank           = ros::Time::now();

    ROS_INFO("INITIALIZATON COMPLETED--> WAITING_FOR_MAP");
  	ros::NodeHandle n;
    clientW = n.serviceClient<gmrf_wind_mapping::WindEstimation>("/WindEstimation");            //de y!! tam thoi bo qua
}

InfotaxisGSL::~InfotaxisGSL() {}

//==================================== CALLBACKS =========================================
// Input the occupancy map once before doing CPT
// ROS convention is to consider cell [0,0] as the lower-left corner 
// see http://docs.ros.org/api/nav_msgs/html/msg/MapMetaData.html
void InfotaxisGSL::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map_ = *msg;
    float map_originX   = map_.info.origin.position.x;
    float map_originY   = map_.info.origin.position.y;
    float map_height     = map_.info.height;
    float map_width      = map_.info.width;
    float map_resolution = map_.info.resolution;

    ROS_INFO("Got the map of the environment!");
    ROS_INFO("--------------INFOTAXIS GSL---------------");
    ROS_INFO("Occupancy Map dimensions:"); //i is y(height), j is x(width)
    ROS_INFO("x_min:%.2f x_max:%.2f / y_min:%.2f y_max:%.2f",
        map_originX, map_originX + map_width*map_resolution, 
        map_originY, map_originY + map_height*map_resolution);
    ROS_INFO("------------------------------------------");

    std::vector<std::vector<int>> map_origin(map_height,std::vector<int>(map_width));

    //invert from 0 is free to 1 is free
    int index=0;
    for(int i=0; i<map_origin.size(); i++) {
        for(int j=0; j<map_origin[0].size(); j++) {
            map_origin[i][j] = (map_.data[index] == 0? 1 : 0);
            index++;
        }
    }
    
    // Initial a scaled down map with initial values
    int cells_height = ceil((float)map_height/cell_map_ratio);
    int cells_width = ceil((float)map_width/cell_map_ratio);
    cells.resize(cells_height);
    for(auto &cell :cells){
        cell.resize(cells_width, Cell(false,0,0,0));
    }

    numCells = 0; //number free cell in reduced map
    int cellsI=0, cellsJ=0;

    for(int i=0; i<map_origin.size(); i+=cell_map_ratio) {
        cellsJ=0;
        for(int j=0; j<map_origin[0].size(); j+=cell_map_ratio) {
            bool libre=true;                        //free cell?
            for(int row=i; row<map_origin.size() && row<i+cell_map_ratio; row++){
                for(int col=j;col<map_origin[0].size()&&col<j+cell_map_ratio;col++){
                    if(map_origin[row][col]!=1){    //if one small cell is not free, the whole big cell is not free
                        libre = false;
                    }
                }
            }
            Eigen::Vector2d coord = indexToCoordinates(cellsI,cellsJ);
            cells[cellsI][cellsJ].free   = libre;
            cells[cellsI][cellsJ].x      = coord.x();
            cells[cellsI][cellsJ].y      = coord.y();
            cells[cellsI][cellsJ].weight = libre?1:0; //free=1, occupied=0 unnormalize probabily of gas source
            cellsJ++; 
            if(libre) numCells++; // for normalize the initial propability map
        }
        cellsI++;
    }
    normalizeWeights(cells);
    showWeights();
    cancel_navigation();
    
    //Start the fun!!
    current_state = Infotaxis_state::STOP_AND_MEASURE;
    inExecution = true;
    start_time = ros::Time::now();  //start measuring time
    robot_poses_vector.clear();        //start measuring distance
    ROS_WARN("[Infotaxis] STARTING THE SEARCH");
}

void InfotaxisGSL::gasCallback(const std_msgs::String::ConstPtr& msg) {
    //Only save gas data if in the Stop_and_Measure
    std::string miss_gas = "nothing";
    if (current_state == Infotaxis_state::STOP_AND_MEASURE) {
        gasHit = (miss_gas.compare(msg->data.c_str()) == 0)? false : true;
        gas_vector.push_back((float)gasHit);
    }
    // updateState(!(miss_gas.compare(msg->data.c_str()) == 0));   
}

void InfotaxisGSL::windCallback(const olfaction_msgs::anemometerPtr& msg) {
    //1. Add obs to the vector of the last N wind speeds
    float downWind_direction = angles::normalize_angle(msg->wind_direction + M_PI); //simulation wind
    geometry_msgs::PoseStamped anemometer_downWind_pose, map_downWind_pose; //Transform from anemometer ref_system to map ref_system using TF
    try {
        anemometer_downWind_pose.header.frame_id = msg->header.frame_id;    
        anemometer_downWind_pose.pose.position.x = 0.0;
        anemometer_downWind_pose.pose.position.y = 0.0;
        anemometer_downWind_pose.pose.position.z = 0.0;
        anemometer_downWind_pose.pose.orientation = tf::createQuaternionMsgFromYaw(downWind_direction);
        tf_.transformPose("map", anemometer_downWind_pose, map_downWind_pose);
    }
    catch(tf::TransformException &ex) {
        ROS_ERROR("Error: %s", ex.what());
        return;
    }

    //Only if we are in the Stop_and_Measure
    if (this->current_state == Infotaxis_state::STOP_AND_MEASURE) {
        wind_spd_vector.push_back(msg->wind_speed);
        wind_dir_vector.push_back(tf::getYaw(map_downWind_pose.pose.orientation));
    }
}

void InfotaxisGSL::goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result) {
    if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Infotaxis - %s - Target achieved!", __FUNCTION__);
    }
    else
        ROS_INFO("Infotaxis - %s - UPS! Couldn't reach the target.", __FUNCTION__);
    cancel_navigation();
}

//========================================== ESTIMATE ===================================================
//Get averaged values of the observations taken while standing
//Wind direction is reported as DownWind in the map frame_id
//Being positive to the right, negative to the left, range [-pi,pi]
void InfotaxisGSL::getGasWindObservations() {
    if((ros::Time::now() - time_stopped).toSec() >= stop_and_measure_time ) {
        // avg_concentration  = get_average_vector(gas_vector);
        avg_concentration  = *max_element(gas_vector.begin(), gas_vector.end());     
        avg_wind_dir = get_avg_wind_dir(wind_dir_vector);
        avg_wind_spd = get_average_vector(wind_spd_vector);

        gas_vector.clear();
        wind_spd_vector.clear();
        previous_robot_pose=Eigen::Vector2d(current_pose.pose.pose.position.x, current_pose.pose.pose.position.y); 
        previous_state = current_state;

        ROS_ERROR("avg_gas=%.2f | avg_wind_spd=%.2f | avg_wind_dir=%.2f", avg_concentration, avg_wind_spd, avg_wind_dir);
        if (avg_concentration > th_gas_present && avg_wind_spd > th_wind_present) {
            //Gas & wind
            gasHit = true;
            ROS_WARN("GAS HIT!!!   New state --> MOVING");
            hit_notify(gasHit);
            // previous_robot_pose = Eigen::Vector2d(current_pose.pose.pose.position.x, current_pose.pose.pose.position.y); //register where you were before moving
            estimateProbabilities(cells, gasHit, avg_wind_dir, currentPosIndex);
            current_state = Infotaxis_state::MOVING;
            
        }
        else if (avg_concentration > th_gas_present) {
            gasHit=true;
            ROS_WARN("GAS, BUT NO WIND  New state --> MOVING");
            hit_notify(gasHit);
            // previous_robot_pose=Eigen::Vector2d(current_pose.pose.pose.position.x, current_pose.pose.pose.position.y); 
            previous_state=current_state;
            current_state = Infotaxis_state::MOVING;
            
        }
        else {
            gasHit=false;
            ROS_WARN("NOTHING!!!! New state --> MOVING");
            hit_notify(gasHit);
            estimateProbabilities(cells, gasHit, avg_wind_dir, currentPosIndex);
            previous_state=current_state;
            current_state = Infotaxis_state::MOVING;     
        }
    }
}

void InfotaxisGSL::estimateProbabilities(std::vector<std::vector<Cell> >& map, bool hit, double wind_direction, Eigen::Vector2i robot_pos) {
    std::unordered_set< std::pair<int, int>, boost::hash<std::pair<int, int> > > openPropagationSet;
    std::unordered_set< std::pair<int, int>, boost::hash<std::pair<int, int> > > activePropagationSet;
    std::unordered_set< std::pair<int, int>, boost::hash<std::pair<int, int> > > closedPropagationSet;
    int i=robot_pos.x();
    int j=robot_pos.y();
    int oI=std::max(0,i-1);
    int fI=std::min((int) map.size()-1,i+1);
    int oJ=std::max(0,j-1);
    int fJ=std::min((int) map[0].size()-1,j+1);
    
    //estimate the probabilities for the 8 neighbours
    Eigen::Vector2d coordR = indexToCoordinates(i,j);
    double upwind_dir      = angles::normalize_angle(wind_direction + M_PI);
    double move_dir        = atan2((coordR.y() - previous_robot_pose.y()),(coordR.x()-previous_robot_pose.x()));
    move_dir += M_PI;   //assign higher probability to opposite moving direction when miss (just like upwind dir)

    double maxHit          = gaussian(0,stdev_hit);
    double maxMiss         = gaussian(0,stdev_miss);

    for(int r=oI; r<=fI; r++) {
        for(int c=oJ; c<=fJ; c++) {
            if(map[r][c].free) {
                if(c!=j || r!=i) {
                    Eigen::Vector2d coordP = indexToCoordinates(r,c);
                    double dist;
                    double cell_vector = angles::normalize_angle(atan2((coordP.y()-coordR.y()),(coordP.x()-coordR.x())));

                    if(hit) {
                        dist=gaussian(atan2(sin(upwind_dir-cell_vector), cos(upwind_dir-cell_vector)),stdev_hit);
                    }
                    else {
                        // dist=gaussian(atan2(sin(move_dir-cell_vector), cos(move_dir-cell_vector)), stdev_miss);
                        dist=gaussian(atan2(sin(wind_direction-cell_vector), cos(wind_direction-cell_vector)), stdev_miss);
                    }
                    activePropagationSet.insert(std::pair<int,int>(r,c));
                    map[r][c].weight    *= dist; 
                    map[r][c].auxWeight = dist;
                    map[r][c].distance  = (r==i||c==j)?1:sqrt(2);
                }
            }
        }
    }
    
    map[i][j].weight = map[i][j].weight*gaussian((hit?0:M_PI), (hit?stdev_hit:stdev_miss));
    // map[i][j].weight = 0;
    map[i][j].weight *= 0.02;
    closedPropagationSet.insert(std::pair<int,int>(i,j));
    map[i][j].auxWeight=0;
    map[i][j].distance=0;

    //propagate these estimations to the entire environment
    propagateProbabilities(map, openPropagationSet, closedPropagationSet, activePropagationSet);  
}

void InfotaxisGSL::propagateProbabilities(std::vector<std::vector<Cell> >& map,
    std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int> > >& openPropagationSet,
    std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int> > >& closedPropagationSet,
    std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int> > >& activePropagationSet){
    while(!activePropagationSet.empty()) {
        while(!activePropagationSet.empty()){
            auto p = *activePropagationSet.begin();
            activePropagationSet.erase(activePropagationSet.begin());
            closedPropagationSet.insert(p);

            // this very ugly and confusing bit of code just does 4-neighbour propagation 
            int oR=std::max(0,p.first-1);
            int fR=std::min((int) map.size()-1,p.first+1);
            int oC=std::max(0,p.second-1);
            int fC=std::min((int) map[0].size()-1,p.second+1);
            
            //8-neighbour propagation
            for(int i=oR;i<=fR;i++){
                for(int j=oC;j<=fC;j++){
                    calculateWeight(map, i,j, p, openPropagationSet,closedPropagationSet,activePropagationSet);
                }
            }
        }
        
        for(auto& par : openPropagationSet){
            map[par.first][par.second].weight *= map[par.first][par.second].auxWeight;
        }
        activePropagationSet = openPropagationSet;
        openPropagationSet.clear();
    }
    normalizeWeights(map);
}


//=================================== NAVIGATION =============================================
void InfotaxisGSL::setGoal() {
    int i,j;
    showWeights();
    updateSets();
    std::vector<WindVector> wind = estimateWind();
    double ent    = -100000;
    double entAux = 0;

    if (planning_mode != "kbp") {
        Eigen::Vector2i target = get_jump_target();
        if (target.x() == -100 &&  target.y() == -100) {
            planning_mode = "infotaxis";
            if(!openMoveSet.empty()) {
                for (auto &p:wind) {
                    int r=p.i, c=p.j;
                    entAux = entropy(r,c, Eigen::Vector2d(p.speed,p.angle));
                    if(entAux > ent){
                        ent = entAux;
                        i=r; j=c;
                    }
                }
            }
            else {
                ROS_ERROR("Set of open nodes is empty!!!!");
            }
            entropy_gain_rate.push_back(ent);   
            if (entropy_gain_rate.size() > 5) {
                std::vector<float>::iterator it;
                it = entropy_gain_rate.begin();
                entropy_gain_rate.erase(it);
            }
            std_msgs::Float32 max_entropy_gain;
            max_entropy_gain.data = get_average_vector(entropy_gain_rate);    
            entropy_reporter.publish(max_entropy_gain);
            // openMoveSet.erase(std::pair<int,int>(i,j));
        }     
        else {
            planning_mode = "dijkstra";
            i=target.x();
            j=target.y();
        }
    }
    else { // (planning_mode == "kbp")
        cancel_navigation();
    }
    switch_notify(planning_mode);
    planning_mode = "infotaxis";
    number_steps += 1;
    ROS_ERROR("Step: %i", number_steps);
    moveTo(i,j);  
}


void InfotaxisGSL::cancel_navigation() {
    mb_ac.cancelAllGoals();           
    inMotion = false;

    //Start a new measurement-phase while standing
    gas_vector.clear();
    wind_spd_vector.clear();
    wind_dir_vector.clear();
    time_stopped = ros::Time::now();    //Start timer for initial wind measurement
    currentPosIndex = coordinatesToIndex(current_pose.pose.pose.position.x, current_pose.pose.pose.position.y);
    previous_state = current_state;
    current_state=Infotaxis_state::STOP_AND_MEASURE;
}

// Update visited or new cell
void InfotaxisGSL::updateSets() {
    int i = currentPosIndex.x();
    int j = currentPosIndex.y();

    // check wether pos is near the boundary
    int oI = std::max(0, i-1);
    int fI = std::min((int) cells.size()-1, i+1);
    int oJ = std::max(0, j-1);
    int fJ = std::min((int) cells[0].size()-1, j+1);

    openMoveSet.clear();

    for(int r=oI; r<=fI; r++){
        for(int c=oJ; c<=fJ; c++){
            if((r==i && c==j)||(r!=i && c!=j)){
                continue;   //Never stay in the same place or move diagonal
            }
            if(cells[r][c].free) {
                std::pair<int,int> p(r,c);
                openMoveSet.insert(p);
            }
        }
    }
}

double InfotaxisGSL::entropy(int i, int j, Eigen::Vector2d wind) { 
    //copy cells to estimate the probability ,aps without changing it
    auto cells2 = cells; 
    double entH = 0;
    double entM = 0;

    if(wind.x() >= th_wind_present) {
        estimateProbabilities(cells2, true, wind.y(), Eigen::Vector2i(i,j));
        for(int r=0; r<cells2.size(); r++){
            for(int c=0; c<cells2[0].size(); c++){
                double aux = cells2[r][c].weight*log(cells2[r][c].weight/cells[r][c].weight)+
                        (1-cells2[r][c].weight)*log((1-cells2[r][c].weight)/(1-cells[r][c].weight));
                entH += isnan(aux)?0:aux;
            }
        }
    }

    cells2 = cells;
    estimateProbabilities(cells2, false, wind.y(), Eigen::Vector2i(i,j));
    for(int r=0; r<cells2.size(); r++){
        for(int c=0; c<cells2[0].size();c++){
            double aux = cells2[r][c].weight*log(cells2[r][c].weight/cells[r][c].weight)+
                        (1-cells2[r][c].weight)*log((1-cells2[r][c].weight)/(1-cells[r][c].weight));
            entM+=isnan(aux)?0:aux;
        }
    }
    return cells[i][j].weight*entH + (1-cells[i][j].weight)*entM;
}

void InfotaxisGSL::moveTo(int i, int j) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    Eigen::Vector2d pos = indexToCoordinates(i,j);
    Eigen::Vector2d coordR = indexToCoordinates(currentPosIndex.x(),currentPosIndex.y());
    ROS_INFO("MOVING TO %f,%f",pos.x(),pos.y());

    double move_angle= (atan2(coordR.y()-pos.y(),coordR.x()-pos.x()));
    // goal.target_pose.pose.position.x = coordR.x();
    // goal.target_pose.pose.position.y = coordR.y();
    // goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angles::normalize_angle(move_angle));
    // mb_ac.sendGoal(goal, boost::bind(&InfotaxisGSL::goalDoneCallback, this,  _1, _2), boost::bind(&InfotaxisGSL::goalActiveCallback, this), boost::bind(&InfotaxisGSL::goalFeedbackCallback, this, _1));
    
    // ros::Rate r(0.4); // 10 hz
    // r.sleep();
    goal.target_pose.pose.position.x = pos.x();
    goal.target_pose.pose.position.y = pos.y();
    // goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angles::normalize_angle(move_angle));
    goal.target_pose.pose.orientation.w = 1.0;
    // while(!checkGoal(&goal));
    mb_ac.sendGoal(goal, boost::bind(&InfotaxisGSL::goalDoneCallback, this,  _1, _2), boost::bind(&InfotaxisGSL::goalActiveCallback, this), boost::bind(&InfotaxisGSL::goalFeedbackCallback, this, _1));

    inMotion=true;
}

void InfotaxisGSL::shift_array() {
    for(int i = sizeof(arxhit)/sizeof(int) - 2; i>=0; i--) {
        arxhit[i+1] = arxhit[i];
    }
}   

int InfotaxisGSL::cal_average() {
    int sum = 0;
    for(int i = 0; i < sizeof(arxhit)/sizeof(int) ; i++) {
        sum += arxhit[i];
    }
    return sum;
}  

// Fake sinal direction (1 is left)
void InfotaxisGSL::updateState(bool signal) {
    if (signal == true) {
        moth_state = 1;
        // Shift the tblank to append new value at the beginning
        for(int i = tblank_size - 2; i>=0; i--) {
            tblank_list[i+1] = tblank_list[i];
        }
        tblank_list[0] = (ros::Time::now() - tblank).toSec();
        tblank = ros::Time::now();
        last_hit = rand() % 2;
    } 
    else {
        if ((moth_state == 1) && ((ros::Time::now() - tblank).toSec() >= 0.7)) {
            moth_state = 2;}
        if ((moth_state == 2) && ((ros::Time::now() - tblank).toSec() >= 1.2)) {
            moth_state = 3;}
        if ((moth_state == 3) && ((ros::Time::now() - tblank).toSec() >= 1.9)) {
            moth_state = 4;}
        if ((moth_state == 4) && ((ros::Time::now() - tblank).toSec() >= 2.6)) {
            moth_state = 5;}
    }
    bool hybridSwitch = reactiveProbabilisticSwitch();
    if (hybridSwitch) {   
        planning_mode = "kbp";
        switch_notify(planning_mode);
        cancel_navigation();
    }
    if (!hybridSwitch && (planning_mode == "kbp") && ((ros::Time::now() - tblank).toSec() >= 10.0)) {
        planning_mode = "infotaxis";
    }
    if (planning_mode == "kbp") {
        getMothVel();
        moveToMoth();
    }
}   

void InfotaxisGSL::getMothVel() {
    switch (moth_state) {
        case 0: // stop
            linear_vel = 0.0;
            angular_vel = 0.0;
            break;
        case 1: // surge
            linear_vel = 0.1;
            angular_vel = 0.062;
            break;
        case 2: case 4:// turn
            linear_vel = 0.08;
            if (last_hit == 1) {
                angular_vel = -0.6;
            }
            else {
                angular_vel = 0.6;
            }
            break;
        case 3: case 5:// turn
            linear_vel = 0.08;
            if (last_hit == 1) {
                angular_vel = 0.6;
            }
            else {
                angular_vel = -0.6;
            }
            break;
        default:
            break;
    }
}

void InfotaxisGSL::moveToMoth() {
    // current_state = Infotaxis_state::MOVING;
    geometry_msgs::Twist msg;
    msg.linear.x = linear_vel;
    msg.angular.z = angular_vel;
    vel_pub.publish(msg);
    // ROS_INFO("DMMMMMMMMMMMMMMMMMMMMMMMM %f, %f", linear_vel, angular_vel); 
}

bool InfotaxisGSL::reactiveProbabilisticSwitch() {
    double sum = 0.0;
    double mean = 0.0;
    double variance = 0.0;
    double stddev = 0.0;

    for(int i = 0; i < tblank_size; ++i) {
        sum += tblank_list[i];
    }
    mean = sum / tblank_size;
    for(int i = 0; i < tblank_size; ++i) {
        variance += pow(tblank_list[i] - mean, 2);
    }
    stddev =  sqrt(variance/tblank_size);

    double burstiness = (stddev-mean) / (stddev+mean);
    return (burstiness + 0.5) < 0.0;
}


//========================= AUXILIARY FUNCTION ==============================

Eigen::Vector2i InfotaxisGSL::coordinatesToIndex(double x, double y){
    return Eigen::Vector2i((y-map_.info.origin.position.y)/(cell_map_ratio*map_.info.resolution),
                        (x-map_.info.origin.position.x)/(cell_map_ratio*map_.info.resolution));
}

Eigen::Vector2d InfotaxisGSL::indexToCoordinates(double i, double j){
    return Eigen::Vector2d(map_.info.origin.position.x+(j+0.5)*cell_map_ratio*map_.info.resolution,
                        map_.info.origin.position.y+(i+0.5)*cell_map_ratio*map_.info.resolution);
}

double InfotaxisGSL::gaussian(double distance, double sigma){
    return exp(-0.5*(pow(distance,2)/pow(sigma,2))) / (sigma*sqrt(2*M_PI));
}

Infotaxis_state InfotaxisGSL::getState(){
    return current_state;
}

float InfotaxisGSL::get_avg_wind_dir(std::vector<float> const &v) {
    //Average of wind direction, avoiding the problems of +/- pi angles.
    float x=0.0, y=0.0;
    for(std::vector<float>::const_iterator i=v.begin(); i!=v.end(); ++i) {
        x += cos(*i);
        y += sin(*i);
    }
    float average_angle = atan2(y, x);   
    return average_angle;
}

//ask the gmrf_wind service to estimate the wind vector of cells in openMoveSet
std::vector<WindVector> InfotaxisGSL::estimateWind(){
    gmrf_wind_mapping::WindEstimation srv;
    std::vector<std::pair<int,int> > indices;

    // Get coordinate of each cells in openMoveSet to call in GMRF_wind service
    for(auto& p: openMoveSet){
        Eigen::Vector2d coords = indexToCoordinates(p.first,p.second);
        srv.request.x.push_back(coords.x());
        srv.request.y.push_back(coords.y());
        indices.push_back(p);
    }

    // Call in GMRF_wind service and get result for cells in openMoveSet
    std::vector<WindVector> result(openMoveSet.size());
    if(clientW.call(srv)){
        for(int ind=0; ind<openMoveSet.size(); ind++){
            result[ind].i     = indices[ind].first;
            result[ind].j     = indices[ind].second;
            result[ind].speed = srv.response.u[ind];
            result[ind].angle = angles::normalize_angle(srv.response.v[ind]+M_PI);
        }
    }else{
        ROS_WARN("CANNOT READ ESTIMATED WIND VECTORS");
    }
    return result;
}

void InfotaxisGSL::calculateWeight(std::vector<std::vector<Cell> >& map, int i, int j, std::pair<int,int> p, 
    std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int> > >& openPropagationSet,
    std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int> > >& closedPropagationSet,
    std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int> > >& activePropagationSet){
    if(map[i][j].free && closedPropagationSet.find(std::pair<int,int>(i,j)) == closedPropagationSet.end() && activePropagationSet.find(std::pair<int,int>(i,j)) == activePropagationSet.end()){
        if(openPropagationSet.find(std::pair<int,int>(i,j))!= openPropagationSet.end()) {
            //if there already was a path to this cell
            double d = map[p.first][p.second].distance + ((i==p.first||j==p.second)?1:sqrt(2)); //distance of this new path to the same cell
            
            if(abs(d-map[i][j].distance)<0.1){ //if the distance is the same, keep the best probability!
                map[i][j].auxWeight=std::max(map[p.first][p.second].auxWeight , map[i][j].auxWeight);

            }else if(d<map[i][j].distance){ //keep the shortest path
                map[i][j].auxWeight=map[p.first][p.second].auxWeight;
                map[i][j].distance=d;
            }                        
        }else{
            map[i][j].auxWeight=map[p.first][p.second].auxWeight;
            map[i][j].distance = map[p.first][p.second].distance + ((i==p.first||j==p.second)?1:sqrt(2));
            openPropagationSet.insert(std::pair<int,int>(i,j));
        }
    }
}

void InfotaxisGSL::normalizeWeights(std::vector<std::vector<Cell> >& map) {
    double s = 0.0;
    for(int i=0; i<map.size(); i++){
        for(int j=0;j<map[0].size();j++){
            if(map[i][j].free)
                s+=map[i][j].weight;
        }
    }

    for(int i=0;i<map.size();i++){
        for(int j=0;j<map[0].size();j++){
            if(map[i][j].free)
                map[i][j].weight=map[i][j].weight/s;
        }
    }
}


Eigen::Vector2i InfotaxisGSL::get_jump_target(){
    double max = 0;
    int subenv_num = subenv_x.size() - 1;
    std::vector<float> p_subenv(subenv_num, 0.0); 

    for (size_t i = 1; i < subenv_x.size(); ++i) {
        for(int a=0; a<cells.size(); a++) {
            for(int b=0; b<cells[0].size(); b++) {
                if (subenv_x[i-1]<= cells[a][b].x && cells[a][b].x < subenv_x[i]) {
                    p_subenv[i-1] += cells[a][b].weight;
                }
            }
        }
    }
    for (size_t i = 0; i < p_subenv.size(); ++i) {
        ROS_INFO("subenv %ld: %f", i, p_subenv[i]); // Output: 1 2 3 4 5
    }
    auto maxIt = std::max_element(p_subenv.begin(), p_subenv.end());
    int index = std::distance(p_subenv.begin(), maxIt);
    bool is_in_max_subenv = (subenv_x[index] <= current_pose.pose.pose.position.x && current_pose.pose.pose.position.x < subenv_x[index+1]);

    if (!is_in_max_subenv && (number_steps>3)) {
        ROS_WARN("JUMP TO SUBENV %d!!!!!!!!!!!!!!!!!", index);
        Eigen::Vector2i jump_target = jump(index);
        return jump_target;     // jump with dijkstra
    }
    else {
        return Eigen::Vector2i(-100, -100);     // don't jump. continue with infotaxis
    }
}


Eigen::Vector2i InfotaxisGSL::jump(int subenv_id){
    int i, j;
    double max=0;
    for(int a=0; a<cells.size(); a++) {
        for(int b=0; b<cells[0].size(); b++) {
            if((cells[a][b].weight > max) && (subenv_x[subenv_id] <= cells[a][b].x && cells[a][b].x < subenv_x[subenv_id+1])) {
                max = cells[a][b].weight;
                i=a; j=b;
    }}}
    return Eigen::Vector2i(i,j);
}

//============================ VISUALIZATION ===============================

void InfotaxisGSL::showWeights() {
    visualization_msgs::Marker points = emptyMarker(numCells);
    for(int a=0; a<cells.size(); a++) {
        for(int b=0; b<cells[0].size(); b++) {
            if(cells[a][b].free){
                geometry_msgs::Point p; 
                    p.x=cells[a][b].x;
                    p.y=cells[a][b].y;
                points.points.push_back(p);

                Eigen::Vector3d col = valueToColor(cells[a][b].weight, 0.0001, convergence_thr);
                std_msgs::ColorRGBA color;
                    color.a=1;
                    color.r=col[0];
                    color.g=col[1];
                    color.b=col[2];
                points.colors.push_back(color);
            }
        }
    }
    probability_markers.publish(points);
}