#include <gsl_infotaxis.h>
#include <boost/format.hpp>

Cell::Cell(bool f, double a, double b, double c) {
    free=f;
    x=a;
    y=b;
    weight=c;
    auxWeight=0;
    distance=0;
}

InfotaxisGSL::InfotaxisGSL(ros::NodeHandle *nh) : GSLAlgorithm(nh), VisualCPT(nh) {
    nh->param<double>("th_gas_present", th_gas_present, 0.3);
    nh->param<double>("th_wind_present", th_wind_present, 0.03);
    nh->param<double>("stop_and_measure_time", stop_and_measure_time, 3);
    nh->param<double>("scale", scale, 10);                      //scale for dynamic map reduction
    // nh->param<double>("convergence_thr", convergence_thr, 0.5); //threshold for source declaration
    nh->param<double>("stdev_hit", stdev_hit, 1.0);             //standard deviation of hit and miss?
    nh->param<double>("stdev_miss", stdev_miss, 2.0);
    // nh->param<double>("ground_truth_x", ground_truth_x, 1.5);
    // nh->param<double>("ground_truth_y", ground_truth_y, 3.0);
    
    // Subscribers & publisher 
    gas_sub_  = nh->subscribe(enose_topic,1,&InfotaxisGSL::gasCallback, this);
    wind_sub_ = nh->subscribe(anemometer_topic,1,&InfotaxisGSL::windCallback, this);
    map_sub_  = nh->subscribe(map_topic, 1, &InfotaxisGSL::mapCallback, this);

    probability_markers = nh->advertise<visualization_msgs::Marker>("probability_markers", 10);
    entropy_reporter    = nh->advertise<std_msgs::Float32>("entropy_reporter", 10);
    
    // Init State
    gasHit           = false;
    number_revisited = 0;
    planning_mode    = 0;
    previous_state   = Infotaxis_state::WAITING_FOR_MAP;
    current_state    = Infotaxis_state::WAITING_FOR_MAP; 

    ROS_INFO("INITIALIZATON COMPLETED--> WAITING_FOR_MAP");
  	ros::NodeHandle n;
    clientW = n.serviceClient<gmrf_wind_mapping::WindEstimation>("/WindEstimation");            //de y!! tam thoi bo qua
    entropy_gain_his.push_back(0.0); 
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
    int cells_height = ceil((float)map_height/scale);
    int cells_width = ceil((float)map_width/scale);
    cells.resize(cells_height);
    for(auto &cell :cells){
        cell.resize(cells_width, Cell(false,0,0,0));
    }

    numCells = 0; //number free cell in reduced map
    int cellsI=0, cellsJ=0;

    for(int i=0; i<map_origin.size(); i+=scale) {
        cellsJ=0;
        for(int j=0; j<map_origin[0].size(); j+=scale) {
            bool libre=true;                        //free cell?
            for(int row=i; row<map_origin.size() && row<i+scale; row++){
                for(int col=j;col<map_origin[0].size()&&col<j+scale;col++){
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
    //Only save gas data if we are in the Stop_and_Measure
    std::string miss_gas = "nothing";
    if (current_state == Infotaxis_state::STOP_AND_MEASURE) {
        gasHit = (miss_gas.compare(msg->data.c_str()) == 0)? false : true;
        gas_vector.push_back((float)gasHit);
    }
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
            // previous_robot_pose = Eigen::Vector2d(current_robot_pose.pose.pose.position.x, current_robot_pose.pose.pose.position.y); //register where you were before moving
            estimateProbabilities(cells, gasHit, avg_wind_dir, currentPosIndex);
            current_state = Infotaxis_state::MOVING;
            
        }
        else if (avg_concentration > th_gas_present) {
            gasHit=true;
            ROS_WARN("GAS, BUT NO WIND  New state --> MOVING");
            hit_notify(gasHit);
            // previous_robot_pose=Eigen::Vector2d(current_robot_pose.pose.pose.position.x, current_robot_pose.pose.pose.position.y); 
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
    
    //estimate the probabilities for the immediate 8 neighbours
    Eigen::Vector2d coordR = indexToCoordinates(i,j);
    double upwind_dir      = angles::normalize_angle(wind_direction+M_PI);
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
                        dist=gaussian(atan2(sin(move_dir-cell_vector), cos(move_dir-cell_vector)), stdev_miss);
                    }
                    activePropagationSet.insert(std::pair<int,int>(r,c));
                    map[r][c].weight=dist*map[r][c].weight; 
                    map[r][c].auxWeight=dist;
                    map[r][c].distance=(r==i||c==j)?1:sqrt(2);
                }
            }
        }
    }
    
    map[i][j].weight = map[i][j].weight*gaussian((hit?0:M_PI), (hit?stdev_hit:stdev_miss));
    // map[i][j].weight = 0;
    // closedPropagationSet.insert(std::pair<int,int>(i,j));
    map[i][j].auxWeight=0;
    map[i][j].distance=0;

    //propagate these short-range estimations to the entire environment using the navigation map
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
            map[par.first][par.second].weight=map[par.first][par.second].weight*map[par.first][par.second].auxWeight;
        }
        activePropagationSet=openPropagationSet;
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
    double ent    = -100;
    double entAux = 0;

    if (planning_mode == 0) {
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
        
        entropy_gain_his.push_back(ent);
        ROS_ERROR("Step: %li", entropy_gain_his.size());

        std_msgs::Float32 max_entropy_gain;
        max_entropy_gain.data = get_average_vector(entropy_gain_rate);    
        entropy_reporter.publish(max_entropy_gain);
        openMoveSet.erase(std::pair<int,int>(i,j));
    }

    else {
        double max=0;
        // Detect max and min weights in the map
        for(int a=0; a<cells.size(); a++) {
            for(int b=0; b<cells[0].size(); b++) {
                if(cells[a][b].weight > max) {
                    max = cells[a][b].weight;
                    i=a; j=b;
                }
            }
        }
        openMoveSet.erase(std::pair<int,int>(i,j));
    }
    switch_notify(planning_mode);
    planning_mode = 0;  //switching back to infotaxis
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
    ROS_ERROR("ENTROPY_GAIN: %f", get_average_vector(entropy_gain_rate));

    if (number_revisited > 0){
        ros::Duration time_spent = ros::Time::now() - last_revisited;
        if (time_spent.toSec() > 60.0) {
            number_revisited = 0;
            visitedSet.clear();
            ROS_WARN("CLEAN!!!!!!");
        }
    }

    if (visitedSet.find(std::pair<int,int>(i,j)) != visitedSet.end()) {
        last_revisited = ros::Time::now();
        
        number_revisited += 1;
        ROS_ERROR("number of revisited: %i", number_revisited);

        if ((number_revisited > 5 && get_average_vector(entropy_gain_rate) < 0.05) || number_revisited > 10) {
            planning_mode = 1;
            number_revisited = 0;
            visitedSet.clear();
            ROS_WARN("SWITCHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH!!!");
        }
    }
    visitedSet.insert(std::pair<int,int>(i,j));

    // check wether pos is near the boundary
    int oI = std::max(0, i-1);
    int fI = std::min((int) cells.size()-1, i+1);
    int oJ = std::max(0, j-1);
    int fJ = std::min((int) cells[0].size()-1, j+1);

    openMoveSet.clear();

    for(int r=oI; r<=fI; r++){
        for(int c=oJ; c<=fJ; c++){
            // if(r==i && c==j){
            //     continue;   //Never stay in the same place
            // }
            std::pair<int,int> p(r,c);
            if(cells[r][c].free && cells[r][c].distance == 1.0) {
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

    double move_angle= (atan2(pos.y()-coordR.y(),pos.x()-coordR.x()));
    // goal.target_pose.pose.position.x = coordR.x();
    // goal.target_pose.pose.position.y = coordR.y();
    // goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angles::normalize_angle(move_angle));
    // mb_ac.sendGoal(goal, boost::bind(&InfotaxisGSL::goalDoneCallback, this,  _1, _2), boost::bind(&InfotaxisGSL::goalActiveCallback, this), boost::bind(&InfotaxisGSL::goalFeedbackCallback, this, _1));
    
    ros::Rate r(0.6); // 10 hz
    r.sleep();
    goal.target_pose.pose.position.x = pos.x();
    goal.target_pose.pose.position.y = pos.y();
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angles::normalize_angle(move_angle));
    while(!checkGoal(&goal));
    mb_ac.sendGoal(goal, boost::bind(&InfotaxisGSL::goalDoneCallback, this,  _1, _2), boost::bind(&InfotaxisGSL::goalActiveCallback, this), boost::bind(&InfotaxisGSL::goalFeedbackCallback, this, _1));

    inMotion=true;
}


//========================= AUXILIARY FUNCTION ==============================

Eigen::Vector2i InfotaxisGSL::coordinatesToIndex(double x, double y){
    return Eigen::Vector2i((y-map_.info.origin.position.y)/(scale*map_.info.resolution),
                        (x-map_.info.origin.position.x)/(scale*map_.info.resolution));
}

Eigen::Vector2d InfotaxisGSL::indexToCoordinates(double i, double j){
    return Eigen::Vector2d(map_.info.origin.position.x+(j+0.5)*scale*map_.info.resolution,
                        map_.info.origin.position.y+(i+0.5)*scale*map_.info.resolution);
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