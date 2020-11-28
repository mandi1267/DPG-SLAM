

#include "dpg_slam.h"

// TODO: Need to add pieces related to pose graph slam

namespace dpg_slam {

    DpgSLAM::DpgSLAM(const DpgParameters &dpg_parameters,
                     const PoseGraphParameters &pose_graph_parameters) : dpg_parameters_(dpg_parameters),
                     pose_graph_parameters_(pose_graph_parameters) {
        // TODO
    }

    void DpgSLAM::ObserveLaser(const std::vector<float>& ranges,
                      float range_min,
                      float range_max,
                      float angle_min,
                      float angle_max) {
        // TODO
    }

    // Observe new odometry-reported location.
    void DpgSLAM::ObserveOdometry(const Eigen::Vector2f& odom_loc,
                         const float odom_angle) {
        // TODO
    }

    bool DpgSLAM::shouldProcessLaser() {

        // Check if odom has changed enough since last time we processed the laser
        return false; // TODO fixed
    }
    
    // TODO::FIX the output structure to represent the labels
    std::vector<dpgMapPoint> DpgSLAM::GetActiveMap(){
  	return curr_active_map_;
    }
	
    std::vector<dpgMapPoint> DpgSLAM::GetDynamicMap(){
	return curr_dynamic_map_;
    }

    // TODO::FIX the output structure to represent the labels
    std::vector<Eigen::Vector2f> DpgSLAM::GetPoseGraphMap(){
	std::vector<Eigen::Vector2f> allMapPoints;
   	/**
	 * TODO: This should return all the map points in the dynamic map
	 */ 
	return allMapPoints;
    
    }

    std::pair<occupancyGrid, occupancyGrid> DpgSLAM::computeLocalSubMap(){
    
    	occupancyGrid currGrid;
	occupancyGrid subMapGrid;

	//TODO: Fill in the occupancy grid data structure
	//TODO: Implement Section 3A from the paper to fill in both the grids.
	
	
	return std::make_pair(currGrid, subMapGrid);
    }

    std::vector<dpgMapPoint> DpgSLAM::detectAndLabelChanges(const occupancyGrid& currGrid, const occupancyGrid& submapGrid){
    
	std::vector<dpgMapPoint> dynamicMapPoints;
	//TODO: Implement Alg. 1 from the paper.
	
	return dynamicMapPoints;
    }

    std::vector<DpgNode> DpgSLAM::UpdateActiveAndDynamicMaps(const std::vector<dpgMapPoint> &removedPoints){
    
    	std::vector<DpgNode> inactiveNodes;
	//TODO: Implement Alg. 2 from the paper.
	

	return inactiveNodes;
    }

    void DpgSLAM::updateActiveMap(){
	
	//TODO: update active and dynamic maps as described in section II.C.2 
    }

}
