#pragma once

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "parameters.h"
#include "dpg_node.h"

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>

// TODO: Need to add pieces related to pose graph slam    

namespace dpg_slam {
	
    /**
     * This enum defines the possible labels for active and dynamic maps.
     */
    enum MapLabel {STATIC, ADDED, REMOVED};

    /**
     * Data structure to define the occupancy grid.
     */
    struct occupancyGrid {
  	// TODO: Think of way to define this keeping the functionality required in Alg. 1 in mind.	
	//std::unordered_map<std::pair<int, int>, bool> gridInfo;
    };

    /**
     * Data structure to define a map point in the active and dynamic maps.
     */
    struct dpgMapPoint{

    	Eigen::Vector2f mapPoint;
	MapLabel label;
    };

    

    /**
     * Data structure for DPG navigation graph containing gpd nodes and edges.
     */
    
    class DpgSLAM {
    public:

        /**
         * Create the DPG SLAM object.
         *
         * @param dpg_parameters        DPG specific parameters.
         * @param pose_graph_parameters Pose graph parameters.
         */
        DpgSLAM(const DpgParameters &dpg_parameters, const PoseGraphParameters &pose_graph_parameters);

        // Observe a new laser scan.
        void ObserveLaser(const std::vector<float>& ranges,
                          float range_min,
                          float range_max,
                          float angle_min,
                          float angle_max);

	/**
	 * Get latest active map 
	 */
	std::vector<dpgMapPoint> GetActiveMap();

	// Get latest dynamic map
	std::vector<dpgMapPoint> GetDynamicMap();

	// Get latest poseGraph map for comparision
	std::vector<Eigen::Vector2f> GetPoseGraphMap();
	
        // Observe new odometry-reported location.
        void ObserveOdometry(const Eigen::Vector2f& odom_loc,
					const float odom_angle);
    private:

        /**
         * Determine if the robot has moved far enough that we should compare the last used laser scan to the current laser
         * scan.
         *
         * @return True if the robot has moved far enough that we should compare the last used laser scan to the current
         * laser scan, false if we should skip this laser scan.
         */
        bool shouldProcessLaser();
	
	/*
	 * computes the local submapGrid and the currGrid
	 * each map is an unordered map from keys in x,y to boolean which says if the grid is occupied or not (int, int)->bool
	 * TODO: Need to figure out the data structure for occupancy grids/
	 */
	std::pair<occupancyGrid, occupancyGrid> computeLocalSubMap();
         
	/**
	 * This method compares each cell in the currGrid to the corresponding cell is the submap.
	 * @param currGrid 		occupancy grid of current pose chain obtained from computeLocalSubmap
	 * @param submapGrid		occupancy grid of local submap obtained from computeLocalSubmap
	 *
	 * @return Vector of dynamic map points with associated labels for measurements in current Grid
	 */
	std::vector<dpgMapPoint> detectAndLabelChanges(const occupancyGrid& currGrid, const occupancyGrid& submapGrid);

	/**
	 * @param removedPoints 	Points from the dynamic map points obtained from detectAndLabelChanges that
	 * 				are labelled as "REMOVED".
	 *
	 * @return vector of inactive nodes.
	 */
	std::vector<DpgNode> UpdateActiveAndDynamicMaps(const std::vector<dpgMapPoint> &removedPoints);

	/**
	 * Method to delete inactive nodes from dpg graph
	 * NOTE: This is not a top priority.
	 */
	void reduceDPGSize();
	
	/**
         * GTSAM factor graph.
         */
        gtsam::NonlinearFactorGraph* graph_;

	/**
	 * dpg pose graph
	 */
	dpgGraph DpgPoseGraph_;

        /**
         * Initial estimates for GTSAM.
         */
        gtsam::Values initialEstimates_;

        /**
         * DPG specific parameters.
         */
        DpgParameters dpg_parameters_;

        /**
         * Pose graph parameters.
         */
        PoseGraphParameters pose_graph_parameters_;
    
    	/**
	 * To store the nodes in every pass for using in local submap computation.
	 */
	std::vector<DpgNode> pervious_nodes_;
	
	/**
	 * To store the nodes in current pose chain obtained from latest pass
	 */
	std::vector<DpgNode> current_pose_chain_nodes_;

	/**
	 * current dynamic map
	 */
	std::vector<dpgMapPoint> curr_dynamic_map_;

	/**
         * current active map
         */
        std::vector<dpgMapPoint> curr_active_map_;

	/**
	 * current pose graph map
	 */
	std::vector<Eigen::Vector2f> curr_pose_graph_map_;

	/*
	 * Method to update active and dynamic map after updating labels and identifying inactive nodes
	 */
	void updateActiveMap();
	
    };
}  // end dpg_slam
