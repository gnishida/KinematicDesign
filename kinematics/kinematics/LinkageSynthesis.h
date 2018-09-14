#pragma once

#include <vector>
#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>
#include "Solution.h"
#include "BBox.h"
#include "KinematicUtils.h"
#include "Kinematics.h"

namespace kinematics {

	class LinkageSynthesis {
	protected:
		std::vector<Object25D> fixed_bodies;
		std::pair<double, double> sigmas;
		bool avoid_branch_defect;
		double min_transmission_angle;
		double min_link_length;
		std::vector<double> weights;

	protected:
		LinkageSynthesis() {}

	public:
		static void calculateStatistics(const std::vector<double>& values, double& mean, double& sd);
		static bool compare(const Solution& s1, const Solution& s2);
		static std::vector<glm::dmat3x3> perturbPoses(const std::vector<glm::dmat3x3>& poses, std::pair<double, double>& sigmas, double& position_error, double& orientation_error);
		static std::vector<glm::dvec2> enlargePolygon(const std::vector<glm::dvec2>& polygon, const glm::dvec2& center, double scale);
		static void createDistanceMapForLinkageRegion(const std::vector<glm::dvec2>& linkage_region_pts, double scale, BBox& dist_map_bbox, cv::Mat& dist_map);
		void particleFilter(const std::vector<glm::dmat3x3>& poses, std::vector<Solution>& solutions, const cv::Mat& dist_map, const BBox& dist_map_bbox, const std::vector<glm::dvec2>& linkage_avoidance_pts, const Object25D& moving_body, int num_particles, int num_iterations, bool record_file);
		void resample(std::vector<Solution> particles, int N, std::vector<Solution>& resampled_particles, double max_cost);

		virtual void calculateSolution(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_avoidance_pts, int num_samples, const Object25D& moving_body, std::vector<Solution>& solutions) = 0;
		virtual bool optimizeCandidate(const std::vector<glm::dmat3x3>& poses, std::vector<glm::dvec2>& points) = 0;
		virtual Solution findBestSolution(const std::vector<glm::dmat3x3>& poses, std::vector<Solution>& solutions, const cv::Mat& dist_map, const BBox& dist_map_bbox, const std::vector<glm::dvec2>& linkage_avoidance_pts, const Object25D& moving_body, int num_particles, int num_iterations, bool record_file) = 0;
		virtual bool checkHardConstraints(std::vector<glm::dvec2>& points, const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_avoidance_pts, const Object25D& moving_body, std::vector<std::vector<int>>& zorder) = 0;
		virtual int getType(const std::vector<glm::dvec2>& points) = 0;
		virtual bool checkOrderDefect(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points) = 0;
		virtual bool checkBranchDefect(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points) = 0;
		virtual bool checkCircuitDefect(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points) = 0;
		virtual Kinematics constructKinematics(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points, const std::vector<std::vector<int>>& zorder, const Object25D& moving_body, bool connect_joints, const std::vector<Object25D>& fixed_bodies, std::vector<glm::dvec2>& connected_pts) = 0;
		virtual void updateMovingBodies(Kinematics& kinematics, const Object25D& moving_body) = 0;
		virtual double calculateCost(Solution& solution, const Object25D& moving_body, const cv::Mat& dist_map, const BBox& dist_map_bbox) = 0;

		virtual void generate3DGeometry(const Kinematics& kinematics, std::vector<Vertex>& vertices) = 0;
		virtual void saveSTL(const QString& dirname, const std::vector<Kinematics>& kinematics) = 0;
		virtual void saveSCAD(const QString& dirname, int index, const Kinematics& kinematics) = 0;
	};

}