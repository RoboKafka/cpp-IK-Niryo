/**
 * Ned2 Pose Validator using KDL (Kinematics and Dynamics Library)
 * 
 * Build:
 *   sudo apt install liborocos-kdl-dev liburdfdom-dev
 *   g++ -std=c++17 -o ned2_validator ned2_validator.cpp \
 *       -lorocos-kdl -lurdfdom_model -lkdl_parser
 * 
 * Usage:
 *   ./ned2_validator <urdf_file> <x> <y> <z> <rx> <ry> <rz>
 *   ./ned2_validator niryo_ned2.urdf 0.25 0.0 0.30 0.0 1.57 0.0
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <vector>
#include <string>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

struct PoseResult {
    bool valid;
    std::vector<double> joints_rad;
    std::vector<double> joints_deg;
    double position_error_mm;
    double orientation_error_deg;
    std::string reason;
};

class Ned2Validator {
public:
    // Joint limits from URDF (radians)
    const std::vector<std::pair<double, double>> JOINT_LIMITS = {
        {-2.999871918328051, 2.999871918328051},   // joint_1
        {-1.8325957145941667, 0.6101671064972578}, // joint_2
        {-1.3400637996813345, 1.5700981950942021}, // joint_3
        {-2.090031779263347, 2.090031779263347},   // joint_4
        {-1.9200367101190883, 1.9228292369222795}, // joint_5
        {-2.53, 2.53}                               // joint_6
    };

    Ned2Validator(const std::string& urdf_file) {
        // Load URDF
        urdf::Model urdf_model;
        if (!urdf_model.initFile(urdf_file)) {
            throw std::runtime_error("Failed to load URDF: " + urdf_file);
        }

        // Parse to KDL tree
        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) {
            throw std::runtime_error("Failed to construct KDL tree");
        }

        // Extract chain from base to end-effector
        std::string base_link = "base_link";
        std::string tip_link = "hand_link";
        
        if (!kdl_tree.getChain(base_link, tip_link, chain_)) {
            throw std::runtime_error("Failed to get chain from " + base_link + " to " + tip_link);
        }

        std::cout << "Loaded chain with " << chain_.getNrOfJoints() << " joints" << std::endl;

        // Initialize solvers
        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
        ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_);
    }

    PoseResult validatePose(
        double x, double y, double z,
        double rx, double ry, double rz,
        double position_tolerance_mm = 25.0,
        double orientation_tolerance_deg = 5.0
    ) {
        PoseResult result;
        result.valid = false;
        result.position_error_mm = 0;
        result.orientation_error_deg = 0;

        // Create target frame from position and RPY
        KDL::Rotation target_rot = KDL::Rotation::RPY(rx, ry, rz);
        KDL::Vector target_pos(x, y, z);
        KDL::Frame target_frame(target_rot, target_pos);

        // Initial joint configuration (zeros)
        unsigned int nj = chain_.getNrOfJoints();
        KDL::JntArray q_init(nj);
        KDL::JntArray q_sol(nj);

        for (unsigned int i = 0; i < nj; i++) {
            q_init(i) = 0.0;
        }

        // Solve IK
        int ik_result = ik_solver_->CartToJnt(q_init, target_frame, q_sol);

        if (ik_result < 0) {
            result.reason = "IK solver failed (code: " + std::to_string(ik_result) + ")";
            return result;
        }

        // Extract joint values
        result.joints_rad.resize(nj);
        result.joints_deg.resize(nj);
        for (unsigned int i = 0; i < nj; i++) {
            result.joints_rad[i] = q_sol(i);
            result.joints_deg[i] = q_sol(i) * 180.0 / M_PI;
        }

        // Check joint limits
        for (unsigned int i = 0; i < std::min(nj, (unsigned int)JOINT_LIMITS.size()); i++) {
            double lower = JOINT_LIMITS[i].first;
            double upper = JOINT_LIMITS[i].second;
            
            if (q_sol(i) < lower  || q_sol(i) > upper ) {
                result.reason = "joint_" + std::to_string(i + 1) + 
                               " (" + std::to_string(result.joints_deg[i]) + "°) outside limits";
                return result;
            }
        }

        // Verify with FK
        KDL::Frame achieved_frame;
        fk_solver_->JntToCart(q_sol, achieved_frame);

        // Position error
        KDL::Vector pos_diff = target_pos - achieved_frame.p;
        double pos_error = pos_diff.Norm();
        result.position_error_mm = pos_error * 1000.0;

        // Orientation error (angle between rotations)
        KDL::Rotation rot_diff = target_rot.Inverse() * achieved_frame.M;
        double angle;
        KDL::Vector axis;
        rot_diff.GetRotAngle(axis, angle);
        result.orientation_error_deg = std::abs(angle) * 180.0 / M_PI;

        // Check tolerances
        if (result.position_error_mm > position_tolerance_mm) {
            result.reason = "position error " + std::to_string(result.position_error_mm) + 
                           "mm > " + std::to_string(position_tolerance_mm) + "mm";
            return result;
        }

        if (result.orientation_error_deg > orientation_tolerance_deg) {
            result.reason = "orientation error " + std::to_string(result.orientation_error_deg) + 
                           "° > " + std::to_string(orientation_tolerance_deg) + "°";
            return result;
        }

        result.valid = true;
        result.reason = "Valid pose";
        return result;
    }

    // Forward kinematics
    KDL::Frame forwardKinematics(const std::vector<double>& joints) {
        KDL::JntArray q(joints.size());
        for (size_t i = 0; i < joints.size(); i++) {
            q(i) = joints[i];
        }
        
        KDL::Frame frame;
        fk_solver_->JntToCart(q, frame);
        return frame;
    }

private:
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
};


void printResult(const PoseResult& result, double x, double y, double z, double rx, double ry, double rz) {
    std::cout << (result.valid ? "✓ VALID" : "✗ INVALID") << std::endl;
    std::cout << "  Position: (" << x << ", " << y << ", " << z << ") m" << std::endl;
    std::cout << "  Orientation: (" << rx * 180/M_PI << "°, " << ry * 180/M_PI << "°, " << rz * 180/M_PI << "°)" << std::endl;
    
    if (result.valid) {
        std::cout << "  Position error: " << result.position_error_mm << " mm" << std::endl;
        std::cout << "  Orientation error: " << result.orientation_error_deg << "°" << std::endl;
        std::cout << "  Joints (deg): [";
        for (size_t i = 0; i < result.joints_deg.size(); i++) {
            std::cout << result.joints_deg[i];
            if (i < result.joints_deg.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    } else {
        std::cout << "  Reason: " << result.reason << std::endl;
    }
}


int main(int argc, char** argv) {
    if (argc < 8) {
        std::cerr << "Usage: " << argv[0] << " <urdf_file> <x> <y> <z> <rx> <ry> <rz> [tolerance_mm]" << std::endl;
        std::cerr << std::endl;
        std::cerr << "Arguments:" << std::endl;
        std::cerr << "  urdf_file    Path to URDF file" << std::endl;
        std::cerr << "  x, y, z      Position in meters" << std::endl;
        std::cerr << "  rx, ry, rz   Orientation in radians (roll, pitch, yaw)" << std::endl;
        std::cerr << "  tolerance_mm Position tolerance (default: 5.0)" << std::endl;
        std::cerr << std::endl;
        std::cerr << "Example:" << std::endl;
        std::cerr << "  " << argv[0] << " niryo_ned2.urdf 0.25 0.0 0.30 0.0 1.57 0.0" << std::endl;
        return 1;
    }

    std::string urdf_file = argv[1];
    double x = std::stod(argv[2]);
    double y = std::stod(argv[3]);
    double z = std::stod(argv[4]);
    double rx = std::stod(argv[5]);
    double ry = std::stod(argv[6]);
    double rz = std::stod(argv[7]);
    double tolerance_mm = (argc > 8) ? std::stod(argv[8]) : 5.0;

    try {
        Ned2Validator validator(urdf_file);
        
        PoseResult result = validator.validatePose(x, y, z, rx, ry, rz, tolerance_mm);
        printResult(result, x, y, z, rx, ry, rz);
        
        return result.valid ? 0 : 1;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
