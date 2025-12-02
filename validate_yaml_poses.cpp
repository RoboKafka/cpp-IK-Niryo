/**
 * Validate poses from YAML file using KDL
 * 
 * Build:
 *   sudo apt install liborocos-kdl-dev liburdfdom-dev libkdl-parser-dev libyaml-cpp-dev
 *   
 *   mkdir build && cd build
 *   cmake ..
 *   make
 * 
 * Usage:
 *   ./validate_yaml_poses <urdf_file> <yaml_file> [--tolerance <mm>] [--section <name>]
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <vector>
#include <string>
#include <map>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <yaml-cpp/yaml.h>

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
    const std::vector<std::pair<double, double>> JOINT_LIMITS = {
        {-2.999871918328051, 2.999871918328051},
        {-1.8325957145941667, 0.6101671064972578},
        {-1.3400637996813345, 1.5700981950942021},
        {-2.090031779263347, 2.090031779263347},
        {-1.9200367101190883, 1.9228292369222795},
        {-2.53, 2.53}
    };

    Ned2Validator(const std::string& urdf_file) {
        urdf::Model urdf_model;
        if (!urdf_model.initFile(urdf_file)) {
            throw std::runtime_error("Failed to load URDF: " + urdf_file);
        }

        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) {
            throw std::runtime_error("Failed to construct KDL tree");
        }

        if (!kdl_tree.getChain("base_link", "hand_link", chain_)) {
            throw std::runtime_error("Failed to get chain");
        }

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

        KDL::Rotation target_rot = KDL::Rotation::RPY(rx, ry, rz);
        KDL::Vector target_pos(x, y, z);
        KDL::Frame target_frame(target_rot, target_pos);

        unsigned int nj = chain_.getNrOfJoints();
        KDL::JntArray q_init(nj);
        KDL::JntArray q_sol(nj);

        for (unsigned int i = 0; i < nj; i++) {
            q_init(i) = 0.0;
        }

        int ik_result = ik_solver_->CartToJnt(q_init, target_frame, q_sol);

        if (ik_result < 0) {
            result.reason = "IK failed (code: " + std::to_string(ik_result) + ")";
            return result;
        }

        result.joints_rad.resize(nj);
        result.joints_deg.resize(nj);
        for (unsigned int i = 0; i < nj; i++) {
            result.joints_rad[i] = q_sol(i);
            result.joints_deg[i] = q_sol(i) * 180.0 / M_PI;
        }

        // Check joint limits
        for (unsigned int i = 0; i < std::min(nj, (unsigned int)JOINT_LIMITS.size()); i++) {
            if (q_sol(i) < JOINT_LIMITS[i].first - 0.01 || 
                q_sol(i) > JOINT_LIMITS[i].second + 0.01) {
                result.reason = "joint_" + std::to_string(i + 1) + " outside limits";
                return result;
            }
        }

        // FK verification
        KDL::Frame achieved_frame;
        fk_solver_->JntToCart(q_sol, achieved_frame);

        KDL::Vector pos_diff = target_pos - achieved_frame.p;
        result.position_error_mm = pos_diff.Norm() * 1000.0;

        KDL::Rotation rot_diff = target_rot.Inverse() * achieved_frame.M;
        double angle;
        KDL::Vector axis;
        rot_diff.GetRotAngle(axis, angle);
        result.orientation_error_deg = std::abs(angle) * 180.0 / M_PI;

        if (result.position_error_mm > position_tolerance_mm) {
            result.reason = "position error " + std::to_string(result.position_error_mm) + 
                           "mm > " + std::to_string(position_tolerance_mm) + "mm";
            return result;
        }

        if (result.orientation_error_deg > orientation_tolerance_deg) {
            result.reason = "orientation error " + std::to_string(result.orientation_error_deg) + "°";
            return result;
        }

        result.valid = true;
        result.reason = "Valid";
        return result;
    }

private:
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
};


void validatePolarWorkspace(Ned2Validator& validator, const YAML::Node& polar_data, double tolerance_mm) {
    int total_valid = 0;
    int total_count = 0;

    for (const auto& plane : polar_data) {
        std::string plane_name = plane.first.as<std::string>();
        const YAML::Node& points = plane.second;

        std::cout << "\n" << std::string(70, '=') << std::endl;
        std::cout << "  " << plane_name << " (tolerance: " << tolerance_mm << "mm)" << std::endl;
        std::cout << std::string(70, '=') << std::endl;

        int plane_valid = 0;
        int plane_count = 0;

        for (size_t i = 0; i < points.size(); i++) {
            const YAML::Node& pose = points[i];
            
            if (!pose.IsSequence() || pose.size() != 6) continue;

            // Check all values are numeric
            bool all_numeric = true;
            for (size_t j = 0; j < 6; j++) {
                if (!pose[j].IsScalar()) {
                    all_numeric = false;
                    break;
                }
                try {
                    pose[j].as<double>();
                } catch (...) {
                    all_numeric = false;
                    break;
                }
            }
            if (!all_numeric) continue;

            double x = pose[0].as<double>();
            double y = pose[1].as<double>();
            double z = pose[2].as<double>();
            double rx = pose[3].as<double>();
            double ry = pose[4].as<double>();
            double rz = pose[5].as<double>();

            plane_count++;
            total_count++;

            PoseResult result = validator.validatePose(x, y, z, rx, ry, rz, tolerance_mm);

            int angle = i * 30;
            if (result.valid) {
                plane_valid++;
                total_valid++;
                std::cout << "  ✓ A" << angle << ": (" 
                         << x << ", " << y << ", " << z << ") | err: " 
                         << result.position_error_mm << "mm" << std::endl;
            } else {
                std::cout << "  ✗ A" << angle << ": (" 
                         << x << ", " << y << ", " << z << ") | " 
                         << result.reason << std::endl;
            }
        }

        std::cout << "\n  Plane: " << plane_valid << "/" << plane_count << " valid" << std::endl;
    }

    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << "  TOTAL: " << total_valid << "/" << total_count << " poses valid" << std::endl;
    std::cout << std::string(70, '=') << std::endl;
}


void validateSection(Ned2Validator& validator, const YAML::Node& section_data, 
                     const std::string& section_name, double tolerance_mm) {
    std::cout << "\n" << std::string(70, '#') << std::endl;
    std::cout << "  SECTION: " << section_name << std::endl;
    std::cout << std::string(70, '#') << std::endl;

    int valid_count = 0;
    int total_count = 0;

    std::function<void(const YAML::Node&, const std::string&)> processNode;
    processNode = [&](const YAML::Node& node, const std::string& path) {
        if (node.IsSequence() && node.size() == 6) {
            // Check if it's a pose
            bool all_numeric = true;
            for (size_t i = 0; i < 6; i++) {
                if (!node[i].IsScalar()) {
                    all_numeric = false;
                    break;
                }
                try {
                    node[i].as<double>();
                } catch (...) {
                    all_numeric = false;
                    break;
                }
            }

            if (all_numeric) {
                total_count++;
                double x = node[0].as<double>();
                double y = node[1].as<double>();
                double z = node[2].as<double>();
                double rx = node[3].as<double>();
                double ry = node[4].as<double>();
                double rz = node[5].as<double>();

                PoseResult result = validator.validatePose(x, y, z, rx, ry, rz, tolerance_mm);

                if (result.valid) {
                    valid_count++;
                    std::cout << "  ✓ " << path << ": VALID (err=" 
                             << result.position_error_mm << "mm)" << std::endl;
                } else {
                    std::cout << "  ✗ " << path << ": " << result.reason << std::endl;
                }
            } else {
                std::cout << "  ⊘ " << path << ": Skipped (placeholder)" << std::endl;
            }
        } else if (node.IsSequence()) {
            for (size_t i = 0; i < node.size(); i++) {
                processNode(node[i], path + "[" + std::to_string(i) + "]");
            }
        } else if (node.IsMap()) {
            for (const auto& kv : node) {
                std::string new_path = path.empty() ? kv.first.as<std::string>() 
                                                    : path + "." + kv.first.as<std::string>();
                processNode(kv.second, new_path);
            }
        }
    };

    processNode(section_data, "");

    std::cout << "\n  Section: " << valid_count << "/" << total_count << " valid" << std::endl;
}


int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <urdf_file> <yaml_file> [options]" << std::endl;
        std::cerr << std::endl;
        std::cerr << "Options:" << std::endl;
        std::cerr << "  --tolerance <mm>    Position tolerance (default: 25.0)" << std::endl;
        std::cerr << "  --section <name>    Validate specific section only" << std::endl;
        std::cerr << std::endl;
        std::cerr << "Example:" << std::endl;
        std::cerr << "  " << argv[0] << " niryo_ned2.urdf demo.yaml" << std::endl;
        std::cerr << "  " << argv[0] << " niryo_ned2.urdf demo.yaml --section polar_workspace_points" << std::endl;
        return 1;
    }

    std::string urdf_file = argv[1];
    std::string yaml_file = argv[2];
    double tolerance_mm = 25.0;
    std::string section;

    // Parse arguments
    for (int i = 3; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--tolerance" && i + 1 < argc) {
            tolerance_mm = std::stod(argv[++i]);
        } else if (arg == "--section" && i + 1 < argc) {
            section = argv[++i];
        }
    }

    try {
        std::cout << "Loading URDF: " << urdf_file << std::endl;
        Ned2Validator validator(urdf_file);

        std::cout << "Loading YAML: " << yaml_file << std::endl;
        YAML::Node yaml_data = YAML::LoadFile(yaml_file);

        std::cout << "Tolerance: " << tolerance_mm << "mm" << std::endl;

        if (!section.empty()) {
            if (!yaml_data[section]) {
                std::cerr << "Section '" << section << "' not found" << std::endl;
                return 1;
            }

            if (section == "polar_workspace_points") {
                validatePolarWorkspace(validator, yaml_data[section], tolerance_mm);
            } else {
                validateSection(validator, yaml_data[section], section, tolerance_mm);
            }
        } else {
            // Validate all sections
            std::vector<std::string> sections = {"poses", "pick_and_place", "polar_workspace_points"};
            
            for (const auto& sec : sections) {
                if (yaml_data[sec]) {
                    if (sec == "polar_workspace_points") {
                        validatePolarWorkspace(validator, yaml_data[sec], tolerance_mm);
                    } else {
                        validateSection(validator, yaml_data[sec], sec, tolerance_mm);
                    }
                }
            }
        }

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
