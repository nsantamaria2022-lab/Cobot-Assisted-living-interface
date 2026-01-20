// auto-generated DO NOT EDIT

#pragma once

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <mutex>
#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/logger.hpp>
#include <set>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

#include <parameter_traits/parameter_traits.hpp>

#include <rsl/static_string.hpp>
#include <rsl/static_vector.hpp>
#include <rsl/parameter_validators.hpp>



namespace cobot_corrector_params {

// Use validators from RSL
using rsl::unique;
using rsl::subset_of;
using rsl::fixed_size;
using rsl::size_gt;
using rsl::size_lt;
using rsl::not_empty;
using rsl::element_bounds;
using rsl::lower_element_bounds;
using rsl::upper_element_bounds;
using rsl::bounds;
using rsl::lt;
using rsl::gt;
using rsl::lt_eq;
using rsl::gt_eq;
using rsl::one_of;
using rsl::to_parameter_result_msg;

// temporarily needed for backwards compatibility for custom validators
using namespace parameter_traits;

template <typename T>
[[nodiscard]] auto to_parameter_value(T value) {
    return rclcpp::ParameterValue(value);
}

template <size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticString<capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_string(value));
}

template <typename T, size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticVector<T, capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_vector(value));
}
    struct Params {
        std::string cobot_prefix = "";
        struct Transforms {
            std::string aruco_frame = "cobot_calibration_aruco";
            std::string base_frame = "base";
        } transforms;
        struct Subscriptions {
            std::string joint_states = "joint_states";
            std::string robot_description = "robot_description";
            std::string camera_base_topic = "tof/ir/image_raw";
        } subscriptions;
        struct Services {
            std::string execute = "cobot_corrector/execute";
        } services;
        struct Publishers {
            std::string commands = "bias_controller/commands";
        } publishers;
        struct Links {
            std::string base = "base";
            std::string tof = "tof_optical_frame_id";
        } links;
        struct GeneticAlg {
            int64_t generation_size = 1000;
            int64_t max_iterations = 1000;
            double exit_error = 0.018;
            double top_cut = 0.1;
            struct Joints {
                std::vector<std::string> names = {"joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5"};
                std::vector<double> mutation_factor = {0.017, 0.017, 0.034, 0.008, 0.034, 0.034};
            } joints;
            struct AxesWeights {
                double x = 1.0;
                double y = 1.0;
                double z = 0.001;
                double roll = 0.02;
                double pitch = 0.02;
                double yaw = 0.02;
            } axes_weights;
        } genetic_alg;
        // for detecting if the parameter struct has been updated
        rclcpp::Time __stamp;
    };
    struct StackParams {
        struct GeneticAlg {
            int64_t generation_size = 1000;
            int64_t max_iterations = 1000;
            double exit_error = 0.018;
            double top_cut = 0.1;
            struct AxesWeights {
                double x = 1.0;
                double y = 1.0;
                double z = 0.001;
                double roll = 0.02;
                double pitch = 0.02;
                double yaw = 0.02;
            } axes_weights;
        } genetic_alg;
    };

  class ParamListener{
  public:
    // throws rclcpp::exceptions::InvalidParameterValueException on initialization if invalid parameter are loaded
    ParamListener(rclcpp::Node::SharedPtr node, std::string const& prefix = "")
    : ParamListener(node->get_node_parameters_interface(), node->get_logger(), prefix) {}

    ParamListener(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string const& prefix = "")
    : ParamListener(node->get_node_parameters_interface(), node->get_logger(), prefix) {}

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  std::string const& prefix = "")
    : ParamListener(parameters_interface, rclcpp::get_logger("cobot_corrector_params"), prefix) {
      RCLCPP_DEBUG(logger_, "ParameterListener: Not using node logger, recommend using other constructors to use a node logger");
    }

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  rclcpp::Logger logger, std::string const& prefix = "") {
      logger_ = logger;
      prefix_ = prefix;
      if (!prefix_.empty() && prefix_.back() != '.') {
        prefix_ += ".";
      }

      parameters_interface_ = parameters_interface;
      declare_params();
      auto update_param_cb = [this](const std::vector<rclcpp::Parameter> &parameters){return this->update(parameters);};
      handle_ = parameters_interface_->add_on_set_parameters_callback(update_param_cb);
      clock_ = rclcpp::Clock();
    }

    Params get_params() const{
      std::lock_guard<std::mutex> lock(mutex_);
      return params_;
    }

    bool is_old(Params const& other) const {
      std::lock_guard<std::mutex> lock(mutex_);
      return params_.__stamp != other.__stamp;
    }

    StackParams get_stack_params() {
      Params params = get_params();
      StackParams output;
      output.genetic_alg.generation_size = params.genetic_alg.generation_size;
      output.genetic_alg.max_iterations = params.genetic_alg.max_iterations;
      output.genetic_alg.exit_error = params.genetic_alg.exit_error;
      output.genetic_alg.top_cut = params.genetic_alg.top_cut;
      output.genetic_alg.axes_weights.x = params.genetic_alg.axes_weights.x;
      output.genetic_alg.axes_weights.y = params.genetic_alg.axes_weights.y;
      output.genetic_alg.axes_weights.z = params.genetic_alg.axes_weights.z;
      output.genetic_alg.axes_weights.roll = params.genetic_alg.axes_weights.roll;
      output.genetic_alg.axes_weights.pitch = params.genetic_alg.axes_weights.pitch;
      output.genetic_alg.axes_weights.yaw = params.genetic_alg.axes_weights.yaw;

      return output;
    }

    void refresh_dynamic_parameters() {
      auto updated_params = get_params();
      // TODO remove any destroyed dynamic parameters

      // declare any new dynamic parameters
      rclcpp::Parameter param;

    }

    rcl_interfaces::msg::SetParametersResult update(const std::vector<rclcpp::Parameter> &parameters) {
      auto updated_params = get_params();

      for (const auto &param: parameters) {
        if (param.get_name() == (prefix_ + "cobot_prefix")) {
            updated_params.cobot_prefix = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "transforms.aruco_frame")) {
            updated_params.transforms.aruco_frame = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "transforms.base_frame")) {
            updated_params.transforms.base_frame = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "subscriptions.joint_states")) {
            updated_params.subscriptions.joint_states = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "subscriptions.robot_description")) {
            updated_params.subscriptions.robot_description = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "subscriptions.camera_base_topic")) {
            updated_params.subscriptions.camera_base_topic = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "services.execute")) {
            updated_params.services.execute = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "publishers.commands")) {
            updated_params.publishers.commands = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "links.base")) {
            updated_params.links.base = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "links.tof")) {
            updated_params.links.tof = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "genetic_alg.generation_size")) {
            updated_params.genetic_alg.generation_size = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "genetic_alg.max_iterations")) {
            updated_params.genetic_alg.max_iterations = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "genetic_alg.exit_error")) {
            updated_params.genetic_alg.exit_error = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "genetic_alg.top_cut")) {
            updated_params.genetic_alg.top_cut = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "genetic_alg.joints.names")) {
            updated_params.genetic_alg.joints.names = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "genetic_alg.joints.mutation_factor")) {
            updated_params.genetic_alg.joints.mutation_factor = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "genetic_alg.axes_weights.x")) {
            updated_params.genetic_alg.axes_weights.x = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "genetic_alg.axes_weights.y")) {
            updated_params.genetic_alg.axes_weights.y = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "genetic_alg.axes_weights.z")) {
            updated_params.genetic_alg.axes_weights.z = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "genetic_alg.axes_weights.roll")) {
            updated_params.genetic_alg.axes_weights.roll = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "genetic_alg.axes_weights.pitch")) {
            updated_params.genetic_alg.axes_weights.pitch = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "genetic_alg.axes_weights.yaw")) {
            updated_params.genetic_alg.axes_weights.yaw = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
      }

      updated_params.__stamp = clock_.now();
      update_interal_params(updated_params);
      return rsl::to_parameter_result_msg({});
    }

    void declare_params(){
      auto updated_params = get_params();
      // declare all parameters and give default values to non-required ones
      if (!parameters_interface_->has_parameter(prefix_ + "cobot_prefix")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The prefix to attach to parameters for this cobot.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.cobot_prefix);
          parameters_interface_->declare_parameter(prefix_ + "cobot_prefix", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "transforms.aruco_frame")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The frame that defines the calibration aruco board.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.transforms.aruco_frame);
          parameters_interface_->declare_parameter(prefix_ + "transforms.aruco_frame", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "transforms.base_frame")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The name of the frame_id of the base of the COBOT.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.transforms.base_frame);
          parameters_interface_->declare_parameter(prefix_ + "transforms.base_frame", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "subscriptions.joint_states")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The topic to subscribe to for the joint states.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.subscriptions.joint_states);
          parameters_interface_->declare_parameter(prefix_ + "subscriptions.joint_states", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "subscriptions.robot_description")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The topic to subscribe to for the robot description.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.subscriptions.robot_description);
          parameters_interface_->declare_parameter(prefix_ + "subscriptions.robot_description", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "subscriptions.camera_base_topic")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The base topic that the overhead camera publishes to.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.subscriptions.camera_base_topic);
          parameters_interface_->declare_parameter(prefix_ + "subscriptions.camera_base_topic", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "services.execute")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The name of the service that executes cobot correction.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.services.execute);
          parameters_interface_->declare_parameter(prefix_ + "services.execute", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "publishers.commands")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The topic to publish biases to. A position_controllers/JointGroupPositionController subscribes to this topic.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.publishers.commands);
          parameters_interface_->declare_parameter(prefix_ + "publishers.commands", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "links.base")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The name of the base link of the robot.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.links.base);
          parameters_interface_->declare_parameter(prefix_ + "links.base", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "links.tof")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The link that defines the TOF camera's optical frame.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.links.tof);
          parameters_interface_->declare_parameter(prefix_ + "links.tof", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "genetic_alg.generation_size")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Number of configurations to try per generation.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.genetic_alg.generation_size);
          parameters_interface_->declare_parameter(prefix_ + "genetic_alg.generation_size", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "genetic_alg.max_iterations")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Maximum number of iterations to run.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.genetic_alg.max_iterations);
          parameters_interface_->declare_parameter(prefix_ + "genetic_alg.max_iterations", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "genetic_alg.exit_error")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The algorithm will exit if it reaches this error.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.genetic_alg.exit_error);
          parameters_interface_->declare_parameter(prefix_ + "genetic_alg.exit_error", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "genetic_alg.top_cut")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The fraction of configurations to keep between generations.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.genetic_alg.top_cut);
          parameters_interface_->declare_parameter(prefix_ + "genetic_alg.top_cut", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "genetic_alg.joints.names")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The names of the joints of the robot.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.genetic_alg.joints.names);
          parameters_interface_->declare_parameter(prefix_ + "genetic_alg.joints.names", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "genetic_alg.joints.mutation_factor")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Maximum change for each joint between generations (radians).";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.genetic_alg.joints.mutation_factor);
          parameters_interface_->declare_parameter(prefix_ + "genetic_alg.joints.mutation_factor", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "genetic_alg.axes_weights.x")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The weight of the X axis in the error function.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.genetic_alg.axes_weights.x);
          parameters_interface_->declare_parameter(prefix_ + "genetic_alg.axes_weights.x", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "genetic_alg.axes_weights.y")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The weight of the Y axis in the error function.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.genetic_alg.axes_weights.y);
          parameters_interface_->declare_parameter(prefix_ + "genetic_alg.axes_weights.y", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "genetic_alg.axes_weights.z")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The weight of the Z axis in the error function.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.genetic_alg.axes_weights.z);
          parameters_interface_->declare_parameter(prefix_ + "genetic_alg.axes_weights.z", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "genetic_alg.axes_weights.roll")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The weight of the roll axis in the error function.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.genetic_alg.axes_weights.roll);
          parameters_interface_->declare_parameter(prefix_ + "genetic_alg.axes_weights.roll", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "genetic_alg.axes_weights.pitch")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The weight of the pitch axis in the error function.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.genetic_alg.axes_weights.pitch);
          parameters_interface_->declare_parameter(prefix_ + "genetic_alg.axes_weights.pitch", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "genetic_alg.axes_weights.yaw")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The weight of the yaw axis in the error function.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.genetic_alg.axes_weights.yaw);
          parameters_interface_->declare_parameter(prefix_ + "genetic_alg.axes_weights.yaw", parameter, descriptor);
      }
      // get parameters and fill struct fields
      rclcpp::Parameter param;
      param = parameters_interface_->get_parameter(prefix_ + "cobot_prefix");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.cobot_prefix = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "transforms.aruco_frame");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.transforms.aruco_frame = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "transforms.base_frame");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.transforms.base_frame = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "subscriptions.joint_states");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.subscriptions.joint_states = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "subscriptions.robot_description");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.subscriptions.robot_description = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "subscriptions.camera_base_topic");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.subscriptions.camera_base_topic = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "services.execute");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.services.execute = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "publishers.commands");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.publishers.commands = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "links.base");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.links.base = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "links.tof");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.links.tof = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "genetic_alg.generation_size");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.genetic_alg.generation_size = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "genetic_alg.max_iterations");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.genetic_alg.max_iterations = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "genetic_alg.exit_error");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.genetic_alg.exit_error = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "genetic_alg.top_cut");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.genetic_alg.top_cut = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "genetic_alg.joints.names");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.genetic_alg.joints.names = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "genetic_alg.joints.mutation_factor");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.genetic_alg.joints.mutation_factor = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "genetic_alg.axes_weights.x");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.genetic_alg.axes_weights.x = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "genetic_alg.axes_weights.y");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.genetic_alg.axes_weights.y = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "genetic_alg.axes_weights.z");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.genetic_alg.axes_weights.z = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "genetic_alg.axes_weights.roll");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.genetic_alg.axes_weights.roll = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "genetic_alg.axes_weights.pitch");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.genetic_alg.axes_weights.pitch = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "genetic_alg.axes_weights.yaw");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.genetic_alg.axes_weights.yaw = param.as_double();


      updated_params.__stamp = clock_.now();
      update_interal_params(updated_params);
    }

    private:
      void update_interal_params(Params updated_params) {
        std::lock_guard<std::mutex> lock(mutex_);
        params_ = updated_params;
      }

      std::string prefix_;
      Params params_;
      rclcpp::Clock clock_;
      std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> handle_;
      std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface_;

      // rclcpp::Logger cannot be default-constructed
      // so we must provide a initialization here even though
      // every one of our constructors initializes logger_
      rclcpp::Logger logger_ = rclcpp::get_logger("cobot_corrector_params");
      std::mutex mutable mutex_;
  };

} // namespace cobot_corrector_params
