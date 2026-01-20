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



namespace aruco_transforms_params {

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
        std::string camera_base_topic = "kinect2/color/image_raw";
        struct Chessboard {
            std::string frame = "chessboard_frame_dynamic";
            struct Warped {
                std::string topic = "chessboard/image_raw";
                int64_t size = 512;
            } warped;
        } chessboard;
        struct Table {
            std::string frame = "table_frame";
            struct Warped {
                std::string topic = "table/image_raw";
                int64_t width = 1024;
            } warped;
        } table;
        struct Cobot0Eef {
            std::string pose_topic = "cobot0/eef_pose";
        } cobot0_eef;
        // for detecting if the parameter struct has been updated
        rclcpp::Time __stamp;
    };
    struct StackParams {
        struct Chessboard {
            struct Warped {
                int64_t size = 512;
            } warped;
        } chessboard;
        struct Table {
            struct Warped {
                int64_t width = 1024;
            } warped;
        } table;
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
    : ParamListener(parameters_interface, rclcpp::get_logger("aruco_transforms_params"), prefix) {
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
      output.chessboard.warped.size = params.chessboard.warped.size;
      output.table.warped.width = params.table.warped.width;

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
        if (param.get_name() == (prefix_ + "camera_base_topic")) {
            updated_params.camera_base_topic = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "chessboard.frame")) {
            updated_params.chessboard.frame = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "chessboard.warped.topic")) {
            updated_params.chessboard.warped.topic = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "chessboard.warped.size")) {
            updated_params.chessboard.warped.size = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "table.frame")) {
            updated_params.table.frame = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "table.warped.topic")) {
            updated_params.table.warped.topic = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "table.warped.width")) {
            updated_params.table.warped.width = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "cobot0_eef.pose_topic")) {
            updated_params.cobot0_eef.pose_topic = param.as_string();
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
      if (!parameters_interface_->has_parameter(prefix_ + "camera_base_topic")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The base topic of the camera publisher that contains the chessboard image.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.camera_base_topic);
          parameters_interface_->declare_parameter(prefix_ + "camera_base_topic", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "chessboard.frame")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The TF2 frame_id of the chessboard.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.chessboard.frame);
          parameters_interface_->declare_parameter(prefix_ + "chessboard.frame", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "chessboard.warped.topic")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The base topic to publish warped images of the chessboard to.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.chessboard.warped.topic);
          parameters_interface_->declare_parameter(prefix_ + "chessboard.warped.topic", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "chessboard.warped.size")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The size of the warped chessboard image in pixels.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.chessboard.warped.size);
          parameters_interface_->declare_parameter(prefix_ + "chessboard.warped.size", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "table.frame")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The TF2 frame_id of the table.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.table.frame);
          parameters_interface_->declare_parameter(prefix_ + "table.frame", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "table.warped.topic")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The base topic to publish warped images of the table to.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.table.warped.topic);
          parameters_interface_->declare_parameter(prefix_ + "table.warped.topic", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "table.warped.width")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The width of the warped table image in pixels.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.table.warped.width);
          parameters_interface_->declare_parameter(prefix_ + "table.warped.width", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "cobot0_eef.pose_topic")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The topic to publish the pose of cobot0's end effector to.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.cobot0_eef.pose_topic);
          parameters_interface_->declare_parameter(prefix_ + "cobot0_eef.pose_topic", parameter, descriptor);
      }
      // get parameters and fill struct fields
      rclcpp::Parameter param;
      param = parameters_interface_->get_parameter(prefix_ + "camera_base_topic");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.camera_base_topic = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "chessboard.frame");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.chessboard.frame = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "chessboard.warped.topic");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.chessboard.warped.topic = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "chessboard.warped.size");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.chessboard.warped.size = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "table.frame");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.table.frame = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "table.warped.topic");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.table.warped.topic = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "table.warped.width");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.table.warped.width = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "cobot0_eef.pose_topic");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.cobot0_eef.pose_topic = param.as_string();


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
      rclcpp::Logger logger_ = rclcpp::get_logger("aruco_transforms_params");
      std::mutex mutable mutex_;
  };

} // namespace aruco_transforms_params
