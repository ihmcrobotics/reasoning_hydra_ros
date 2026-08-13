#include "hydra_ros/hydra_ros_pipeline.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <hydra/backend/backend_module.h>
#include <hydra/backend/update_frontiers_functor.h>
#include <hydra/backend/update_surface_places_functor.h>
#include <hydra/common/dsg_types.h>
#include <hydra/common/global_info.h>
#include <hydra/frontend/frontend_module.h>
#include <hydra/loop_closure/loop_closure_module.h>
#include <hydra/reconstruction/reconstruction_module.h>

#include <memory>

#include "hydra_ros/backend/ros_backend_publisher.h"
#include "hydra_ros/backend/ros_vlm_relationships.h"
#include "hydra_ros/navigation/ros_navigation_interface.h"

namespace hydra {

void declare_config(HydraRosConfig& config) {
  using namespace config;
  name("HydraRosConfig");
  field(config.enable_frontend_output, "enable_frontend_output");
  field(config.enable_reasoning, "enable_reasoning");
  field(config.active_object_edges_topic, "active_object_edges_topic");
  field(config.vlm_relationship_service, "vlm_relationship_service");
  field(config.vlm_encodings_topic, "vlm_encodings_topic");
  field(config.vlm_labels_topic, "vlm_labels_topic");
  field(config.input, "input");
}

HydraRosPipeline::HydraRosPipeline(int robot_id, int config_verbosity)
    : HydraPipeline(config::fromContext<PipelineConfig>(),
                    robot_id,
                    config_verbosity),
      config_(config::checkValid(config::fromContext<HydraRosConfig>())),
      nh_(ianvs::NodeHandle::this_node("~")) {
  LOG_IF(INFO, config_verbosity >= 1)
      << "Starting "
      << (config_.enable_reasoning ? "reasoning-enabled" : "standard")
      << " Hydra ROS 2 with input configuration\n"
      << config::toString(config_.input);
}

HydraRosPipeline::~HydraRosPipeline() = default;

void HydraRosPipeline::init() {
  const auto& pipeline_config = GlobalInfo::instance().getConfig();
  initFrontend();
  initBackend();
  initReconstruction();
  initNavigation();
  if (pipeline_config.enable_lcd) {
    initLCD();
  }

  const auto reconstruction = getModule<ReconstructionModule>("reconstruction");
  CHECK(reconstruction) << "Reconstruction module is required by RosInputModule";
  input_module_ =
      std::make_shared<RosInputModule>(config_.input, reconstruction->queue());
}

void HydraRosPipeline::initFrontend() {
  const auto logs = GlobalInfo::instance().getLogs();
  auto frontend = config::createFromContextWithNamespace<FrontendModule>(
      "frontend", frontend_dsg_, shared_state_, logs);
  CHECK(frontend) << "Failed to construct frontend";
  if (config_.enable_frontend_output) {
    LOG(WARNING) << "Frontend ROS output is configured but its ROS 2 publisher "
                    "is not ported yet";
  }
  modules_["frontend"] = std::shared_ptr<FrontendModule>(std::move(frontend));
}

void HydraRosPipeline::initBackend() {
  const auto logs = GlobalInfo::instance().getLogs();
  auto backend = config::createFromContextWithNamespace<BackendModule>(
      "backend", backend_dsg_, shared_state_, logs);
  CHECK(backend) << "Failed to construct backend";
  auto backend_ptr = std::shared_ptr<BackendModule>(std::move(backend));
  modules_["backend"] = backend_ptr;

  if (config_.enable_reasoning) {
    ros2_backend_publisher_ = std::make_shared<Ros2BackendPublisher>(
        nh_, config_.active_object_edges_topic);
    backend_ptr->addSink(ros2_backend_publisher_);
    LOG(INFO) << "Publishing active object relationships on "
              << config_.active_object_edges_topic;

    shared_state_->vlm_labels_queue =
        std::make_shared<InputQueue<BackendVLMLabelsInput::Ptr>>();
    Ros2VLMRelationships::Config vlm_config;
    vlm_config.service = config_.vlm_relationship_service;
    vlm_config.encodings_topic = config_.vlm_encodings_topic;
    vlm_config.labels_topic = config_.vlm_labels_topic;
    ros2_vlm_relationships_ = std::make_shared<Ros2VLMRelationships>(
        nh_, vlm_config, shared_state_->vlm_labels_queue);
    backend_ptr->addSink(ros2_vlm_relationships_);
    LOG(INFO) << "VLM relationship bridge listening on "
              << config_.vlm_relationship_service;
  }

  if (backend_ptr->config.use_vlm) {
    LOG(WARNING) << "VLM relationships are enabled, but the ROS 2 VLM adapter "
                    "is not ported yet";
  }

  const auto frontend = getModule<FrontendModule>("frontend");
  CHECK(frontend);
  if (frontend->config.surface_places) {
    backend_ptr->setUpdateFunctor(
        DsgLayers::MESH_PLACES,
        std::make_shared<Update2dPlacesFunctor>(
            backend_ptr->config.places2d_config));
  }
  if (frontend->config.use_frontiers && frontend->config.frontier_places) {
    backend_ptr->setUpdateFunctor(
        DsgLayers::BUILDINGS + 1,
        std::make_shared<UpdateFrontiersFunctor>(
            backend_ptr->config.frontier_config));
  }
}

void HydraRosPipeline::initReconstruction() {
  const auto frontend = getModule<FrontendModule>("frontend");
  CHECK(frontend) << "Frontend is required by reconstruction";
  auto reconstruction =
      config::createFromContextWithNamespace<ReconstructionModule>(
          "reconstruction", frontend->getQueue());
  CHECK(reconstruction) << "Failed to construct reconstruction";
  modules_["reconstruction"] =
      std::shared_ptr<ReconstructionModule>(std::move(reconstruction));
}

void HydraRosPipeline::initLCD() {
  auto config = config::fromContext<LoopClosureConfig>("lcd");
  config.detector.num_semantic_classes =
      GlobalInfo::instance().getTotalLabels();
  config::checkValid(config);
  shared_state_->lcd_queue = std::make_shared<InputQueue<LcdInput::Ptr>>();
  modules_["lcd"] = std::make_shared<LoopClosureModule>(config, shared_state_);
}

void HydraRosPipeline::initNavigation() {
  if (!config_.enable_reasoning) {
    return;
  }

  auto navigation =
      std::make_shared<NavigationModule>(NavigationModule::Config{});
  auto object_search_config = config::checkValid(
      config::fromContext<ObjectSearchModule::Config>("object_search"));
  auto object_search =
      std::make_shared<ObjectSearchModule>(object_search_config);

  ros2_navigation_interface_ = std::make_shared<Ros2NavigationInterface>(
      nh_,
      Ros2NavigationInterface::Config{},
      backend_dsg_,
      navigation,
      object_search);
  modules_["navigation"] = navigation;
  modules_["object_search"] = object_search;
  LOG(INFO) << "Navigation bridge listening on "
            << Ros2NavigationInterface::Config{}.find_paths_service;
}

void HydraRosPipeline::loadGraph(const std::string& filepath) {
  auto graph = DynamicSceneGraph::load(filepath);
  auto backend = getModule<BackendModule>("backend");
  auto frontend = getModule<FrontendModule>("frontend");
  if (!graph || !backend || !frontend) {
    LOG(ERROR) << "Failed to load graph or invalid modules";
    return;
  }
  backend->setGraph(graph);
  frontend->setGraph(graph->clone());
}

}  // namespace hydra
