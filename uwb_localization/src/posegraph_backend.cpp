#include "uwb_localization/posegraph_backend.hpp"

#include <stdexcept>

#include "uwb_localization/ceres_backend.hpp"
#ifdef UWB_LOCALIZATION_WITH_GTSAM
#include "uwb_localization/gtsam_backend.hpp"
#endif

namespace uwb_localization {

std::unique_ptr<PoseGraphBackend> makePoseGraphBackend(const std::string& name,
                                                        const PoseGraphBackendOptions& options,
                                                        const rclcpp::Logger& logger) {
  if (name == "ceres") return std::make_unique<CeresBackend>(options, logger);
  if (name == "gtsam") {
#ifdef UWB_LOCALIZATION_WITH_GTSAM
    return std::make_unique<GtsamBackend>(options, logger);
#else
    throw std::invalid_argument("pose-graph backend 'gtsam' requested, but uwb_localization was built "
                                "without GTSAM (build GTSAM >= 4.3 and rebuild)");
#endif
  }
  throw std::invalid_argument("unknown pose-graph backend '" + name + "' (expected 'ceres' or 'gtsam')");
}

} // namespace uwb_localization
