#include "UWBGazeboSystem.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/AxisAlignedBox.hh>
#include <gz/msgs/double.pb.h>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/math/Pose3.hh>
#include <sdf/Element.hh>

using namespace custom;

GZ_ADD_PLUGIN(UWBGazeboSystem,
              gz::sim::System,
              UWBGazeboSystem::ISystemConfigure,
              UWBGazeboSystem::ISystemPreUpdate)

// Helper functions            
namespace
{
constexpr double kPositiveEpsilon = 1e-6;
constexpr double kOrderingEpsilon = 1e-6;

  //
  double exponentialRamp(const double _x,
                        const double _xStart,
                        const double _xEnd,
                        const double _yStart,
                        const double _yEnd)
  {
    if (_x <= _xStart)
      return _yStart;

    if (_x >= _xEnd)
      return _yEnd;

    const double growthRate = std::log(_yEnd / _yStart) / (_xEnd - _xStart);
    const double value = _yStart * std::exp(growthRate * (_x - _xStart));
    return std::clamp(value, std::min(_yStart, _yEnd), std::max(_yStart, _yEnd));
  }

  // Normalized linear mapping between a maximum and a minimum value, returns a value in range [0,1]
  double normalizedRamp(const double _x,
                        const double _xStart,
                        const double _xEnd)
  {
    if (_x <= _xStart)
      return 0.0;

    if (_x >= _xEnd)
      return 1.0;

    return (_x - _xStart) / (_xEnd - _xStart);
  }

  // Combine the LOS distance baseline with the NLOS thickness effect while
  // preserving the remaining headroom. This avoids early saturation and keeps
  // more contrast between LOS / Soft NLOS / Hard NLOS.
  double combineNlosSeverity(const double _distanceSeverity,
                             const double _thicknessSeverity,
                             const double _thicknessExtraWeight)
  {
    const double distanceSeverity = std::clamp(_distanceSeverity, 0.0, 1.0);
    const double weightedThicknessSeverity = std::clamp(
        (1.0 + _thicknessExtraWeight) * _thicknessSeverity,
        0.0,
        1.0);

    return distanceSeverity +
           ((1.0 - distanceSeverity) * weightedThicknessSeverity);
  }

  // Create a topic name based on tag and anchor id pairId (.e.g. a1t2 for anchor 1 - tag2), and a common namespace
  std::string makePairTopic(const std::string &_baseTopic, const std::string &_pairId)
  {
    if (_baseTopic.empty())
      return "/" + _pairId;

    if (_baseTopic.back() == '/')
      return _baseTopic + _pairId;

    return _baseTopic + "/" + _pairId;
  }
}  // namespace

double UWBGazeboSystem::ThicknessSeverity(const double _blockedThicknessM) const
{
  if (_blockedThicknessM <= this->losMaxThicknessM_)
    return 0.0;

  if (_blockedThicknessM >= this->blackoutThicknessM_)
    return 1.0;

  const double softLength = this->softNlosMaxThicknessM_ - this->losMaxThicknessM_;
  const double hardLength = this->blackoutThicknessM_ - this->softNlosMaxThicknessM_;
  const double hardSlopeMultiplier = 1.0 + this->hardNlosGapRatio_;
  const double softSlope = 1.0 / (softLength + (hardSlopeMultiplier * hardLength));
  const double hardSlope = hardSlopeMultiplier * softSlope;

  if (_blockedThicknessM < this->softNlosMaxThicknessM_)
    return (_blockedThicknessM - this->losMaxThicknessM_) * softSlope;

  const double softEndSeverity = softLength * softSlope;
  return softEndSeverity +
         ((_blockedThicknessM - this->softNlosMaxThicknessM_) * hardSlope);
}

double UWBGazeboSystem::ComputeBodyShadowEquivalentThicknessM(
    const double _bodyBlockedThicknessM) const
{
  if (_bodyBlockedThicknessM <= this->losMaxThicknessM_)
    return 0.0;

  return std::min(_bodyBlockedThicknessM, this->softNlosMaxThicknessM_);
}

double UWBGazeboSystem::ComputeDropoutProbability(const double _distanceM,
                                                  const double _blockedThicknessM) const
{
  const double losDropoutProbability = exponentialRamp(_distanceM,
                                                       this->losDropoutStartDistanceM_,
                                                       this->losDropoutEndDistanceM_,
                                                       this->minDropoutProbability_,
                                                       this->maxDropoutProbability_);

  if (_distanceM > this->losHardDropoutDistanceM_)
    return 1.0;

  if (this->enableNlosDropout_ && _blockedThicknessM > this->losMaxThicknessM_)
  {
    if (_blockedThicknessM >= this->blackoutThicknessM_)
      return 1.0;

    const double distanceSeverity = normalizedRamp(_distanceM,
                                                   this->losDropoutStartDistanceM_,
                                                   this->losDropoutEndDistanceM_);
    const double nlosThicknessSeverity = this->ThicknessSeverity(_blockedThicknessM);
    const double combinedSeverity = combineNlosSeverity(
        distanceSeverity,
        nlosThicknessSeverity,
        this->nlosDropoutThicknessExtraWeight_);

    return exponentialRamp(combinedSeverity,
                           0.0,
                           1.0,
                           this->minDropoutProbability_,
                           this->maxDropoutProbability_);
  }

  return losDropoutProbability;
}

double UWBGazeboSystem::ComputeNoiseStddevCm(const double _distanceM,
                                             const double _blockedThicknessM) const
{
  const double distanceSeverity = normalizedRamp(_distanceM,
                                                 this->losStddevStartDistanceM_,
                                                 this->losStddevEndDistanceM_);

  if (_blockedThicknessM > this->losMaxThicknessM_)
  {
    if (_blockedThicknessM >= this->blackoutThicknessM_)
      return this->maxNoiseStddevCm_;

    const double nlosThicknessSeverity = this->ThicknessSeverity(_blockedThicknessM);
    const double combinedSeverity = combineNlosSeverity(
        distanceSeverity,
        nlosThicknessSeverity,
        this->nlosStddevThicknessExtraWeight_);

    return exponentialRamp(combinedSeverity,
                           0.0,
                           1.0,
                           this->minNoiseStddevCm_,
                           this->maxNoiseStddevCm_);
  }

  return exponentialRamp(distanceSeverity,
                         0.0,
                         1.0,
                         this->minNoiseStddevCm_,
                         this->maxNoiseStddevCm_);
}

void UWBGazeboSystem::ValidateAndNormalizeParameters()
{
  if (this->biasMinCm_ > this->biasMaxCm_)
  {
    gzwarn << "[UWBGazeboSystem] bias_min_cm > bias_max_cm. Swapping values.\n";
    std::swap(this->biasMinCm_, this->biasMaxCm_);
  }

  if (this->nlosEndpointMarginM_ < 0.0)
  {
    gzwarn << "[UWBGazeboSystem] nlos_endpoint_margin_m < 0.0. Clamping to 0.0.\n";
    this->nlosEndpointMarginM_ = 0.0;
  }

  auto clampPositive = [](const double value) {
    return std::max(value, kPositiveEpsilon);
  };

  this->minDropoutProbability_ = std::clamp(this->minDropoutProbability_,
                                            kPositiveEpsilon,
                                            1.0);
  this->maxDropoutProbability_ = std::clamp(this->maxDropoutProbability_,
                                            kPositiveEpsilon,
                                            1.0);
  if (this->minDropoutProbability_ > this->maxDropoutProbability_)
  {
    gzwarn << "[UWBGazeboSystem] min_dropout_probability > max_dropout_probability. Swapping values.\n";
    std::swap(this->minDropoutProbability_, this->maxDropoutProbability_);
  }

  this->minNoiseStddevCm_ = clampPositive(this->minNoiseStddevCm_);
  this->maxNoiseStddevCm_ = clampPositive(this->maxNoiseStddevCm_);
  if (this->minNoiseStddevCm_ > this->maxNoiseStddevCm_)
  {
    gzwarn << "[UWBGazeboSystem] min_noise_stddev_cm > max_noise_stddev_cm. Swapping values.\n";
    std::swap(this->minNoiseStddevCm_, this->maxNoiseStddevCm_);
  }

  if (this->losDropoutEndDistanceM_ < this->losDropoutStartDistanceM_)
  {
    gzwarn << "[UWBGazeboSystem] los_dropout_end_distance_m < los_dropout_start_distance_m. Swapping values.\n";
    std::swap(this->losDropoutStartDistanceM_, this->losDropoutEndDistanceM_);
  }

  if (this->losStddevEndDistanceM_ < this->losStddevStartDistanceM_)
  {
    gzwarn << "[UWBGazeboSystem] los_stddev_end_distance_m < los_stddev_start_distance_m. Swapping values.\n";
    std::swap(this->losStddevStartDistanceM_, this->losStddevEndDistanceM_);
  }

  if (this->losHardDropoutDistanceM_ < this->losDropoutEndDistanceM_)
  {
    gzwarn << "[UWBGazeboSystem] los_hard_dropout_distance_m < los_dropout_end_distance_m. Raising to los_dropout_end_distance_m.\n";
    this->losHardDropoutDistanceM_ = this->losDropoutEndDistanceM_;
  }

  this->losMaxThicknessM_ = std::max(this->losMaxThicknessM_, 0.0);
  this->softNlosMaxThicknessM_ =
      std::max(this->softNlosMaxThicknessM_, this->losMaxThicknessM_ + kOrderingEpsilon);
  this->blackoutThicknessM_ =
      std::max(this->blackoutThicknessM_, this->softNlosMaxThicknessM_ + kOrderingEpsilon);

  const bool hardGapWasOutOfRange =
      (this->hardNlosGapRatio_ < 0.0) || (this->hardNlosGapRatio_ > 1.0);
  this->hardNlosGapRatio_ = std::clamp(this->hardNlosGapRatio_, 0.0, 1.0);
  if (hardGapWasOutOfRange)
  {
    gzwarn << "[UWBGazeboSystem] hard_nlos_gap_ratio out of [0,1]. Clamped to "
           << this->hardNlosGapRatio_ << ".\n";
  }

  const bool dropoutThicknessExtraWasOutOfRange =
      (this->nlosDropoutThicknessExtraWeight_ < 0.0) ||
      (this->nlosDropoutThicknessExtraWeight_ > 1.0);
  this->nlosDropoutThicknessExtraWeight_ =
      std::clamp(this->nlosDropoutThicknessExtraWeight_, 0.0, 1.0);
  if (dropoutThicknessExtraWasOutOfRange)
  {
    gzwarn << "[UWBGazeboSystem] nlos_dropout_thickness_extra_weight out of [0,1]. Clamped to "
           << this->nlosDropoutThicknessExtraWeight_ << ".\n";
  }

  const bool stddevThicknessExtraWasOutOfRange =
      (this->nlosStddevThicknessExtraWeight_ < 0.0) ||
      (this->nlosStddevThicknessExtraWeight_ > 1.0);
  this->nlosStddevThicknessExtraWeight_ =
      std::clamp(this->nlosStddevThicknessExtraWeight_, 0.0, 1.0);
  if (stddevThicknessExtraWasOutOfRange)
  {
    gzwarn << "[UWBGazeboSystem] nlos_stddev_thickness_extra_weight out of [0,1]. Clamped to "
           << this->nlosStddevThicknessExtraWeight_ << ".\n";
  }

  if (this->topic_.empty())
  {
    gzwarn << "[UWBGazeboSystem] topic is empty. Falling back to /uwb_gz_simulator/distances.\n";
    this->topic_ = "/uwb_gz_simulator/distances";
  }

  if (this->groundTruthTopic_.empty())
  {
    gzwarn << "[UWBGazeboSystem] ground_truth_topic is empty. Falling back to "
              "/uwb_gz_simulator/distances_ground_truth.\n";
    this->groundTruthTopic_ = "/uwb_gz_simulator/distances_ground_truth";
  }
}

void UWBGazeboSystem::Configure(const gz::sim::Entity &,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                gz::sim::EntityComponentManager &,
                                gz::sim::EventManager &)
{
  if (_sdf && _sdf->HasElement("gaussian_noise_mean_cm"))
    this->noiseMeanCm_ = _sdf->Get<double>("gaussian_noise_mean_cm");

  if (_sdf && _sdf->HasElement("bias_min_cm"))
    this->biasMinCm_ = _sdf->Get<double>("bias_min_cm");

  if (_sdf && _sdf->HasElement("bias_max_cm"))
    this->biasMaxCm_ = _sdf->Get<double>("bias_max_cm");

  if (_sdf && _sdf->HasElement("apply_pair_bias"))
    this->applyPairBias_ = _sdf->Get<bool>("apply_pair_bias");

  if (_sdf && _sdf->HasElement("enable_nlos_dropout"))
    this->enableNlosDropout_ = _sdf->Get<bool>("enable_nlos_dropout");

  if (_sdf && _sdf->HasElement("nlos_endpoint_margin_m"))
    this->nlosEndpointMarginM_ = _sdf->Get<double>("nlos_endpoint_margin_m");

  if (_sdf && _sdf->HasElement("nlos_dropout_thickness_extra_weight"))
  {
    this->nlosDropoutThicknessExtraWeight_ =
        _sdf->Get<double>("nlos_dropout_thickness_extra_weight");
  }

  if (_sdf && _sdf->HasElement("nlos_stddev_thickness_extra_weight"))
  {
    this->nlosStddevThicknessExtraWeight_ =
        _sdf->Get<double>("nlos_stddev_thickness_extra_weight");
  }

  if (_sdf && _sdf->HasElement("los_dropout_start_distance_m"))
    this->losDropoutStartDistanceM_ = _sdf->Get<double>("los_dropout_start_distance_m");

  if (_sdf && _sdf->HasElement("los_dropout_end_distance_m"))
    this->losDropoutEndDistanceM_ = _sdf->Get<double>("los_dropout_end_distance_m");

  if (_sdf && _sdf->HasElement("los_hard_dropout_distance_m"))
    this->losHardDropoutDistanceM_ = _sdf->Get<double>("los_hard_dropout_distance_m");

  if (_sdf && _sdf->HasElement("los_max_thickness_m"))
    this->losMaxThicknessM_ = _sdf->Get<double>("los_max_thickness_m");

  if (_sdf && _sdf->HasElement("soft_nlos_max_thickness_m"))
    this->softNlosMaxThicknessM_ = _sdf->Get<double>("soft_nlos_max_thickness_m");

  if (_sdf && _sdf->HasElement("blackout_thickness_m"))
    this->blackoutThicknessM_ = _sdf->Get<double>("blackout_thickness_m");

  if (_sdf && _sdf->HasElement("hard_nlos_gap_ratio"))
    this->hardNlosGapRatio_ = _sdf->Get<double>("hard_nlos_gap_ratio");

  if (_sdf && _sdf->HasElement("min_dropout_probability"))
    this->minDropoutProbability_ = _sdf->Get<double>("min_dropout_probability");

  if (_sdf && _sdf->HasElement("max_dropout_probability"))
    this->maxDropoutProbability_ = _sdf->Get<double>("max_dropout_probability");

  if (_sdf && _sdf->HasElement("min_noise_stddev_cm"))
    this->minNoiseStddevCm_ = _sdf->Get<double>("min_noise_stddev_cm");

  if (_sdf && _sdf->HasElement("max_noise_stddev_cm"))
    this->maxNoiseStddevCm_ = _sdf->Get<double>("max_noise_stddev_cm");

  if (_sdf && _sdf->HasElement("los_stddev_start_distance_m"))
    this->losStddevStartDistanceM_ = _sdf->Get<double>("los_stddev_start_distance_m");

  if (_sdf && _sdf->HasElement("los_stddev_end_distance_m"))
    this->losStddevEndDistanceM_ = _sdf->Get<double>("los_stddev_end_distance_m");

  if (_sdf && _sdf->HasElement("topic"))
    this->topic_ = _sdf->Get<std::string>("topic");

  if (_sdf && _sdf->HasElement("ground_truth_topic"))
    this->groundTruthTopic_ = _sdf->Get<std::string>("ground_truth_topic");

  this->ValidateAndNormalizeParameters();

  this->biasCmDist_ =
      std::uniform_real_distribution<double>(this->biasMinCm_, this->biasMaxCm_);
  this->pairBiasCm_.clear();
  this->publishers_.clear();
  this->groundTruthPublishers_.clear();

  gzmsg << "[UWBGazeboSystem] Sensor model: gaussian_mean_cm=" << this->noiseMeanCm_
        << ", dropout_probability_range=[" << this->minDropoutProbability_ << ", "
        << this->maxDropoutProbability_ << "]"
        << ", dynamic_stddev_cm=[" << this->minNoiseStddevCm_ << ", "
        << this->maxNoiseStddevCm_ << "]"
        << ", los_dropout_range_m=[" << this->losDropoutStartDistanceM_ << ", "
        << this->losDropoutEndDistanceM_ << "]"
        << ", los_hard_dropout_distance_m=" << this->losHardDropoutDistanceM_
        << ", los_stddev_range_m=[" << this->losStddevStartDistanceM_ << ", "
        << this->losStddevEndDistanceM_ << "]"
        << ", thickness_regimes_m={los<=" << this->losMaxThicknessM_
        << ", soft<=" << this->softNlosMaxThicknessM_
        << ", hard<" << this->blackoutThicknessM_
        << ", blackout>=" << this->blackoutThicknessM_ << "}"
        << ", thickness_severity=piecewise_linear(thickness){hard_slope_multiplier="
        << (1.0 + this->hardNlosGapRatio_) << "}"
        << ", body_shadow_equivalent_thickness_m=min(body_blocked_thickness, soft_nlos_max_thickness_m)"
        << ", bias_min_cm=" << this->biasMinCm_
        << ", bias_max_cm=" << this->biasMaxCm_
        << ", apply_pair_bias=" << std::boolalpha << this->applyPairBias_
        << ", enable_nlos_dropout=" << std::boolalpha << this->enableNlosDropout_
        << ", nlos_endpoint_margin_m=" << this->nlosEndpointMarginM_
        << ", nlos_dropout_thickness_extra_weight="
        << this->nlosDropoutThicknessExtraWeight_
        << ", nlos_stddev_thickness_extra_weight="
        << this->nlosStddevThicknessExtraWeight_
        << ", topic=" << this->topic_
        << ", ground_truth_topic=" << this->groundTruthTopic_ << "\n";
}

gz::math::Pose3d computeWorldPose(const gz::sim::Entity &entity,
                                  gz::sim::EntityComponentManager &ecm)
{
  gz::math::Pose3d pose(gz::math::Vector3d::Zero, gz::math::Quaterniond::Identity);
  gz::sim::Entity current = entity;

  while (current != gz::sim::kNullEntity)
  {
    auto poseComp = ecm.Component<gz::sim::components::Pose>(current);
    if (poseComp)
      pose = poseComp->Data() * pose;

    current = ecm.ParentEntity(current);
  }

  return pose;
}

bool segmentIntersectsAabb(const gz::math::Vector3d &_start,
                           const gz::math::Vector3d &_segment,
                           const gz::math::AxisAlignedBox &_box,
                           double &_tEnter,
                           double &_tExit)
{
  constexpr double kParallelEps = 1e-9;

  double tMin = 0.0;
  double tMax = 1.0;

  const auto &boxMin = _box.Min();
  const auto &boxMax = _box.Max();

  for (int axis = 0; axis < 3; ++axis)
  {
    const double startAxis = _start[axis];
    const double segmentAxis = _segment[axis];

    if (std::abs(segmentAxis) < kParallelEps)
    {
      if (startAxis < boxMin[axis] || startAxis > boxMax[axis])
        return false;
      continue;
    }

    const double invSegmentAxis = 1.0 / segmentAxis;
    double t1 = (boxMin[axis] - startAxis) * invSegmentAxis;
    double t2 = (boxMax[axis] - startAxis) * invSegmentAxis;
    if (t1 > t2)
      std::swap(t1, t2);

    tMin = std::max(tMin, t1);
    tMax = std::min(tMax, t2);

    if (tMax < tMin)
      return false;
  }

  _tEnter = tMin;
  _tExit = tMax;
  return true;
}

double computeAabbBlockedThicknessM(const gz::math::Vector3d &_start,
                                    const gz::math::Vector3d &_end,
                                    const std::vector<gz::sim::Entity> &_allLinks,
                                    const gz::sim::Entity &_tagLinkEntity,
                                    const gz::sim::Entity &_anchorLinkEntity,
                                    const double _endpointMarginM,
                                    gz::sim::EntityComponentManager &_ecm)
{
  const double segmentLength = _start.Distance(_end);
  if (segmentLength <= (2.0 * _endpointMarginM))
    return 0.0;

  const gz::math::Vector3d segment = _end - _start;
  const double tMargin = _endpointMarginM / segmentLength;
  const double tMinAllowed = tMargin;
  const double tMaxAllowed = 1.0 - tMargin;
  if (tMaxAllowed <= tMinAllowed)
    return 0.0;

  std::vector<std::pair<double, double>> blockedIntervals;
  blockedIntervals.reserve(_allLinks.size());

  for (const auto &linkEntity : _allLinks)
  {
    if (linkEntity == _tagLinkEntity || linkEntity == _anchorLinkEntity)
      continue;

    const gz::sim::Link link(linkEntity);
    const auto worldBox = link.WorldAxisAlignedBox(_ecm);
    if (!worldBox.has_value())
      continue;

    double tEnter = 0.0;
    double tExit = 0.0;
    if (!segmentIntersectsAabb(_start, segment, *worldBox, tEnter, tExit))
      continue;

    const double clippedEnter = std::max(tEnter, tMinAllowed);
    const double clippedExit = std::min(tExit, tMaxAllowed);

    if (clippedExit > clippedEnter)
      blockedIntervals.emplace_back(clippedEnter, clippedExit);
  }

  if (blockedIntervals.empty())
    return 0.0;

  std::sort(blockedIntervals.begin(), blockedIntervals.end());

  double blockedFraction = 0.0;
  double currentStart = blockedIntervals.front().first;
  double currentEnd = blockedIntervals.front().second;

  for (size_t i = 1; i < blockedIntervals.size(); ++i)
  {
    const auto &interval = blockedIntervals[i];
    if (interval.first <= currentEnd)
    {
      currentEnd = std::max(currentEnd, interval.second);
      continue;
    }

    blockedFraction += (currentEnd - currentStart);
    currentStart = interval.first;
    currentEnd = interval.second;
  }

  blockedFraction += (currentEnd - currentStart);
  if (blockedFraction <= 0.0)
    return 0.0;

  return blockedFraction * segmentLength;
}

void UWBGazeboSystem::PreUpdate(const gz::sim::UpdateInfo &info,
                                gz::sim::EntityComponentManager &ecm)
{

	const auto now = info.simTime;
	const auto dt = now - this->lastUpdateTime_;

	if (dt < std::chrono::milliseconds(100)) return;

	this->lastUpdateTime_ = now;

	std::vector<std::pair<std::string, gz::sim::Entity>> tags;
	std::vector<std::pair<std::string, gz::sim::Entity>> anchors;
	std::vector<gz::sim::Entity> obstacleLinks;
	std::vector<gz::sim::Entity> bodyShadowLinks;

	// Find all tag and anchor links
	ecm.Each<gz::sim::components::Name, gz::sim::components::Link>(
	[&](const gz::sim::Entity &linkEntity,
	const gz::sim::components::Name *linkName,
	const gz::sim::components::Link *) -> bool
	{
		// Find the parent model
		auto parentEntity = ecm.ParentEntity(linkEntity);
		auto parentNameComp = ecm.Component<gz::sim::components::Name>(parentEntity);

		if (!parentNameComp) return true;

		const std::string &modelName = parentNameComp->Data();
		const std::string &linkBaseName = linkName->Data();

		// Check for uwb_tag_* in x500_* models
		if (modelName.rfind(modelTagPrefix_, 0) == 0 && linkBaseName.rfind(tagPrefix_, 0) == 0)
		tags.emplace_back(linkBaseName, linkEntity);

		// Check for uwb_anchor_* in r1_rover_* models
		if (modelName.rfind(modelAnchorPrefix_, 0) == 0 && linkBaseName.rfind(anchorPrefix_, 0) == 0)
		anchors.emplace_back(linkBaseName, linkEntity);

		// For wall-induced NLOS, consider only static map obstacles.
		const bool isTagModel = (modelName.rfind(modelTagPrefix_, 0) == 0);
		const bool isAnchorModel = (modelName.rfind(modelAnchorPrefix_, 0) == 0);
		const bool isGroundPlane = (modelName == "ground_plane");
		if (!isTagModel && !isAnchorModel && !isGroundPlane)
		{
			obstacleLinks.emplace_back(linkEntity);
		}

		// Add a lightweight body-shadow penalty from the main vehicle bodies only.
		if ((isTagModel || isAnchorModel) && linkBaseName == "base_link")
		{
			bodyShadowLinks.emplace_back(linkEntity);
		}

		return true;
	});

	// Ensure world AABB is available for LOS/NLOS checks.
	for (const auto &linkEntity : obstacleLinks)
	{
		gz::sim::Link(linkEntity).EnableBoundingBoxChecks(ecm, true);
	}
	for (const auto &linkEntity : bodyShadowLinks)
	{
		gz::sim::Link(linkEntity).EnableBoundingBoxChecks(ecm, true);
	}

	// Compute distance between each anchor-tag pair 
	for (const auto &[tagName, tagEntity] : tags)
	{
		auto tagPose = computeWorldPose(tagEntity, ecm);

    // Compute per-pair dropout from LOS distance or NLOS blocked thickness.
		for (const auto &[anchorName, anchorEntity] : anchors)
		{
			auto anchorPose = computeWorldPose(anchorEntity, ecm);
			std::string tagId = tagName.substr(tagName.find_last_of('_') + 1);
			std::string anchorId = anchorName.substr(anchorName.find_last_of('_') + 1);
			std::string pairId = "a" + anchorId + "t" + tagId;

			const double wallBlockedThicknessM =
				computeAabbBlockedThicknessM(tagPose.Pos(),
											 anchorPose.Pos(),
											 obstacleLinks,
											 tagEntity,
											 anchorEntity,
											 this->nlosEndpointMarginM_,
											 ecm);
			const double bodyShadowBlockedThicknessM =
				computeAabbBlockedThicknessM(tagPose.Pos(),
											 anchorPose.Pos(),
											 bodyShadowLinks,
											 tagEntity,
											 anchorEntity,
											 this->nlosEndpointMarginM_,
											 ecm);
			const double effectiveBlockedThicknessM =
				wallBlockedThicknessM +
				this->ComputeBodyShadowEquivalentThicknessM(bodyShadowBlockedThicknessM);

			const double distanceM = tagPose.Pos().Distance(anchorPose.Pos());
			const double groundTruthDistCm = distanceM * 100.0;
			const double dropoutProbability =
				this->ComputeDropoutProbability(distanceM, effectiveBlockedThicknessM);
			const double noiseStddevCm =
				this->ComputeNoiseStddevCm(distanceM, effectiveBlockedThicknessM);

			if (wallBlockedThicknessM > 0.0 || bodyShadowBlockedThicknessM > 0.0)
			{
				gzmsg << "[UWBGazeboSystem] wall_blocked_thickness_m=" << wallBlockedThicknessM
				      << ", body_shadow_blocked_thickness_m=" << bodyShadowBlockedThicknessM
				      << ", effective_blocked_thickness_m=" << effectiveBlockedThicknessM
				      << " pair=" << pairId << "\n";
			}

			const std::string groundTruthTopic = makePairTopic(this->groundTruthTopic_, pairId);
			if (this->groundTruthPublishers_.find(groundTruthTopic) == this->groundTruthPublishers_.end())
			{
				this->groundTruthPublishers_[groundTruthTopic] =
					this->node_.Advertise<gz::msgs::Double>(groundTruthTopic);
			}

			gz::msgs::Double groundTruthMsg;
			groundTruthMsg.set_data(groundTruthDistCm);
			this->groundTruthPublishers_[groundTruthTopic].Publish(groundTruthMsg);

			if (std::bernoulli_distribution(dropoutProbability)(this->rng_))
			{
				continue;
			}

			const bool isNlos = (effectiveBlockedThicknessM > this->losMaxThicknessM_);
			double noiseCm =
				std::normal_distribution<double>(this->noiseMeanCm_, noiseStddevCm)(this->rng_);
			if (isNlos && noiseCm < 0.0)
			{
				noiseCm = -noiseCm;
			}

			double dist = groundTruthDistCm;
			dist += noiseCm;

			// Build stable key per anchor–tag pair
			const std::string key = "a" + anchorName + "t" + tagName;

			if (this->applyPairBias_)
			{
				// Draw and cache a persistent bias for this pair (in cm)
				auto it = this->pairBiasCm_.find(key);
				if (it == this->pairBiasCm_.end())
				{
					it = this->pairBiasCm_.emplace(key, this->biasCmDist_(this->rng_)).first;
				}
				dist += it->second;
			}

			// Keep distance physically valid
			if (dist < 0.0) dist = 0.0;

			const std::string topic = makePairTopic(this->topic_, pairId);

			// Create publisher if not already existing
			if (this->publishers_.find(topic) == this->publishers_.end())
			{
				this->publishers_[topic] = this->node_.Advertise<gz::msgs::Double>(topic);
			}

			gz::msgs::Double msg;
			msg.set_data(dist);
			this->publishers_[topic].Publish(msg);
		}
	}
}
