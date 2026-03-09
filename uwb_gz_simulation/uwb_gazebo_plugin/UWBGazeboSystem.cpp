#include "UWBGazeboSystem.hpp"

#include <algorithm>
#include <chrono>

#include <gz/common/Console.hh>
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

void UWBGazeboSystem::Configure(const gz::sim::Entity &,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                gz::sim::EntityComponentManager &,
                                gz::sim::EventManager &)
{
  if (_sdf && _sdf->HasElement("gaussian_noise_mean_cm"))
    this->noiseMeanCm_ = _sdf->Get<double>("gaussian_noise_mean_cm");

  if (_sdf && _sdf->HasElement("gaussian_noise_stddev_cm"))
    this->noiseStddevCm_ = _sdf->Get<double>("gaussian_noise_stddev_cm");

  if (_sdf && _sdf->HasElement("bias_min_cm"))
    this->biasMinCm_ = _sdf->Get<double>("bias_min_cm");

  if (_sdf && _sdf->HasElement("bias_max_cm"))
    this->biasMaxCm_ = _sdf->Get<double>("bias_max_cm");

  if (_sdf && _sdf->HasElement("dropout_probability"))
    this->dropoutProbability_ = _sdf->Get<double>("dropout_probability");

  if (_sdf && _sdf->HasElement("apply_pair_bias"))
    this->applyPairBias_ = _sdf->Get<bool>("apply_pair_bias");

  if (this->noiseStddevCm_ < 0.0)
  {
    gzwarn << "[UWBGazeboSystem] gaussian_noise_stddev_cm < 0.0. Clamping to 0.0.\n";
    this->noiseStddevCm_ = 0.0;
  }

  if (this->biasMinCm_ > this->biasMaxCm_)
  {
    gzwarn << "[UWBGazeboSystem] bias_min_cm > bias_max_cm. Swapping values.\n";
    std::swap(this->biasMinCm_, this->biasMaxCm_);
  }

  const bool dropoutWasOutOfRange =
      (this->dropoutProbability_ < 0.0) || (this->dropoutProbability_ > 1.0);
  this->dropoutProbability_ = std::clamp(this->dropoutProbability_, 0.0, 1.0);
  if (dropoutWasOutOfRange)
  {
    gzwarn << "[UWBGazeboSystem] dropout_probability out of [0,1]. Clamped to "
           << this->dropoutProbability_ << ".\n";
  }

  this->noise_dist_ =
      std::normal_distribution<double>(this->noiseMeanCm_, this->noiseStddevCm_);
  this->biasCmDist_ =
      std::uniform_real_distribution<double>(this->biasMinCm_, this->biasMaxCm_);
  this->dropout_flag_ = std::bernoulli_distribution(this->dropoutProbability_);
  this->pairBiasCm_.clear();

  gzmsg << "[UWBGazeboSystem] Sensor model: gaussian_mean_cm=" << this->noiseMeanCm_
        << ", gaussian_stddev_cm=" << this->noiseStddevCm_
        << ", bias_min_cm=" << this->biasMinCm_
        << ", bias_max_cm=" << this->biasMaxCm_
        << ", apply_pair_bias=" << std::boolalpha << this->applyPairBias_
        << ", dropout_probability=" << this->dropoutProbability_ << "\n";
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

void UWBGazeboSystem::PreUpdate(const gz::sim::UpdateInfo &info,
                                gz::sim::EntityComponentManager &ecm)
{

	const auto now = info.simTime;
	const auto dt = now - this->lastUpdateTime_;

	if (dt < std::chrono::milliseconds(100)) return;

	this->lastUpdateTime_ = now;

	std::vector<std::pair<std::string, gz::sim::Entity>> tags;
	std::vector<std::pair<std::string, gz::sim::Entity>> anchors;

	// --- Step 1 & 2: Find all tag and anchor links ---
	ecm.Each<gz::sim::components::Name, gz::sim::components::Link>(
	[&](const gz::sim::Entity &linkEntity,
	const gz::sim::components::Name *linkName,
	const gz::sim::components::Link *) -> bool
	{
		// Find the parent model
		auto parentEntity = ecm.ParentEntity(linkEntity);
		auto parentNameComp = ecm.Component<gz::sim::components::Name>(parentEntity);

		if (!parentNameComp)
		return true;

		const std::string &modelName = parentNameComp->Data();
		const std::string &linkBaseName = linkName->Data();

		// Check for uwb_tag_* in x500_* models
		if (modelName.rfind(modelTagPrefix_, 0) == 0 && linkBaseName.rfind(tagPrefix_, 0) == 0)
		tags.emplace_back(linkBaseName, linkEntity);

		// Check for uwb_anchor_* in r1_rover_* models
		if (modelName.rfind(modelAnchorPrefix_, 0) == 0 && linkBaseName.rfind(anchorPrefix_, 0) == 0)
		anchors.emplace_back(linkBaseName, linkEntity);

		return true;
	});

	// --- Step 3: Compute distance between each anchor-tag pair ---
	for (const auto &[tagName, tagEntity] : tags)
	{
		auto tagPose = computeWorldPose(tagEntity, ecm);

		for (const auto &[anchorName, anchorEntity] : anchors)
		{

			//With a small probability, skip publishing
			if (dropout_flag_(rng_)) {
				continue;
			}

			auto anchorPose = computeWorldPose(anchorEntity, ecm);

			double dist = tagPose.Pos().Distance(anchorPose.Pos()) * 100.0; // Convert to cm
			dist += noise_dist_(rng_);  // Add zero-mean Gaussian noise

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

			// Extract numeric IDs from names (assumes uwb_tag_1, uwb_anchor_2, etc.)
			std::string tagId = tagName.substr(tagName.find_last_of('_') + 1);
			std::string anchorId = anchorName.substr(anchorName.find_last_of('_') + 1);
			std::string topic = "/uwb_gz_simulator/distances/a" + anchorId + "t" + tagId;

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
