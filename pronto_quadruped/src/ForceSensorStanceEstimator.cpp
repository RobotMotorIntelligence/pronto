#include "pronto_quadruped/ForceSensorStanceEstimator.hpp"

using namespace pronto::quadruped;



bool ForceSensorStanceEstimator::getStance(LegBoolMap& stance,
                                           LegScalarMap& stance_probability) {

  stance[LF] = grf_[LF].norm() > force_threshold_;
  stance[RF] = grf_[RF].norm() > force_threshold_;
  stance[LH] = grf_[LH].norm() > force_threshold_;
  stance[RH] = grf_[RH].norm() > force_threshold_;

  stance_probability[LF] = static_cast<double>(stance[LF]);
  stance_probability[RF] = static_cast<double>(stance[RF]);
  stance_probability[LH] = static_cast<double>(stance[LH]);
  stance_probability[RH] = static_cast<double>(stance[RH]);

  return true;

}

bool ForceSensorStanceEstimator::getStance(LegBoolMap &stance) {
  LegScalarMap stance_probability;
  return getStance(stance, stance_probability);
}

bool ForceSensorStanceEstimator::getGRF(LegVectorMap& grf) {
  grf = grf_;
  return true;
}

void ForceSensorStanceEstimator::setGRF(const LegVectorMap& grf) {
  grf_ = grf;
}

bool ForceSensorStanceEstimator::isStance(LegID leg) const {
  return stance_[leg];
}
