//
// Created by qiayuan on 5/19/21.
//

#ifndef RM_MANUAL_COMMON_HEAT_LIMIT_H_
#define RM_MANUAL_COMMON_HEAT_LIMIT_H_
namespace rm_manual {
class HeatLimit {
 public:
  typedef enum { ID1_17MM, ID2_17MM, ID1_42MM } type;
  HeatLimit(ros::NodeHandle &nh, type t, Referee *referee) {
    bullet_heat_ = 10.;
    switch (t) {
      case ID1_17MM: {
        cooling_limit_ = &referee->referee_data_.game_robot_status_.shooter_id1_17mm_cooling_limit;
        cooling_rate_ = &referee->referee_data_.game_robot_status_.shooter_id1_17mm_cooling_rate;
        cooling_heat_ = &referee->referee_data_.power_heat_data_.shooter_id1_17mm_cooling_heat;
        break;
      }
      case ID2_17MM: {
        cooling_limit_ = &referee->referee_data_.game_robot_status_.shooter_id2_17mm_cooling_limit;
        cooling_rate_ = &referee->referee_data_.game_robot_status_.shooter_id2_17mm_cooling_rate;
        cooling_heat_ = &referee->referee_data_.power_heat_data_.shooter_id2_17mm_cooling_heat;
        break;
      }
      case ID1_42MM: {
        bullet_heat_ = 100.;
        cooling_limit_ = &referee->referee_data_.game_robot_status_.shooter_id1_42mm_cooling_limit;
        cooling_rate_ = &referee->referee_data_.game_robot_status_.shooter_id1_42mm_cooling_rate;
        cooling_heat_ = &referee->referee_data_.power_heat_data_.shooter_id1_42mm_cooling_heat;
        break;
      }
    }
    if (nh.getParam("safe_shoot_frequency", safe_shoot_frequency_))
      ROS_ERROR("Safe shoot frequency no defined (namespace: %s)", nh.getNamespace().c_str());
    if (nh.getParam("heat_coeff", heat_coeff_))
      ROS_ERROR("Safe shoot heat coeff frequency no defined (namespace: %s)", nh.getNamespace().c_str());
  }

  double getHz(double expect_hz) const {
    double cooling_limit = static_cast<double>(*cooling_limit_);
    double cooling_rate = static_cast<double>(*cooling_rate_);
    double cooling_heat = static_cast<double>(*cooling_heat_);

    if (cooling_heat < cooling_limit - bullet_heat_ * heat_coeff_)
      return expect_hz;
    else if (cooling_heat >= cooling_limit)
      return 0.0;
    else
      return cooling_rate / cooling_heat;
  }

 private:
  double bullet_heat_, safe_shoot_frequency_{}, heat_coeff_{};
  uint16_t *cooling_limit_{}, *cooling_rate_{}, *cooling_heat_{};
};

}
#endif //RM_MANUAL_COMMON_HEAT_LIMIT_H_
