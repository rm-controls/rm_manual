//
// Created by qiayuan on 5/19/21.
//

#ifndef RM_MANUAL_COMMON_HEAT_LIMIT_H_
#define RM_MANUAL_COMMON_HEAT_LIMIT_H_
namespace rm_manual {
class HeatLimit {
 public:
  typedef enum { ID1_17MM, ID2_17MM, ID1_42MM } Type;
  HeatLimit(ros::NodeHandle &nh, Type type, const Referee &referee) : type_(type), referee_(referee) {
    if (type > ID2_17MM)
      bullet_heat_ = 100.;
    else
      bullet_heat_ = 10.;

    if (nh.getParam("safe_shoot_frequency", safe_shoot_frequency_))
      ROS_ERROR("Safe shoot frequency no defined (namespace: %s)", nh.getNamespace().c_str());
    if (nh.getParam("heat_coeff", heat_coeff_))
      ROS_ERROR("Safe shoot heat coeff frequency no defined (namespace: %s)", nh.getNamespace().c_str());
  }

  double getHz(double expect_hz) const {
    if (!referee_.is_open_) return safe_shoot_frequency_;
    double cooling_limit, cooling_rate, cooling_heat;
    switch (type_) {
      case ID1_17MM: {
        cooling_limit = referee_.referee_data_.game_robot_status_.shooter_id1_17mm_cooling_limit;
        cooling_rate = referee_.referee_data_.game_robot_status_.shooter_id1_17mm_cooling_rate;
        cooling_heat = referee_.referee_data_.power_heat_data_.shooter_id1_17mm_cooling_heat;
        break;
      }
      case ID2_17MM: {
        cooling_limit = referee_.referee_data_.game_robot_status_.shooter_id2_17mm_cooling_limit;
        cooling_rate = referee_.referee_data_.game_robot_status_.shooter_id2_17mm_cooling_rate;
        cooling_heat = referee_.referee_data_.power_heat_data_.shooter_id2_17mm_cooling_heat;
        break;
      }
      case ID1_42MM: {
        cooling_limit = referee_.referee_data_.game_robot_status_.shooter_id1_42mm_cooling_limit;
        cooling_rate = referee_.referee_data_.game_robot_status_.shooter_id1_42mm_cooling_rate;
        cooling_heat = referee_.referee_data_.power_heat_data_.shooter_id1_42mm_cooling_heat;
        break;
      }
    }
    if (cooling_heat < cooling_limit - bullet_heat_ * heat_coeff_)
      return expect_hz;
    else if (cooling_heat >= cooling_limit)
      return 0.0;
    else
      return cooling_rate / cooling_heat;
  }

 private:
  Type type_;
  const Referee &referee_;
  double bullet_heat_, safe_shoot_frequency_{}, heat_coeff_{};
};

}
#endif //RM_MANUAL_COMMON_HEAT_LIMIT_H_
