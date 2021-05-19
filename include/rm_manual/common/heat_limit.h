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
    switch (t) {
      case ID1_17MM: {
        bullet_heat_ = 10.;
        cooling_limit_ = &referee->referee_data_.game_robot_status_.shooter_id1_17mm_cooling_limit;
        cooling_rate_ = &referee->referee_data_.game_robot_status_.shooter_id1_17mm_cooling_rate;
        cooling_limit_ = &referee->referee_data_.power_heat_data_.shooter_id1_17mm_cooling_heat;
        break;
      }
      case ID2_17MM: {
        cooling_limit_ = &referee->referee_data_.game_robot_status_.shooter_id2_17mm_cooling_limit;
        cooling_rate_ = &referee->referee_data_.game_robot_status_.shooter_id2_17mm_cooling_rate;
        cooling_limit_ = &referee->referee_data_.power_heat_data_.shooter_id2_17mm_cooling_heat;
        break;
      }
      case ID1_42MM:bullet_heat_ = 100.;
        cooling_limit_ = &referee->referee_data_.game_robot_status_.shooter_id1_42mm_cooling_limit;
        cooling_rate_ = &referee->referee_data_.game_robot_status_.shooter_id1_42mm_cooling_rate;
        cooling_limit_ = &referee->referee_data_.power_heat_data_.shooter_id1_42mm_cooling_heat;
        break;
    }
    if (nh.getParam("safe_shoot_frequency", safe_shoot_frequency_))
      ROS_ERROR("Safe shoot frequency no defined (namespace: %s)", nh.getNamespace().c_str());
  }
 private:
  double bullet_heat_, safe_shoot_frequency_{};
  uint16_t *cooling_limit_{}, *cooling_rate_{}, *shooter_heat_{};
};

}
#endif //RM_MANUAL_COMMON_HEAT_LIMIT_H_
