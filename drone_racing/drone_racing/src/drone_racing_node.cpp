#include <rpg_common/main.h>

#include "drone_racing/drone_racing.h"

RPG_COMMON_MAIN {
  ros::init(argc, argv, "drone_racing");

  drone_racing::DroneRacing drone_racing;

  ros::MultiThreadedSpinner spinner;
  spinner.spin();
  return 0;
}
