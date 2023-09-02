/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-25 09:52:30
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-02 09:10:18
 */
#include <iostream>

#include "demo.h"

using namespace std;

int main(int argc, char **argv) {
  std::string config_path;
  if (argc > 1) {
    config_path = argv[1];
  }
  std::cout << "---------" << std::endl;
  ros::init(argc, argv, "Demo");
  Demo demo(config_path);
  if (!demo.Init()) {
    return 1;
  }
  demo.Run();

  return 0;
}
