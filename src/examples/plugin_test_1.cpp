#include "class_loader/class_loader.hpp"
#include "robotont_driver/plugin_base_abstract.h"

namespace robotont
{
class PluginTest1 : public PluginBaseAbstract
{
public:
  PluginTest1(){};

  virtual void initialize(HardwarePtr hw_ptr, const std::string& name)
  {

  }

  virtual void packetReceived(const RobotontPacket& packet)
  {

  }
};
} // robotont namespace

CLASS_LOADER_REGISTER_CLASS(robotont::PluginTest1, robotont::PluginBaseAbstract)