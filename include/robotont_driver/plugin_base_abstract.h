#ifndef PLUGIN_BASE_ABSTRACT
#define PLUGIN_BASE_ABSTRACT

#include <robotont_driver/hardware.h>
#include <memory>

namespace robotont
{
class PluginBaseAbstract
{
public:

  virtual ~PluginBaseAbstract(){};

  virtual void initialize(HardwarePtr hw_ptr, const std::string& name) = 0;
  virtual void packetReceived(const RobotontPacket& packet) = 0;
  //virtual const std::string& getName() const ;

protected:
  HardwarePtr hw_ptr_;
  std::string name_;
};

typedef std::shared_ptr<PluginBaseAbstract> PluginBaseAbstractPtr;
}  // namespace robotont
#endif
