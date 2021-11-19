#include <iostream>
#include "class_loader/multi_library_class_loader.hpp"
#include "robotont_driver/plugin_base_abstract.h"

int main(int argc, char** argv)
{
  class_loader::MultiLibraryClassLoader loader(true);

  loader.loadLibrary("LIBRARY_1");
  loader.loadLibrary("LIBRARY_2");

  robotont::PluginBaseAbstractPtr plugin_test_1 = loader.createSharedInstance<robotont::PluginBaseAbstract>("PluginTest1");
  robotont::PluginBaseAbstractPtr plugin_test_2 = loader.createSharedInstance<robotont::PluginBaseAbstract>("PluginTest2");
}