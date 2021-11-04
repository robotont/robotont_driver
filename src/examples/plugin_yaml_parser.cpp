#include <iostream>
#include "yaml-cpp/yaml.h"
#include <fstream>

int main(int argc, char** argv)
{
  std::string yaml_file_path = argv[1];
  std::ifstream yaml_fstream(yaml_file_path);
  YAML::Node config = YAML::Load(yaml_fstream);
  std::map<std::string, YAML::Node> plugin_configs_parsed;

  std::cout << "plugin_config path: " << yaml_file_path << std::endl;
  YAML::Node plugins_node = config["plugins"];

  if (!plugins_node)
  {
    std::cout << "The plugin config file has no plugins specified" << std::endl;
    return 1;
  }
  
  if (!plugins_node.IsSequence())
  {
    std::cout << "problems 1" << std::endl;
    return 1;
  }

  // Get the plugin names and corresponding configurations
  for (const auto& plugin_node : plugins_node)
  {
    std::string plugin_name = plugin_node.begin()->first.as<std::string>();
    YAML::Node plugin_config = plugin_node.begin()->second;

    if (plugin_name.empty())
    {
      std::cout << "Plugin name not specified" << std::endl;
      return 1;
    }

    if (!plugin_config.IsMap())
    {
      std::cout << "Improper format of plugin: " << plugin_name << std::endl;
      return 1;
    }

    plugin_configs_parsed.insert({plugin_name, plugin_config});
  }

  // Print out the plugin configs
  for (const auto& plugin_config : plugin_configs_parsed)
  {
    std::cout << "plugin: " << plugin_config.first << std::endl;
    std::cout << plugin_config.second << std::endl;
  }

  return 0;
}