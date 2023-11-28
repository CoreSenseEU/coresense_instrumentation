#ifndef ICELL_HPP
#define ICELL_HPP

#include "rclcpp/rclcpp.hpp"

// Esta es una clase base abstracta que define la interfaz común de los nodos de instrumentación
class ICell
{
public:
  // Este es un método virtual puro que debe ser implementado por las clases derivadas
  virtual void process() = 0;

  // Este es un destructor virtual para permitir el polimorfismo
  virtual ~ICell() {}

  void set_name(const std::string & name)
  {
    name_ = name;
  }

  void set_topic(const std::string & topic)
  {
    topic_ = topic;
  }

  void set_node_base_interface(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface)
  {
    node_base_interface_ = node_base_interface;
  }

  std::string get_name()
  {
    return name_;
  }

  std::string get_topic()
  {
    return topic_;
  }


  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_base_interface()
  {
    return node_base_interface_;
  }

private:
  std::string name_;
  std::string topic_;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
};

#endif // ICELL_HPP
