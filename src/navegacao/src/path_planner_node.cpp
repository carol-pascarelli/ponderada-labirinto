#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "navegacao/grid_utils.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class PathPlannerNode : public rclcpp::Node
{
public:
  PathPlannerNode()
  : rclcpp::Node("path_planner")
  {
    get_map_client_ = this->create_client<cg_interfaces::srv::GetMap>("get_map");
    move_client_ = this->create_client<cg_interfaces::srv::MoveCmd>("move_command");
  }

  void run()
  {
    if (plan_executed_) {
      return;
    }

    // Aguarda serviços ficarem disponíveis
    while (rclcpp::ok() &&
           (!get_map_client_->wait_for_service(1s) ||
            !move_client_->wait_for_service(1s))) {
      RCLCPP_INFO(get_logger(), "Aguardando os serviços get_map e move_command ficarem disponíveis...");
    }

    navegacao::GridMap grid;
    if (!fetch_map(grid)) {
      return;
    }

    auto start = navegacao::find_symbol(grid, 'r');
    auto goal = navegacao::find_symbol(grid, 't');
    if (!start || !goal) {
      RCLCPP_ERROR(get_logger(), "Não encontrei robô (r) ou alvo (t) no mapa.");
      plan_executed_ = true;
      return;
    }

    std::vector<navegacao::GridCoordinate> path;
    if (!navegacao::run_a_star(grid, *start, *goal, path)) {
      RCLCPP_ERROR(get_logger(), "A* não encontrou caminho.");
      plan_executed_ = true;
      return;
    }

    RCLCPP_INFO(get_logger(), "Caminho encontrado com %zu passos. Reproduzindo...", path.size());
    if (execute_path(path)) {
      RCLCPP_INFO(get_logger(), "Percurso completo enviado ao simulador.");
    } else {
      RCLCPP_WARN(get_logger(), "Não consegui executar todo o caminho.");
    }

    plan_executed_ = true;
  }

private:
  bool fetch_map(navegacao::GridMap & grid)
  {
    auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();
    auto future = get_map_client_->async_send_request(request);
    auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 3s);
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Falha ao chamar /get_map");
      return false;
    }

    auto response = future.get();
    try {
      // Converte o tipo específico de ROS2 (BoundedVector<uint8_t, 2>)
      // para um std::vector<uint8_t> simples, usado pela função auxiliar.
      std::vector<uint8_t> shape_values(
        response->occupancy_grid_shape.begin(),
        response->occupancy_grid_shape.end());

      grid = navegacao::build_grid_from_response(
        response->occupancy_grid_flattened,
        shape_values);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Erro ao interpretar mapa: %s", ex.what());
      return false;
    }
    return true;
  }

  bool execute_path(const std::vector<navegacao::GridCoordinate> & path)
  {
    if (path.size() < 2) {
      RCLCPP_WARN(get_logger(), "Caminho trivial, nada para executar.");
      return true;
    }

    for (size_t i = 1; i < path.size(); ++i) {
      auto direction = navegacao::direction_from_step(path[i - 1], path[i]);
      if (direction.empty()) {
        RCLCPP_ERROR(get_logger(), "Passo inválido entre (%d,%d) e (%d,%d)",
                     path[i - 1].row, path[i - 1].col, path[i].row, path[i].col);
        return false;
      }
      if (!send_move(direction)) {
        RCLCPP_ERROR(get_logger(), "Falha ao mover para %s", direction.c_str());
        return false;
      }
      rclcpp::sleep_for(50ms);
    }
    return true;
  }

  bool send_move(const std::string & direction)
  {
    auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    request->direction = direction;
    auto future = move_client_->async_send_request(request);
    auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 2s);
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Serviço /move_command não respondeu.");
      return false;
    }

    auto response = future.get();
    if (!response->success) {
      RCLCPP_WARN(get_logger(), "Simulador recusou o movimento %s.", direction.c_str());
    }
    return response->success;
  }

  rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr get_map_client_;
  rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;
  bool plan_executed_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathPlannerNode>();
  node->run();
  rclcpp::shutdown();
  return 0;
}

