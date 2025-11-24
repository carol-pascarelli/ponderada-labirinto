#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "ponderada_navegacao/grid_map.hpp"
#include "ponderada_navegacao/a_star.hpp"
#include <chrono>

using namespace std::chrono_literals;

class Pathfinder : public rclcpp::Node {
public:
  Pathfinder(): Node("pathfinder_node") {
    map_cli = this->create_client<cg_interfaces::srv::GetMap>("/get_map");
    move_cli = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
    
    // Aguardar serviços estarem disponíveis
    timer_ = this->create_wall_timer(500ms, std::bind(&Pathfinder::check_and_start, this));
    RCLCPP_INFO(this->get_logger(), "Pathfinder iniciado");
  }

private:
  rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr map_cli;
  rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_cli;
  rclcpp::TimerBase::SharedPtr timer_;
  bool done_{false};
  bool path_calculated_{false};
  bool waiting_for_map_{false};
  bool waiting_for_move_{false};
  std::vector<PathNode> path_;
  size_t path_index_ = 1;

  void check_and_start() {
    if (done_) return;
    
    if (!path_calculated_ && !waiting_for_map_) {
      if (map_cli->wait_for_service(100ms)) {
        request_map();
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Aguardando serviço /get_map... Certifique-se de que o simulador está rodando!");
      }
    } else if (path_calculated_ && !waiting_for_move_ && path_index_ < path_.size()) {
      execute_next_move();
    }
  }

  void request_map() {
    waiting_for_map_ = true;
    RCLCPP_INFO(this->get_logger(), "Chamando serviço /get_map...");
    auto req = std::make_shared<cg_interfaces::srv::GetMap::Request>();
    auto fut = map_cli->async_send_request(req, 
      [this](rclcpp::Client<cg_interfaces::srv::GetMap>::SharedFuture future) {
        waiting_for_map_ = false;
        auto resp = future.get();
        if (!resp) {
          RCLCPP_ERROR(this->get_logger(), "get_map retornou null");
          return;
        }
        process_map_response(resp);
      });
  }

  void process_map_response(const cg_interfaces::srv::GetMap::Response::SharedPtr resp) {
    int rows = resp->occupancy_grid_shape[0];
    int cols = resp->occupancy_grid_shape[1];

    std::vector<std::string> rows_vec;
    for (int r = 0; r < rows; ++r) {
      std::string line;
      for (int c = 0; c < cols; ++c) {
        line.push_back(resp->occupancy_grid_flattened[r*cols + c][0]);
      }
      rows_vec.push_back(line);
    }
    
    GridMap g;
    g.build_from_flat_strvec(rows_vec, rows, cols);

    // localizar robot e target
    PathNode start{-1,-1}, goal{-1,-1};
    for (int r=0;r<rows;r++){
      for (int c=0;c<cols;c++){
        char ch = rows_vec[r][c];
        if (ch == 'r') { start.r = r; start.c = c; }
        if (ch == 't') { goal.r = r; goal.c = c; }
      }
    }
    
    if (start.r<0 || goal.r<0) {
      RCLCPP_ERROR(this->get_logger(), "Não encontrou start/goal no mapa");
      return;
    }

    path_ = astar_path(g, start, goal);
    if (path_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "nenhum caminho encontrado");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Caminho calculado com %zu passos", path_.size());
    path_calculated_ = true;
    path_index_ = 1;
  }

  void execute_next_move() {
    if (path_index_ >= path_.size()) {
      RCLCPP_INFO(this->get_logger(), "Execução do caminho finalizada");
      done_ = true;
      return;
    }

    if (!move_cli->wait_for_service(100ms)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "move_command indisponivel");
      return;
    }

    auto dr = path_[path_index_].r - path_[path_index_ - 1].r;
    auto dc = path_[path_index_].c - path_[path_index_ - 1].c;
    std::string dir;
    
    if (dr == -1) dir = "up";
    else if (dr == 1) dir = "down";
    else if (dc == -1) dir = "left";
    else if (dc == 1) dir = "right";
    else { 
      RCLCPP_WARN(this->get_logger(), "Passo invalido");
      path_index_++;
      return;
    }

    waiting_for_move_ = true;
    auto mreq = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    mreq->direction = dir;
    auto mfut = move_cli->async_send_request(mreq,
      [this, dir](rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture future) {
        waiting_for_move_ = false;
        auto mres = future.get();
        if (!mres->success) {
          RCLCPP_WARN(this->get_logger(), "move falhou no simulador");
        } else {
          RCLCPP_INFO(this->get_logger(), "movido %s -> robot_pos=[%d,%d]", 
                      dir.c_str(), mres->robot_pos[0], mres->robot_pos[1]);
          path_index_++;
        }
      });
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pathfinder>());
  rclcpp::shutdown();
  return 0;
}
