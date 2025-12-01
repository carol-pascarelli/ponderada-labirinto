#include <chrono>
#include <climits>
#include <fstream>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "cg_interfaces/msg/robot_sensors.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/srv/reset.hpp"
#include "navegacao/grid_utils.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// Nó simples de mapeamento incremental:
//  - Usa apenas /culling_games/robot_sensors e /move_command
//  - Constrói um mapa em memória
//  - Após encontrar o alvo, executa A* nesse mapa para demonstrar o caminho ótimo

class MapperNode : public rclcpp::Node
{
public:
  MapperNode()
  : rclcpp::Node("mapper")
  {
    map_output_path_ = this->declare_parameter<std::string>("map_output_path", default_output_path());

    // Usar QoS de sensor (best_effort) para ser compatível com o publisher do simulador.
    sensors_sub_ = this->create_subscription<cg_interfaces::msg::RobotSensors>(
      "/culling_games/robot_sensors",
      rclcpp::SensorDataQoS(),
      std::bind(&MapperNode::on_sensors, this, std::placeholders::_1));

    move_client_ = this->create_client<cg_interfaces::srv::MoveCmd>("move_command");
    reset_client_ = this->create_client<cg_interfaces::srv::Reset>("reset");

    RCLCPP_INFO(get_logger(), "Mapper iniciado. Aguardando sensores...");
  }

  void run()
  {
    while (rclcpp::ok() && !finished_) {
      rclcpp::spin_some(shared_from_this());

      if (!have_sensors_) {
        rclcpp::sleep_for(100ms);
        continue;
      }

      if (!mapping_complete_) {
        exploration_step();
      } else {
        handle_post_mapping();
      }

      rclcpp::sleep_for(80ms);
    }

    RCLCPP_INFO(get_logger(), "Mapper finalizado.");
  }

private:
  // --- Tipos auxiliares ---

  struct CellCoord
  {
    int r = 0;
    int c = 0;
  };

  using Key = int;

  // --- Estado ROS ---

  rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensors_sub_;
  rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;
  rclcpp::Client<cg_interfaces::srv::Reset>::SharedPtr reset_client_;

  cg_interfaces::msg::RobotSensors last_sensors_;
  bool have_sensors_ = false;

  // --- Estado de mapeamento ---

  std::unordered_map<Key, char> mapdata_;  // 'b' parede, 'f' livre, 't' alvo
  std::unordered_set<Key> visited_;        // células que o robô já pisou

  int robot_r_ = 0;
  int robot_c_ = 0;
  bool robot_initialized_ = false;
  int start_r_ = 0;
  int start_c_ = 0;
  bool start_set_ = false;

  bool target_found_ = false;
  int target_r_ = -1;
  int target_c_ = -1;

  bool mapping_complete_ = false;
  bool map_saved_ = false;
  bool reset_done_ = false;
  bool replayed_ = false;
  bool finished_ = false;

  std::string map_output_path_;

  const std::vector<std::pair<std::string, CellCoord>> dirs_{
    {"up", {-1, 0}},
    {"down", {1, 0}},
    {"left", {0, -1}},
    {"right", {0, 1}}
  };

  // --- Callbacks ROS ---

  void on_sensors(const cg_interfaces::msg::RobotSensors::SharedPtr msg)
  {
    last_sensors_ = *msg;
    have_sensors_ = true;
  }

  // --- Helpers de grid interno ---

  Key key(int r, int c) const
  {
    return r * 1000 + c;
  }

  static std::string default_output_path()
  {
    const char * home = std::getenv("HOME");
    std::string base = home ? home : "/tmp";
    return base + "/mapped_labyrinth.txt";
  }

  void update_map_from_sensors()
  {
    // Para cada direção cardinal, atualiza o mapa interno com base no sensor.
    for (const auto & d : dirs_) {
      int dr = d.second.r;
      int dc = d.second.c;
      int nr = robot_r_ + dr;
      int nc = robot_c_ + dc;
      Key k = key(nr, nc);

      char val = 'b';
      if (d.first == "up") {
        val = last_sensors_.up.empty() ? 'b' : last_sensors_.up[0];
      } else if (d.first == "down") {
        val = last_sensors_.down.empty() ? 'b' : last_sensors_.down[0];
      } else if (d.first == "left") {
        val = last_sensors_.left.empty() ? 'b' : last_sensors_.left[0];
      } else if (d.first == "right") {
        val = last_sensors_.right.empty() ? 'b' : last_sensors_.right[0];
      }

      if (val == 'b') {
        mapdata_[k] = 'b';
      } else if (val == 'f') {
        if (!mapdata_.count(k)) {
          mapdata_[k] = 'f';
        }
      } else if (val == 't') {
        mapdata_[k] = 't';
        if (!target_found_) {
          target_found_ = true;
          target_r_ = nr;
          target_c_ = nc;
          RCLCPP_INFO(get_logger(), "Target detectado pelos sensores em (%d, %d).", target_r_, target_c_);
        }
      }
    }
  }

  bool call_move(const std::string & dir, int & new_r, int & new_c)
  {
    if (!move_client_->wait_for_service(500ms)) {
      RCLCPP_ERROR(get_logger(), "Serviço /move_command indisponível.");
      return false;
    }
    auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    req->direction = dir;
    auto fut = move_client_->async_send_request(req);
    auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut, 1s);
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Serviço /move_command não respondeu.");
      return false;
    }
    auto res = fut.get();
    if (!res->success) {
      return false;
    }

    new_r = res->robot_pos[0];
    new_c = res->robot_pos[1];

    // Se o simulador indicar que a posição do robô é igual ao alvo, salvamos o target.
    if (res->robot_pos == res->target_pos) {
      target_found_ = true;
      target_r_ = new_r;
      target_c_ = new_c;
      RCLCPP_INFO(get_logger(), "Target alcançado em (%d, %d) durante exploração.", target_r_, target_c_);
    }

    return true;
  }

  void initialize_robot_position()
  {
    if (robot_initialized_) {
      return;
    }

    // Tentativa simples: mover em alguma direção válida só para ler a posição real.
    std::vector<std::string> test_dirs{"right", "down", "left", "up"};
    bool found = false;
    for (const auto & d : test_dirs) {
      int nr, nc;
      if (call_move(d, nr, nc)) {
        found = true;
        // Movimento oposto para voltar.
        std::string opp;
        if (d == "right") { opp = "left"; }
        else if (d == "left") { opp = "right"; }
        else if (d == "up") { opp = "down"; }
        else { opp = "up"; }

        int br, bc;
        if (call_move(opp, br, bc)) {
          robot_r_ = br;
          robot_c_ = bc;
        } else {
          robot_r_ = nr;
          robot_c_ = nc;
        }
        break;
      }
    }

    if (!found) {
      robot_r_ = 0;
      robot_c_ = 0;
    }

    if (!start_set_) {
      start_r_ = robot_r_;
      start_c_ = robot_c_;
      start_set_ = true;
    }

    Key k = key(robot_r_, robot_c_);
    mapdata_[k] = 'f';
    visited_.insert(k);
    robot_initialized_ = true;
    RCLCPP_INFO(get_logger(), "Robô inicializado em (%d, %d).", robot_r_, robot_c_);
  }

  std::pair<int, int> find_closest_unvisited() const
  {
    std::queue<std::pair<int, int>> q;
    std::unordered_set<Key> seen;

    q.push({robot_r_, robot_c_});
    seen.insert(key(robot_r_, robot_c_));

    while (!q.empty()) {
      auto [r, c] = q.front();
      q.pop();

      for (const auto & d : dirs_) {
        int nr = r + d.second.r;
        int nc = c + d.second.c;
        Key k = key(nr, nc);
        if (seen.count(k)) {
          continue;
        }
        seen.insert(k);

        auto it = mapdata_.find(k);
        if (it != mapdata_.end() && it->second != 'b') {
          if (!visited_.count(k)) {
            return {nr, nc};
          }
          q.push({nr, nc});
        }
      }
    }

    return {-1, -1};
  }

  navegacao::GridMap build_grid_from_mapdata() const
  {
    navegacao::GridMap grid;
    if (mapdata_.empty()) {
      return grid;
    }

    int min_r = INT_MAX, min_c = INT_MAX, max_r = INT_MIN, max_c = INT_MIN;
    for (const auto & p : mapdata_) {
      int r = p.first / 1000;
      int c = p.first % 1000;
      min_r = std::min(min_r, r);
      min_c = std::min(min_c, c);
      max_r = std::max(max_r, r);
      max_c = std::max(max_c, c);
    }

    int rows = max_r - min_r + 1;
    int cols = max_c - min_c + 1;
    grid.cells.assign(static_cast<size_t>(rows), std::vector<char>(static_cast<size_t>(cols), 'b'));

    for (const auto & p : mapdata_) {
      int r = p.first / 1000;
      int c = p.first % 1000;
      grid.cells[static_cast<size_t>(r - min_r)][static_cast<size_t>(c - min_c)] = p.second;
    }

    // Marca robô e alvo
    int rr = robot_r_ - min_r;
    int rc = robot_c_ - min_c;
    if (rr >= 0 && rr < rows && rc >= 0 && rc < cols) {
      grid.cells[static_cast<size_t>(rr)][static_cast<size_t>(rc)] = 'r';
    }
    if (target_found_) {
      int tr = target_r_ - min_r;
      int tc = target_c_ - min_c;
      if (tr >= 0 && tr < rows && tc >= 0 && tc < cols) {
        grid.cells[static_cast<size_t>(tr)][static_cast<size_t>(tc)] = 't';
      }
    }

    return grid;
  }

  void save_map_to_disk(const std::string & path) const
  {
    auto grid = build_grid_from_mapdata();
    if (grid.rows() == 0 || grid.cols() == 0) {
      RCLCPP_WARN(get_logger(), "Mapa vazio, nada para salvar.");
      return;
    }

    std::ofstream ofs(path);
    if (!ofs.is_open()) {
      RCLCPP_ERROR(get_logger(), "Não consegui abrir arquivo de mapa em %s", path.c_str());
      return;
    }

    for (size_t r = 0; r < grid.rows(); ++r) {
      for (size_t c = 0; c < grid.cols(); ++c) {
        ofs << grid.cells[r][c];
      }
      ofs << '\n';
    }
    ofs.close();
    RCLCPP_INFO(get_logger(), "Mapa salvo em %s", path.c_str());
  }

  void exploration_step()
  {
    if (mapping_complete_) {
      return;
    }

    update_map_from_sensors();

    if (!robot_initialized_) {
      initialize_robot_position();
      return;
    }

    // 1) Tentar vizinho livre não visitado
    for (const auto & d : dirs_) {
      int nr = robot_r_ + d.second.r;
      int nc = robot_c_ + d.second.c;
      Key k = key(nr, nc);
      auto it = mapdata_.find(k);
      if (it != mapdata_.end() && it->second != 'b' && !visited_.count(k)) {
        int new_r, new_c;
        if (call_move(d.first, new_r, new_c)) {
          robot_r_ = new_r;
          robot_c_ = new_c;
          Key nk = key(robot_r_, robot_c_);
          visited_.insert(nk);
          mapdata_[nk] = 'f';
          RCLCPP_INFO(get_logger(), "Explorando célula (%d, %d).", robot_r_, robot_c_);
        }
        return;
      }
    }

    // 2) Não há vizinho novo; procurar célula não visitada mais próxima via BFS
    auto unvisited = find_closest_unvisited();
    if (unvisited.first >= 0) {
      auto grid = build_grid_from_mapdata();
      if (grid.rows() == 0 || grid.cols() == 0) {
        mapping_complete_ = true;
        return;
      }

      // Converter coordenadas globais (robot_r_, robot_c_) para índices na matriz
      int min_r = INT_MAX, min_c = INT_MAX;
      for (const auto & p : mapdata_) {
        int r = p.first / 1000;
        int c = p.first % 1000;
        min_r = std::min(min_r, r);
        min_c = std::min(min_c, c);
      }

      navegacao::GridCoordinate start_idx{
        robot_r_ - min_r,
        robot_c_ - min_c};
      navegacao::GridCoordinate goal_idx{
        unvisited.first - min_r,
        unvisited.second - min_c};

      std::vector<navegacao::GridCoordinate> path;
      if (navegacao::run_a_star(grid, start_idx, goal_idx, path) && path.size() > 1) {
        auto next = path[1];
        int global_r = next.row + min_r;
        int global_c = next.col + min_c;
        int dr = global_r - robot_r_;
        int dc = global_c - robot_c_;
        std::string dir;
        if (dr == -1 && dc == 0) dir = "up";
        else if (dr == 1 && dc == 0) dir = "down";
        else if (dr == 0 && dc == -1) dir = "left";
        else if (dr == 0 && dc == 1) dir = "right";
        else {
          return;
        }

        int new_r, new_c;
        if (call_move(dir, new_r, new_c)) {
          robot_r_ = new_r;
          robot_c_ = new_c;
          Key nk = key(robot_r_, robot_c_);
          visited_.insert(nk);
          mapdata_[nk] = 'f';
          RCLCPP_INFO(get_logger(), "Movendo por caminho planejado para (%d, %d).", robot_r_, robot_c_);
        }
        return;
      }
    }

    // 3) Não há mais nada para explorar
    mapping_complete_ = true;
    RCLCPP_INFO(get_logger(), "Mapeamento completo.");
  }

  void handle_post_mapping()
  {
    if (!map_saved_) {
      save_map_to_disk(map_output_path_);
      map_saved_ = true;
    }

    if (!target_found_) {
      finished_ = true;
      RCLCPP_WARN(get_logger(), "Target não foi encontrado durante o mapeamento.");
      return;
    }

    if (!reset_done_) {
      if (reset_game()) {
        robot_r_ = start_r_;
        robot_c_ = start_c_;
        reset_done_ = true;
        RCLCPP_INFO(get_logger(), "Simulador reiniciado. Repetindo trajeto ótimo...");
      }
      return;
    }

    if (!replayed_) {
      replay_path_to_target();
      replayed_ = true;
      finished_ = true;
    }
  }

  void replay_path_to_target()
  {
    if (!target_found_) {
      return;
    }

    auto grid = build_grid_from_mapdata();
    if (grid.rows() == 0 || grid.cols() == 0) {
      return;
    }

    int min_r = INT_MAX, min_c = INT_MAX;
    for (const auto & p : mapdata_) {
      int r = p.first / 1000;
      int c = p.first % 1000;
      min_r = std::min(min_r, r);
      min_c = std::min(min_c, c);
    }

    navegacao::GridCoordinate start_idx{
      robot_r_ - min_r,
      robot_c_ - min_c};
    navegacao::GridCoordinate goal_idx{
      target_r_ - min_r,
      target_c_ - min_c};

    std::vector<navegacao::GridCoordinate> path;
    if (!navegacao::run_a_star(grid, start_idx, goal_idx, path) || path.size() < 2) {
      RCLCPP_ERROR(get_logger(), "Falha ao encontrar caminho no mapa mapeado.");
      return;
    }

    RCLCPP_INFO(get_logger(), "Reexecutando A* no mapa mapeado (%zu passos)...", path.size() - 1);

    for (size_t i = 1; i < path.size() && rclcpp::ok(); ++i) {
      auto next = path[i];
      int global_r = next.row + min_r;
      int global_c = next.col + min_c;

      int dr = global_r - robot_r_;
      int dc = global_c - robot_c_;
      std::string dir;
      if (dr == -1 && dc == 0) dir = "up";
      else if (dr == 1 && dc == 0) dir = "down";
      else if (dr == 0 && dc == -1) dir = "left";
      else if (dr == 0 && dc == 1) dir = "right";
      else {
        continue;
      }

      int new_r, new_c;
      if (!call_move(dir, new_r, new_c)) {
        RCLCPP_WARN(get_logger(), "Falha ao mover %s durante a repetição.", dir.c_str());
        break;
      }
      robot_r_ = new_r;
      robot_c_ = new_c;
      RCLCPP_INFO(get_logger(), "Repetição: movido %s -> (%d, %d)", dir.c_str(), robot_r_, robot_c_);
      rclcpp::sleep_for(80ms);
    }
  }

  bool reset_game()
  {
    if (!reset_client_->wait_for_service(1s)) {
      RCLCPP_ERROR(get_logger(), "Serviço /reset indisponível.");
      return false;
    }

    auto req = std::make_shared<cg_interfaces::srv::Reset::Request>();
    req->is_random = false;
    req->map_name = "";
    auto fut = reset_client_->async_send_request(req);
    auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut, 3s);
    if (status != rclcpp::FutureReturnCode::SUCCESS || !fut.get()->success) {
      RCLCPP_ERROR(get_logger(), "Falha ao chamar /reset.");
      return false;
    }

    rclcpp::sleep_for(500ms);
    return true;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapperNode>();
  node->run();
  rclcpp::shutdown();
  return 0;
}


