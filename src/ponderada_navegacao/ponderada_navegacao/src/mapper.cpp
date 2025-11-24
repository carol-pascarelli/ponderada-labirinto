#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "ponderada_navegacao/grid_map.hpp"
#include "ponderada_navegacao/a_star.hpp"

#include <unordered_map>
#include <unordered_set>
#include <stack>
#include <vector>
#include <queue>
#include <fstream>
#include <climits>
#include <chrono>

using namespace std::chrono_literals;

class Mapper : public rclcpp::Node {
public:
  Mapper(): Node("mapper_node") {
    sensors_sub = this->create_subscription<cg_interfaces::msg::RobotSensors>(
      "/culling_games/robot_sensors", 10,
      std::bind(&Mapper::sensors_cb, this, std::placeholders::_1)
    );
    move_cli = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
    timer_ = this->create_wall_timer(300ms, std::bind(&Mapper::loop, this));
    RCLCPP_INFO(this->get_logger(), "Mapper iniciado - aguardando sensores...");
  }
private:
  rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensors_sub;
  rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_cli;
  rclcpp::TimerBase::SharedPtr timer_;
  cg_interfaces::msg::RobotSensors last_sensors;
  bool have_sensors{false};

  // Estado do mapeamento
  std::unordered_map<int, char> mapdata;  // key = r*10000 + c -> char ('b', 'f', 't')
  std::unordered_set<int> visited;        // células já visitadas
  std::stack<std::pair<int, int>> exploration_stack;  // pilha para DFS
  int robot_r = 0, robot_c = 0;
  bool robot_initialized = false;
  bool target_found = false;
  int target_r = -1, target_c = -1;
  bool mapping_complete = false;
  bool path_executing = false;
  std::vector<PathNode> path_to_target;
  size_t path_index = 0;
  bool done = false;
  std::vector<PathNode> path_to_explore;
  size_t explore_path_index = 0;

  // Direções: up, down, left, right
  std::vector<std::pair<std::string, std::pair<int, int>>> dirs = {
    {"up", {-1, 0}},
    {"down", {1, 0}},
    {"left", {0, -1}},
    {"right", {0, 1}}
  };

  void sensors_cb(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
    last_sensors = *msg;
    have_sensors = true;
  }

  int key(int r, int c) { return r * 10000 + c; }

  void update_map_from_sensors() {
    if (!have_sensors) return;

    // Atualizar células adjacentes baseado nos sensores
    std::vector<std::pair<std::string, std::pair<int, int>>> sensor_dirs = {
      {"up", {-1, 0}}, {"down", {1, 0}}, {"left", {0, -1}}, {"right", {0, 1}}
    };

    for (auto &d : sensor_dirs) {
      int dr = d.second.first, dc = d.second.second;
      int nbr_r = robot_r + dr, nbr_c = robot_c + dc;
      int k = key(nbr_r, nbr_c);

      char sensor_val = 'b';
      if (d.first == "up") sensor_val = last_sensors.up.empty() ? 'b' : last_sensors.up[0];
      else if (d.first == "down") sensor_val = last_sensors.down.empty() ? 'b' : last_sensors.down[0];
      else if (d.first == "left") sensor_val = last_sensors.left.empty() ? 'b' : last_sensors.left[0];
      else if (d.first == "right") sensor_val = last_sensors.right.empty() ? 'b' : last_sensors.right[0];

      if (sensor_val == 'b') {
        mapdata[k] = 'b';
      } else if (sensor_val == 'f') {
        if (!mapdata.count(k)) {
          mapdata[k] = 'f';
        }
      } else if (sensor_val == 't') {
        mapdata[k] = 't';
        if (!target_found) {
          target_found = true;
          target_r = nbr_r;
          target_c = nbr_c;
          RCLCPP_INFO(this->get_logger(), "Target encontrado em (%d, %d)!", target_r, target_c);
        }
      }
    }
  }

  bool call_move(const std::string &dir, int &newr, int &newc) {
    if (!move_cli->wait_for_service(500ms)) {
      return false;
    }
    auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    req->direction = dir;
    auto fut = move_cli->async_send_request(req);
    if (fut.wait_for(1s) != std::future_status::ready) {
      return false;
    }
    auto res = fut.get();
    if (!res->success) return false;
    newr = res->robot_pos[0];
    newc = res->robot_pos[1];
    return true;
  }

  void initialize_robot_position() {
    // Descobrir posição inicial fazendo um movimento inicial
    // Tentamos mover em todas as direções até conseguir uma que funcione
    if (!robot_initialized) {
      std::vector<std::string> test_dirs = {"right", "down", "left", "up"};
      bool found_pos = false;
      
      for (const auto& dir : test_dirs) {
        if (call_move(dir, robot_r, robot_c)) {
          // Conseguiu mover, agora temos a posição
          found_pos = true;
          // Voltar para a posição original (movimento oposto)
          std::string opposite;
          if (dir == "right") opposite = "left";
          else if (dir == "left") opposite = "right";
          else if (dir == "up") opposite = "down";
          else opposite = "up";
          
          int orig_r, orig_c;
          if (call_move(opposite, orig_r, orig_c)) {
            robot_r = orig_r;
            robot_c = orig_c;
          }
          break;
        }
      }
      
      // Se não conseguiu descobrir, usar (0,0) como fallback
      // Mas marcar a posição atual como visitada
      int k = key(robot_r, robot_c);
      mapdata[k] = 'f';
      visited.insert(k);
      exploration_stack.push({robot_r, robot_c});
      robot_initialized = true;
      RCLCPP_INFO(this->get_logger(), "Robô inicializado em (%d, %d)", robot_r, robot_c);
    }
  }

  std::pair<int, int> find_unvisited_cell() {
    // BFS para encontrar a célula não visitada mais próxima
    std::queue<std::pair<int, int>> q;
    std::unordered_set<int> seen;
    q.push({robot_r, robot_c});
    seen.insert(key(robot_r, robot_c));

    while (!q.empty()) {
      auto [r, c] = q.front();
      q.pop();

      for (auto &d : dirs) {
        int dr = d.second.first, dc = d.second.second;
        int nr = r + dr, nc = c + dc;
        int k = key(nr, nc);

        if (seen.count(k)) continue;
        seen.insert(k);

        if (mapdata.count(k) && mapdata[k] != 'b' && !visited.count(k)) {
          return {nr, nc};
        }
        if (mapdata.count(k) && mapdata[k] != 'b') {
          q.push({nr, nc});
        }
      }
    }
    return {-1, -1};
  }

  void exploration_phase() {
    if (mapping_complete) return;

    update_map_from_sensors();

    // Inicializar posição do robô se necessário
    if (!robot_initialized) {
      initialize_robot_position();
      return;
    }

    // Se estamos seguindo um caminho para explorar, continuar
    if (explore_path_index < path_to_explore.size()) {
      PathNode next = path_to_explore[explore_path_index];
      int dr = next.r - robot_r, dc = next.c - robot_c;
      std::string dir;
      if (dr == -1) dir = "up";
      else if (dr == 1) dir = "down";
      else if (dc == -1) dir = "left";
      else if (dc == 1) dir = "right";
      else {
        explore_path_index++;
        return;
      }

      int newr, newc;
      if (call_move(dir, newr, newc)) {
        robot_r = newr;
        robot_c = newc;
        explore_path_index++;
        int k = key(robot_r, robot_c);
        if (!visited.count(k)) {
          visited.insert(k);
          mapdata[k] = 'f';
        }
        return;
      } else {
        // Caminho bloqueado, limpar e tentar novamente
        path_to_explore.clear();
        explore_path_index = 0;
      }
    }

    // Procurar células não visitadas adjacentes
    for (auto &d : dirs) {
      int dr = d.second.first, dc = d.second.second;
      int nbr_r = robot_r + dr, nbr_c = robot_c + dc;
      int k = key(nbr_r, nbr_c);

      if (mapdata.count(k) && mapdata[k] != 'b' && !visited.count(k)) {
        std::string dir = d.first;
        int newr, newc;
        if (call_move(dir, newr, newc)) {
          robot_r = newr;
          robot_c = newc;
          visited.insert(key(robot_r, robot_c));
          mapdata[key(robot_r, robot_c)] = 'f';
          exploration_stack.push({robot_r, robot_c});
          RCLCPP_INFO(this->get_logger(), "Movido para (%d, %d) - explorando", robot_r, robot_c);
          return;
        }
      }
    }

    // Se não há células adjacentes não visitadas, procurar a mais próxima
    auto unvisited = find_unvisited_cell();
    if (unvisited.first >= 0) {
      GridMap temp_map = build_grid_map();
      PathNode start{robot_r, robot_c};
      PathNode goal{unvisited.first, unvisited.second};
      path_to_explore = astar_path(temp_map, start, goal);
      if (!path_to_explore.empty() && path_to_explore.size() > 1) {
        explore_path_index = 1;  // Pular a primeira (posição atual)
        RCLCPP_INFO(this->get_logger(), "Caminho calculado para próxima célula não visitada (%zu passos)",
                    path_to_explore.size() - 1);
        return;
      }
    }

    // Se não há mais células para explorar, finalizar mapeamento
    RCLCPP_INFO(this->get_logger(), "Mapeamento completo!");
    save_map_csv("mapped_map.csv");
    mapping_complete = true;
    
    // Se encontrou o target, calcular caminho
    if (target_found) {
      GridMap final_map = build_grid_map();
      PathNode start{robot_r, robot_c};
      PathNode goal{target_r, target_c};
      path_to_target = astar_path(final_map, start, goal);
      if (!path_to_target.empty()) {
        RCLCPP_INFO(this->get_logger(), "Caminho calculado com %zu passos", path_to_target.size());
        path_index = 1;  // Começar do segundo elemento (primeiro é a posição atual)
        path_executing = true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Não foi possível encontrar caminho até o target");
        done = true;
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Target não foi encontrado durante o mapeamento");
      done = true;
    }
  }

  void path_execution_phase() {
    if (!path_executing || path_index >= path_to_target.size()) {
      RCLCPP_INFO(this->get_logger(), "Navegação até o target concluída!");
      done = true;
      return;
    }

    PathNode next = path_to_target[path_index];
    int dr = next.r - robot_r, dc = next.c - robot_c;
    std::string dir;
    if (dr == -1) dir = "up";
    else if (dr == 1) dir = "down";
    else if (dc == -1) dir = "left";
    else if (dc == 1) dir = "right";
    else {
      path_index++;
      return;
    }

    int newr, newc;
    if (call_move(dir, newr, newc)) {
      robot_r = newr;
      robot_c = newc;
      path_index++;
      RCLCPP_INFO(this->get_logger(), "Movido %s -> (%d, %d) [%zu/%zu]", 
                  dir.c_str(), robot_r, robot_c, path_index, path_to_target.size());
      
      // Verificar se chegou ao target
      if (robot_r == target_r && robot_c == target_c) {
        RCLCPP_INFO(this->get_logger(), "Target alcançado!");
        done = true;
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Falha ao mover %s", dir.c_str());
    }
  }

  GridMap build_grid_map() {
    if (mapdata.empty()) {
      GridMap empty_map;
      empty_map.rows = 0;
      empty_map.cols = 0;
      return empty_map;
    }

    // Encontrar limites do mapa
    int minr = INT_MAX, minc = INT_MAX, maxr = INT_MIN, maxc = INT_MIN;
    for (auto &p : mapdata) {
      int r = p.first / 10000;
      int c = p.first % 10000;
      minr = std::min(minr, r);
      minc = std::min(minc, c);
      maxr = std::max(maxr, r);
      maxc = std::max(maxc, c);
    }

    int R = maxr - minr + 1;
    int C = maxc - minc + 1;
    std::vector<std::string> grid(R, std::string(C, 'b'));

    for (auto &p : mapdata) {
      int r = p.first / 10000;
      int c = p.first % 10000;
      grid[r - minr][c - minc] = p.second;
    }

    // Ajustar coordenadas do robô e target para o novo sistema
    int adj_robot_r = robot_r - minr;
    int adj_robot_c = robot_c - minc;
    if (adj_robot_r >= 0 && adj_robot_r < R && adj_robot_c >= 0 && adj_robot_c < C) {
      grid[adj_robot_r][adj_robot_c] = 'r';
    }

    if (target_found) {
      int adj_target_r = target_r - minr;
      int adj_target_c = target_c - minc;
      if (adj_target_r >= 0 && adj_target_r < R && adj_target_c >= 0 && adj_target_c < C) {
        grid[adj_target_r][adj_target_c] = 't';
      }
    }

    GridMap g;
    g.build_from_flat_strvec(grid, R, C);
    return g;
  }

  void save_map_csv(const std::string &filename) {
    if (mapdata.empty()) return;

    int minr = INT_MAX, minc = INT_MAX, maxr = INT_MIN, maxc = INT_MIN;
    for (auto &p : mapdata) {
      int r = p.first / 10000;
      int c = p.first % 10000;
      minr = std::min(minr, r);
      minc = std::min(minc, c);
      maxr = std::max(maxr, r);
      maxc = std::max(maxc, c);
    }

    int R = maxr - minr + 1;
    int C = maxc - minc + 1;
    std::vector<std::string> grid(R, std::string(C, 'b'));

    for (auto &p : mapdata) {
      int r = p.first / 10000;
      int c = p.first % 10000;
      grid[r - minr][c - minc] = p.second;
    }

    // Adicionar robô e target
    int adj_robot_r = robot_r - minr;
    int adj_robot_c = robot_c - minc;
    if (adj_robot_r >= 0 && adj_robot_r < R && adj_robot_c >= 0 && adj_robot_c < C) {
      grid[adj_robot_r][adj_robot_c] = 'r';
    }

    if (target_found) {
      int adj_target_r = target_r - minr;
      int adj_target_c = target_c - minc;
      if (adj_target_r >= 0 && adj_target_r < R && adj_target_c >= 0 && adj_target_c < C) {
        grid[adj_target_r][adj_target_c] = 't';
      }
    }

    std::ofstream ofs(filename);
    for (int i = 0; i < R; i++) {
      for (int j = 0; j < C; j++) {
        ofs << grid[i][j];
        if (j < C - 1) ofs << ",";
      }
      ofs << "\n";
    }
    ofs.close();
    RCLCPP_INFO(this->get_logger(), "Mapa salvo em %s (%dx%d)", filename.c_str(), R, C);
  }

  void loop() {
    if (done) return;
    if (!have_sensors) return;

    if (path_executing) {
      path_execution_phase();
    } else {
      exploration_phase();
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mapper>());
  rclcpp::shutdown();
  return 0;
}
