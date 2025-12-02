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
#include <stack>

#include "cg_interfaces/msg/robot_sensors.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/srv/reset.hpp"
#include "navegacao/grid_utils.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MapperNode : public rclcpp::Node
{
public:
  MapperNode()
  : rclcpp::Node("mapper")
  {
    map_output_path_ = this->declare_parameter<std::string>("map_output_path", default_output_path());

    sensors_sub_ = this->create_subscription<cg_interfaces::msg::RobotSensors>(
      "/culling_games/robot_sensors",
      rclcpp::SensorDataQoS(),
      std::bind(&MapperNode::on_sensors, this, std::placeholders::_1));

    move_client_ = this->create_client<cg_interfaces::srv::MoveCmd>("move_command");
    reset_client_ = this->create_client<cg_interfaces::srv::Reset>("reset");

    RCLCPP_INFO(get_logger(), "╔════════════════════════════════════════════════════╗");
    RCLCPP_INFO(get_logger(), "║           MAPPER NODE INICIADO                     ║");
    RCLCPP_INFO(get_logger(), "╚════════════════════════════════════════════════════╝");
  }

  void run()
  {
    if (!move_client_->wait_for_service(5s)) {
      RCLCPP_ERROR(get_logger(), "Serviço /move_command indisponível!");
      return;
    }

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
  struct Pos
  {
    int x = 0;
    int y = 0;
    
    bool operator==(const Pos& other) const {
      return x == other.x && y == other.y;
    }
    
    bool operator!=(const Pos& other) const {
      return !(*this == other);
    }
    
    bool operator<(const Pos& other) const {
      if (x != other.x) return x < other.x;
      return y < other.y;
    }
  };

  struct PosHash {
    size_t operator()(const Pos& p) const {
      return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
    }
  };

  enum CellType { UNKNOWN, FREE, WALL, TARGET };

  rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensors_sub_;
  rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;
  rclcpp::Client<cg_interfaces::srv::Reset>::SharedPtr reset_client_;

  cg_interfaces::msg::RobotSensors::SharedPtr last_sensors_;
  bool have_sensors_ = false;

  std::unordered_map<Pos, CellType, PosHash> map_;
  std::unordered_set<Pos, PosHash> visited_;
  std::stack<Pos> path_stack_;

  Pos current_pos_{0, 0};
  Pos initial_pos_{0, 0};
  bool initialized_ = false;

  bool target_found_ = false;
  Pos target_pos_{-1, -1};

  bool mapping_complete_ = false;
  bool map_saved_ = false;
  bool reset_done_ = false;
  bool replayed_ = false;
  bool finished_ = false;

  int exploration_moves_ = 0;
  int optimal_moves_ = 0;

  std::string map_output_path_;

  const std::vector<std::pair<std::string, std::pair<int, int>>> directions_{
    {"up", {0, 1}},
    {"down", {0, -1}},
    {"left", {-1, 0}},
    {"right", {1, 0}}
  };

  void on_sensors(const cg_interfaces::msg::RobotSensors::SharedPtr msg)
  {
    last_sensors_ = msg;
    have_sensors_ = true;
  }

  static std::string default_output_path()
  {
    const char * home = std::getenv("HOME");
    std::string base = home ? home : "/tmp";
    return base + "/mapped_labyrinth.txt";
  }

  void update_map_from_sensors()
  {
    if (!last_sensors_) return;

    std::map<std::string, std::string> sensor_values = {
      {"up", last_sensors_->up},
      {"down", last_sensors_->down},
      {"left", last_sensors_->left},
      {"right", last_sensors_->right}
    };

    for (const auto& [dir_name, offset] : directions_) {
      Pos neighbor{current_pos_.x + offset.first, current_pos_.y + offset.second};
      std::string value = sensor_values[dir_name];

      if (value == "b") {
        map_[neighbor] = WALL;
      } else if (value == "f") {
        if (map_.find(neighbor) == map_.end()) {
          map_[neighbor] = FREE;
        }
      } else if (value == "t") {
        map_[neighbor] = TARGET;
        if (!target_found_) {
          target_found_ = true;
          target_pos_ = neighbor;
          RCLCPP_INFO(get_logger(), "");
          RCLCPP_INFO(get_logger(), "🎯 ═══════════════════════════════════════════════════");
          RCLCPP_INFO(get_logger(), "   ALVO DETECTADO em (%d, %d) [SENSORES]", target_pos_.x, target_pos_.y);
          RCLCPP_INFO(get_logger(), "   Continuando mapeamento sem entrar no alvo...");
          RCLCPP_INFO(get_logger(), "   ═══════════════════════════════════════════════════");
          RCLCPP_INFO(get_logger(), "");
        }
      }
    }

    map_[current_pos_] = FREE;
  }

  bool call_move(const std::string& dir, Pos& new_pos)
  {
    auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    req->direction = dir;
    auto fut = move_client_->async_send_request(req);
    auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut, 2s);
    
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Timeout ao chamar move_command");
      return false;
    }
    
    auto res = fut.get();
    if (!res->success) {
      return false;
    }

    // Calcular nova posição baseado na direção
    for (const auto& [name, offset] : directions_) {
      if (name == dir) {
        new_pos.x = current_pos_.x + offset.first;
        new_pos.y = current_pos_.y + offset.second;
        break;
      }
    }

    return true;
  }

  std::string get_direction_to(const Pos& from, const Pos& to)
  {
    int dx = to.x - from.x;
    int dy = to.y - from.y;

    for (const auto& [name, offset] : directions_) {
      if (offset.first == dx && offset.second == dy) {
        return name;
      }
    }
    return "";
  }

  void exploration_step()
  {
    if (mapping_complete_) return;

    if (!initialized_) {
      visited_.insert(current_pos_);
      path_stack_.push(current_pos_);
      initial_pos_ = current_pos_;
      initialized_ = true;
      RCLCPP_INFO(get_logger(), "Iniciando exploração em (%d, %d)", current_pos_.x, current_pos_.y);
    }

    update_map_from_sensors();

    // Verificar se há células livres não visitadas (excluindo target)
    bool has_unexplored = false;
    for (const auto& [pos, type] : map_) {
      if (type == FREE && visited_.find(pos) == visited_.end()) {
        has_unexplored = true;
        break;
      }
    }
    
    if (!has_unexplored && target_found_) {
      mapping_complete_ = true;
      RCLCPP_INFO(get_logger(), "");
      RCLCPP_INFO(get_logger(), "✓ Exploração concluída: %d movimentos", exploration_moves_);
      RCLCPP_INFO(get_logger(), "✓ Células visitadas: %zu", visited_.size());
      RCLCPP_INFO(get_logger(), "✓ Alvo em: (%d, %d)", target_pos_.x, target_pos_.y);
      return;
    }

    // Procurar vizinho livre não visitado (NÃO incluir TARGET)
    std::string best_dir = "";
    
    for (const auto& [dir_name, offset] : directions_) {
      Pos neighbor{current_pos_.x + offset.first, current_pos_.y + offset.second};
      auto it = map_.find(neighbor);
      
      // Apenas células FREE, nunca TARGET
      if (it != map_.end() && 
          it->second == FREE &&
          visited_.find(neighbor) == visited_.end()) {
        best_dir = dir_name;
        break;
      }
    }

    if (!best_dir.empty()) {
      Pos new_pos;
      if (call_move(best_dir, new_pos)) {
        current_pos_ = new_pos;
        visited_.insert(current_pos_);
        path_stack_.push(current_pos_);
        exploration_moves_++;
        
        if (exploration_moves_ % 5 == 0) {
          RCLCPP_INFO(get_logger(), "Explorando: (%d, %d) | Movimentos: %d", 
                      current_pos_.x, current_pos_.y, exploration_moves_);
        }
      }
      return;
    }

    // Backtracking
    if (path_stack_.size() > 1) {
      path_stack_.pop();
      Pos previous = path_stack_.top();
      std::string dir = get_direction_to(current_pos_, previous);
      
      if (!dir.empty()) {
        Pos new_pos;
        if (call_move(dir, new_pos)) {
          current_pos_ = new_pos;
          exploration_moves_++;
          RCLCPP_INFO(get_logger(), "← Backtracking para (%d, %d)", current_pos_.x, current_pos_.y);
        }
      }
      return;
    }

    // Sem opções
    if (target_found_) {
      mapping_complete_ = true;
      RCLCPP_INFO(get_logger(), "");
      RCLCPP_INFO(get_logger(), "✓ Exploração concluída: %d movimentos", exploration_moves_);
    } else {
      RCLCPP_WARN(get_logger(), "⚠ Exploração terminou sem encontrar o alvo!");
    }
  }

  std::vector<Pos> calculate_optimal_path()
  {
    std::queue<Pos> q;
    std::unordered_map<Pos, Pos, PosHash> parent;
    std::unordered_set<Pos, PosHash> visited;

    q.push(initial_pos_);
    visited.insert(initial_pos_);
    parent[initial_pos_] = initial_pos_;

    while (!q.empty()) {
      Pos current = q.front();
      q.pop();

      if (current == target_pos_) {
        std::vector<Pos> path;
        Pos pos = target_pos_;
        
        while (pos != initial_pos_) {
          path.push_back(pos);
          pos = parent[pos];
        }
        path.push_back(initial_pos_);
        
        std::reverse(path.begin(), path.end());
        return path;
      }

      for (const auto& [_, offset] : directions_) {
        Pos neighbor{current.x + offset.first, current.y + offset.second};
        auto it = map_.find(neighbor);
        
        if (visited.find(neighbor) == visited.end() &&
            it != map_.end() && 
            (it->second == FREE || it->second == TARGET)) {
          visited.insert(neighbor);
          parent[neighbor] = current;
          q.push(neighbor);
        }
      }
    }

    return {};
  }

  void handle_post_mapping()
  {
    if (!map_saved_) {
      save_map_to_disk(map_output_path_);
      map_saved_ = true;
    }

    if (!target_found_) {
      finished_ = true;
      RCLCPP_WARN(get_logger(), "⚠ Target não encontrado - finalizando.");
      return;
    }

    if (!reset_done_) {
      auto optimal_path = calculate_optimal_path();
      
      if (optimal_path.empty()) {
        RCLCPP_ERROR(get_logger(), "✗ Não foi possível calcular rota ótima!");
        finished_ = true;
        return;
      }
      
      int optimal_length = optimal_path.size() - 1;
      int saved_moves = exploration_moves_ - optimal_length;
      float efficiency = (100.0f * saved_moves) / exploration_moves_;
      
      RCLCPP_INFO(get_logger(), "");
      RCLCPP_INFO(get_logger(), "┌────────────────────────────────────────────────────┐");
      RCLCPP_INFO(get_logger(), "│  Movimentos exploração:  %d%s│", 
                  exploration_moves_, std::string(27 - std::to_string(exploration_moves_).length(), ' ').c_str());
      RCLCPP_INFO(get_logger(), "│  Rota ótima calculada:   %d movimentos%s│", 
                  optimal_length, std::string(21 - std::to_string(optimal_length).length(), ' ').c_str());
      RCLCPP_INFO(get_logger(), "│  Economia:               %d movimentos (%.0f%%)%s│", 
                  saved_moves, efficiency, std::string(13 - std::to_string(saved_moves).length(), ' ').c_str());
      RCLCPP_INFO(get_logger(), "└────────────────────────────────────────────────────┘");
      RCLCPP_INFO(get_logger(), "");
      
      RCLCPP_INFO(get_logger(), "Chamando serviço /reset...");
      
      if (reset_game()) {
        current_pos_ = initial_pos_;
        reset_done_ = true;
        RCLCPP_INFO(get_logger(), "✓ Reset concluído. Robô em (%d, %d)", current_pos_.x, current_pos_.y);
        RCLCPP_INFO(get_logger(), "");
      } else {
        RCLCPP_ERROR(get_logger(), "✗ Falha no reset!");
        finished_ = true;
      }
      return;
    }

    if (!replayed_) {
      RCLCPP_INFO(get_logger(), "Aguardando estabilização do simulador...");
      rclcpp::sleep_for(2s);
      
      replay_optimal_path();
      replayed_ = true;
      
      RCLCPP_INFO(get_logger(), "");
      RCLCPP_INFO(get_logger(), "╔════════════════════════════════════════════════════╗");
      RCLCPP_INFO(get_logger(), "║  ESTATÍSTICAS FINAIS                               ║");
      RCLCPP_INFO(get_logger(), "╠════════════════════════════════════════════════════╣");
      RCLCPP_INFO(get_logger(), "║  Células exploradas:        %zu%s║", 
                  visited_.size(), std::string(23 - std::to_string(visited_.size()).length(), ' ').c_str());
      RCLCPP_INFO(get_logger(), "║  Movimentos exploração:     %d%s║", 
                  exploration_moves_, std::string(23 - std::to_string(exploration_moves_).length(), ' ').c_str());
      RCLCPP_INFO(get_logger(), "║  Movimentos rota ótima:     %d%s║", 
                  optimal_moves_, std::string(23 - std::to_string(optimal_moves_).length(), ' ').c_str());
      RCLCPP_INFO(get_logger(), "╚════════════════════════════════════════════════════╝");
      RCLCPP_INFO(get_logger(), "");
      
      finished_ = true;
    }
  }

  void replay_optimal_path()
  {
    // Resetar flag de sensores e aguardar nova leitura
    have_sensors_ = false;
    int wait_attempts = 0;
    
    RCLCPP_INFO(get_logger(), "Aguardando sensores após reset...");
    while (!have_sensors_ && wait_attempts < 100 && rclcpp::ok()) {
      rclcpp::spin_some(shared_from_this());
      rclcpp::sleep_for(50ms);
      wait_attempts++;
    }
    
    if (!have_sensors_) {
      RCLCPP_ERROR(get_logger(), "✗ Timeout aguardando sensores!");
      return;
    }
    
    RCLCPP_INFO(get_logger(), "✓ Sensores recebidos. Iniciando replay...");
    RCLCPP_INFO(get_logger(), "");
    
    auto path = calculate_optimal_path();
    
    if (path.empty() || path.size() < 2) {
      RCLCPP_ERROR(get_logger(), "✗ Caminho ótimo inválido!");
      return;
    }

    RCLCPP_INFO(get_logger(), "Executando rota ótima: %zu movimentos", path.size() - 1);
    RCLCPP_INFO(get_logger(), "De (%d, %d) até (%d, %d)", 
                initial_pos_.x, initial_pos_.y, target_pos_.x, target_pos_.y);
    RCLCPP_INFO(get_logger(), "");

    for (size_t i = 1; i < path.size() && rclcpp::ok(); ++i) {
      std::string dir = get_direction_to(path[i-1], path[i]);
      
      if (dir.empty()) {
        RCLCPP_ERROR(get_logger(), "✗ Direção inválida entre (%d,%d) e (%d,%d)!", 
                     path[i-1].x, path[i-1].y, path[i].x, path[i].y);
        break;
      }

      RCLCPP_INFO(get_logger(), "[%zu/%zu] Movendo %s...", i, path.size() - 1, dir.c_str());

      Pos new_pos;
      if (!call_move(dir, new_pos)) {
        RCLCPP_WARN(get_logger(), "✗ Falha ao mover %s", dir.c_str());
        break;
      }
      
      current_pos_ = new_pos;
      optimal_moves_++;
      
      RCLCPP_INFO(get_logger(), "   → Posição: (%d, %d)", current_pos_.x, current_pos_.y);
      
      rclcpp::sleep_for(500ms);
    }

    RCLCPP_INFO(get_logger(), "");
    
    if (current_pos_ == target_pos_) {
      RCLCPP_INFO(get_logger(), "🎯 Alvo alcançado com sucesso!");
    } else {
      RCLCPP_WARN(get_logger(), "⚠ Posição final (%d,%d) diferente do alvo (%d,%d)", 
                  current_pos_.x, current_pos_.y, target_pos_.x, target_pos_.y);
    }
  }

  void save_map_to_disk(const std::string& path) const
  {
    if (map_.empty()) {
      RCLCPP_WARN(get_logger(), "Mapa vazio.");
      return;
    }

    int min_x = INT_MAX, max_x = INT_MIN;
    int min_y = INT_MAX, max_y = INT_MIN;

    for (const auto& [pos, _] : map_) {
      min_x = std::min(min_x, pos.x);
      max_x = std::max(max_x, pos.x);
      min_y = std::min(min_y, pos.y);
      max_y = std::max(max_y, pos.y);
    }

    std::ofstream ofs(path);
    if (!ofs.is_open()) {
      RCLCPP_ERROR(get_logger(), "Erro ao abrir %s", path.c_str());
      return;
    }

    for (int y = max_y; y >= min_y; --y) {
      for (int x = min_x; x <= max_x; ++x) {
        Pos p{x, y};
        
        if (p == initial_pos_) {
          ofs << 's';  // start
        } else if (p == target_pos_) {
          ofs << 't';
        } else {
          auto it = map_.find(p);
          if (it != map_.end()) {
            switch (it->second) {
              case FREE: ofs << 'f'; break;
              case WALL: ofs << 'b'; break;
              case TARGET: ofs << 't'; break;
              default: ofs << '?'; break;
            }
          } else {
            ofs << '?';
          }
        }
      }
      ofs << '\n';
    }
    
    ofs.close();
    RCLCPP_INFO(get_logger(), "✓ Mapa salvo em: %s", path.c_str());
  }

  bool reset_game()
  {
    if (!reset_client_->wait_for_service(3s)) {
      RCLCPP_ERROR(get_logger(), "Serviço /reset indisponível!");
      return false;
    }

    auto req = std::make_shared<cg_interfaces::srv::Reset::Request>();
    req->is_random = false;
    req->map_name = "";
    
    auto fut = reset_client_->async_send_request(req);
    auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut, 5s);
    
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Timeout ao chamar /reset");
      return false;
    }
    
    auto res = fut.get();
    if (!res->success) {
      RCLCPP_ERROR(get_logger(), "Reset retornou success=false");
      return false;
    }

    RCLCPP_INFO(get_logger(), "✓ Serviço /reset retornou sucesso");
    
    // Aguardar reset completo
    rclcpp::sleep_for(3s);
    
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