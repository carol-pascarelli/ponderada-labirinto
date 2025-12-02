# Navegação e Mapeamento ROS 2 

Projeto em ROS que integra o simulador `culling_games` escrito em pygame com dois nós C++ extremamente simples:

- `path_planner`: resolve a Parte 1 executando A* com o mapa oficial fornecido pelo serviço `/get_map`.
- `mapper`: resolve a Parte 2 explorando o labirinto apenas com os sensores, monta um mapa em memória, salva em `.txt` e comprova o trajeto ótimo rodando A* novamente sem consultar `/get_map`.

Tudo foi escrito com foco em clareza para facilitar a arguição da atividade.

---

## 1. Estrutura do workspace

```
ponderada_navegacao/
├── README.md
├── src/
│   ├── culling_games/        # simulador pygame + serviços
│   └── navegacao/            # <--- novo pacote C++ desta atividade
│       ├── include/navegacao/grid_utils.hpp   # helpers de grafo/A*
│       ├── src/path_planner_node.cpp          # Parte 1
│       ├── src/mapper_node.cpp               # Parte 2
│       └── launch/*.launch.py
```

---

## 2. Dependências e build

1. Instalar ROS 2 Humble (ou superior) conforme documentação oficial.
2. Instalar pygame (`sudo apt install python3-pygame`) se ainda não estiver no sistema.
3. Na raiz do workspace:
   ```bash
   colcon build --packages-select cg cg_interfaces navegacao
   source install/setup.bash
   ```

> Dica: sempre abra **dois terminais** com `source install/setup.bash`. Um roda o simulador, o outro executa os nós C++.

---

## 3. Como executar

### 3.1 Simulador
```bash
source install/setup.bash
ros2 run cg maze           # inicia pygame + serviços (/get_map, /move_command, /reset)
```

### 3.2 Parte 1 — `path_planner`
```bash
source install/setup.bash
ros2 launch navegacao path_planner.launch.py
```

Fluxo:
1. O nó chama `/get_map`, converte o grid em matriz simples (`f` livre, `b` parede, `r` robô, `t` alvo).
2. Executa A* puro (sem bibliotecas extras) e gera uma sequência de direções.
3. Envia cada passo ao simulador usando `/move_command`.

### 3.3 Parte 2 — `mapper`
```bash
source install/setup.bash
ros2 launch navegacao mapper.launch.py map_output_path:=/tmp/meu_mapa.txt
```

Fluxo:
1. O nó ouve `/culling_games/robot_sensors` e atualiza um grafo incremental usando DFS iterativo.
2. Ao caminhar, chama `/move_command` apenas quando a célula vizinha foi confirmada como livre pelos sensores.
3. Quando todas as células acessíveis foram visitadas e o alvo foi encontrado:
   - O mapa descoberto é salvo em `map_output_path` (default: `~/mapped_labyrinth.txt`).
   - Um reset do jogo é solicitado via `/reset`.
   - O nó executa A* novamente usando **apenas** o mapa salvo, comprovando que o planejamento funciona sem consultar `/get_map`.

---

[Vídeo Demonstrativo](https://studio.youtube.com/video/_fuaRmTKc5E)
