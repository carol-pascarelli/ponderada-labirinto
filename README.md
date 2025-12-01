# Navegação e Mapeamento ROS 2 (Atividade 10%)

Projeto acadêmico em ROS 2 (Humble) que integra o simulador `culling_games` escrito em pygame com dois nós C++ extremamente simples:

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

## 4. Explicações rápidas para a arguição

### Como o A* funciona neste projeto
- O grid vira um grafo em que cada célula livre conecta às quatro adjacentes.
- O custo de cada passo é 1; a heurística é Manhattan (`|Δlinha| + |Δcoluna|`), que é admissível em grids ortogonais.
- Usamos uma `priority_queue` que sempre expande o nó com menor `g + h`.
- Pais são guardados em um `unordered_map`; ao atingir o alvo, reconstruímos o caminho do fim para o início.

### Representação do labirinto
- Cada célula é um caractere: `f` (free), `b` (barreira), `r` (robô) e `t` (target).
- O serviço `/get_map` envia a matriz achatada (`string[]`). A função `build_grid_from_response` só reestrutura isso em `std::vector<std::vector<char>>`.
- No mapeamento incremental guardamos um dicionário `coord -> estado` (com coordenadas relativas). Assim conseguimos salvar o mapa em texto e reutilizá-lo depois.

### Uso de smart pointers
- ROS2 já trabalha com `std::shared_ptr` para nós (`rclcpp::Node`), clientes e mensagens.
- Os nós são criados via `std::make_shared<PathPlannerNode>()` e `std::make_shared<MapperNode>()`, então nunca precisamos de `new`/`delete`.
- As respostas dos serviços (`auto response = future.get();`) usam `std::shared_ptr` providos pelo rclcpp, atendendo ao requisito de evitar ponteiros crus.

### Diferenças entre BFS, DFS e A*
- **DFS incremental** (usado no mapper) serve para cobrir todo o labirinto com pouco overhead de memória, seguindo sempre um ramo até o fim e depois voltando.
- **BFS** encontra caminhos mínimos, mas explodiria em memória se rodássemos em cada passo da exploração.
- **A*** combina o custo real (`g`) com a estimativa (`h`) e por isso chega rápido no alvo quando o mapa é conhecido. É ideal para a Parte 1 e para a verificação pós-mapeamento.

### Limitações do mapeamento
- A estratégia é intencionalmente simples: caminha devagar, uma célula de cada vez, e depende de o robô conseguir voltar para cobrir áreas não visitadas.
- Obstáculos só são conhecidos quando um sensor os detecta, então áreas isoladas permanecem como `b` no arquivo final.
- Como usamos apenas 4 direções, caminhos diagonais não existem e o custo sempre é a quantidade de células percorridas.

---

## 5. Testes manuais sugeridos

1. **Planejamento direto**  
   - Inicie o simulador (`ros2 run cg maze`).  
   - Rode `ros2 run navegacao path_planner`.  
   - Observe o robô chegar ao alvo em linha quase reta (dependendo do labirinto).

2. **Exploração + replanejamento**  
   - Com o simulador ativo, rode `ros2 run navegacao mapper`.  
   - Aguarde o log dizendo que o alvo foi encontrado e que o mapa foi salvo.  
   - Verifique o arquivo `~/mapped_labyrinth.txt`.  
   - Depois do reset automático, o robô percorre novamente o trajeto ótimo calculado apenas com o mapa interno.

---

## 6. Argumentos para justificativa do projeto

- **Algoritmo escolhido:** A* garante o caminho mínimo em grids ortogonais; é mais rápido que BFS porque usa heurística, e mais curto que DFS porque evita voltar à toa.
- **Comunicação ROS 2:**  
  - `/get_map` fornece o labirinto completo para a Parte 1.  
  - `/move_command` executa cada passo tanto no planejador quanto no mapeador.  
  - `/reset` é usado apenas na Parte 2 para reiniciar o cenário antes de reproduzir o caminho otimizado.
- **Mapeamento incremental:** O DFS iterativo precisa só de uma pilha com as células visitadas. Cada leitura dos sensores atualiza as oito vizinhanças imediatas, garantindo que paredes e espaços livres fiquem registrados.
- **Limitações reconhecidas:** Não tratamos ruído sensor, não otimizamos o tempo total de exploração e consideramos o grid fixo (sem expansão infinita). Esses pontos podem virar melhoria futura, mas fogem ao escopo da atividade de 10%.

---

## 7. Próximos passos sugeridos (caso queira evoluir)

- Adicionar visualização do grafo descoberto (por exemplo, publicando `nav_msgs/OccupancyGrid`).
- Substituir DFS por um algoritmo de fronteira (Fringe Search / D* Lite) para reduzir o tempo total de exploração.
- Integrar testes automatizados com `ament_cmake_gtest` para validar o A* em labirintos sintéticos.