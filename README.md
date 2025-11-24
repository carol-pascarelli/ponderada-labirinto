# Navegação e Mapeamento de Labirinto - ROS 2

Este projeto implementa uma solução completa para navegação autônoma em labirintos usando ROS 2, dividida em duas partes principais conforme o enunciado da atividade ponderada.

## 📋 Descrição

O desafio consiste em aplicar algoritmos de grafos para resolver problemas de busca em um simulador de labirinto. O projeto é dividido em duas partes:

1. **Parte 1 - Navegação com Mapa:** Encontrar a rota otimizada até o alvo tendo acesso ao mapa completo.
2. **Parte 2 - Mapeamento e Navegação:** Mapear o labirinto usando apenas sensores locais e, em seguida, usar o mapa criado para navegar até o alvo.

## 🏗️ Estrutura do Projeto

```
ponderada_labirinto/
├── src/
│        #solução
│       └── ponderada_navegacao/
│           ├── src/
│           │   ├── pathfinder.cpp   # Parte 1: Navegação com mapa
│           │   └── mapper.cpp       # Parte 2: Mapeamento + navegação
│           ├── include/
│           │   └── ponderada_navegacao/
│           │       ├── a_star.hpp   # Algoritmo A* para busca de caminho
│           │       └── grid_map.hpp # Estrutura de dados para representar o mapa
│           └── launch/
│               ├── pathfinder.launch.py
│               └── mapper.launch.py
└── README.md
```

## 🔧 Componentes Principais

### Parte 1: Pathfinder (`pathfinder.cpp`)

O `pathfinder` implementa a navegação otimizada quando o mapa completo está disponível:

- **Objetivo:** Encontrar o caminho mais curto do robô até o alvo
- **Algoritmo:** A* (A-star) para busca de caminho otimizada
- **Entrada:** Mapa completo via serviço `/get_map`
- **Saída:** Execução automática dos movimentos até o alvo

**Funcionamento:**
1. Conecta-se ao serviço `/get_map` para obter o mapa completo
2. Localiza a posição do robô ('r') e do alvo ('t')
3. Calcula o caminho otimizado usando algoritmo A*
4. Executa os movimentos automaticamente via serviço `/move_command`

### Parte 2: Mapper (`mapper.cpp`)

O `mapper` implementa a exploração e mapeamento do labirinto usando apenas sensores:

- **Objetivo:** Mapear o labirinto e depois navegar até o alvo
- **Algoritmo:** Exploração baseada em BFS + A* para navegação
- **Entrada:** Dados dos sensores via tópico `/culling_games/robot_sensors`
- **Saída:** Mapa salvo em CSV + navegação até o alvo

**Funcionamento:**
1. **Fase de Exploração:**
   - Recebe dados dos sensores (8 direções: up, down, left, right, diagonais)
   - Atualiza o mapa interno conforme explora
   - Move-se para células não visitadas
   - Quando não há células adjacentes não visitadas, usa A* para ir até a próxima área não explorada
   - Detecta o target ('t') quando encontrado nos sensores

2. **Fase de Mapeamento:**
   - Salva o mapa completo em `mapped_map.csv`
   - Marca células como: 'b' (bloqueado), 'f' (livre), 't' (target), 'r' (robô)

3. **Fase de Navegação:**
   - Usa o mapa criado para calcular caminho otimizado até o target com A*
   - Executa os movimentos automaticamente

### Algoritmo A* (`a_star.hpp`)

Implementação do algoritmo A* para busca de caminho:

- **Heurística:** Distância de Manhattan
- **Estrutura:** Priority queue para exploração eficiente
- **Retorno:** Vetor de nós representando o caminho do início ao objetivo

### GridMap (`grid_map.hpp`)

Estrutura de dados para representar o labirinto:

- Armazena o grid como vetor de strings
- Método `is_free()` para verificar se uma célula é transitável
- Suporta construção a partir de dados do serviço `/get_map`

## 🚀 Como Executar

### Pré-requisitos

- ROS 2 (testado com Jazzy)
- Workspace ROS 2 configurado
- Pacotes `cg`, `cg_interfaces` e `cg_teleop` compilados

### Compilação

```bash
cd /home/inteli/ponderada_labirinto
colcon build
source install/setup.bash
```

### Parte 1: Pathfinder (Navegação com Mapa)

**Terminal 1 - Simulador:**
```bash
cd /home/inteli/ponderada_labirinto
source install/setup.bash
ros2 run cg maze -- --map 1.csv
```

**Terminal 2 - Pathfinder:**
```bash
cd /home/inteli/ponderada_labirinto
source install/setup.bash
ros2 launch ponderada_navegacao pathfinder.launch.py
```

**O que acontece:**
- O pathfinder obtém o mapa completo
- Calcula o caminho otimizado usando A*
- O robô azul se move automaticamente até o target vermelho

### Parte 2: Mapper (Mapeamento + Navegação)

**Terminal 1 - Simulador:**
```bash
cd /home/inteli/ponderada_labirinto
source install/setup.bash
ros2 run cg maze -- --map 1.csv
```

**Terminal 2 - Mapper:**
```bash
cd /home/inteli/ponderada_labirinto
source install/setup.bash
ros2 launch ponderada_navegacao mapper.launch.py
```

**O que acontece:**
1. O mapper começa a explorar o labirinto usando apenas os sensores
2. Constrói o mapa dinamicamente conforme explora
3. Quando encontra o target, continua mapeando até explorar tudo
4. Salva o mapa em `mapped_map.csv`
5. Calcula e executa o caminho otimizado até o target usando A*

### Mapas Disponíveis

Você pode usar diferentes mapas:

**Mapas pré-definidos:**
- `test.csv` - Labirinto simples (sem paredes internas)
- `1.csv` a `10.csv` - Labirintos com diferentes complexidades

**Mapa aleatório:**
- `--generate` - Gera um novo labirinto aleatório a cada execução

**Exemplos:**

Usar um mapa específico:
```bash
ros2 run cg maze -- --map 2.csv
```

Gerar um mapa aleatório (recomendado para testar o mapper):
```bash
ros2 run cg maze -- --generate
```

Usar mapa aleatório padrão (seleciona um dos mapas existentes):
```bash
ros2 run cg maze
```

### Testando com Mapas Aleatórios

O **mapper** foi projetado para funcionar com **qualquer labirinto**, incluindo mapas aleatórios gerados. Isso comprova que o algoritmo de mapeamento funciona independentemente do formato do labirinto:

**Terminal 1 - Simulador com mapa aleatório:**
```bash
ros2 run cg maze -- --generate
```

**Terminal 2 - Mapper:**
```bash
ros2 launch ponderada_navegacao mapper.launch.py
```

O mapper irá:
1. Explorar o labirinto aleatório usando apenas sensores
2. Construir o mapa dinamicamente
3. Encontrar o target durante a exploração
4. Salvar o mapa em `mapped_map.csv`
5. Navegar até o target usando A* no mapa criado

Isso demonstra que o algoritmo funciona corretamente para qualquer configuração de labirinto!

## 📡 Comunicação ROS 2

### Tópicos

| Tópico | Tipo | Uso |
|--------|------|-----|
| `/culling_games/robot_sensors` | `cg_interfaces/msg/RobotSensors` | Dados dos sensores (8 direções) - usado pelo mapper |

### Serviços

| Serviço | Tipo | Uso |
|---------|------|-----|
| `/get_map` | `cg_interfaces/srv/GetMap` | Obter mapa completo - usado pelo pathfinder |
| `/move_command` | `cg_interfaces/srv/MoveCmd` | Enviar comandos de movimento (up, down, left, right) |

## 📊 Saídas

### Mapa Gerado (`mapped_map.csv`)

O mapper salva o mapa explorado em formato CSV:
- `b` = Bloqueado (parede)
- `f` = Livre (caminho)
- `r` = Robô (posição atual)
- `t` = Target (objetivo)

## 🔍 Algoritmos Utilizados

### A* (A-star)

- **Tipo:** Algoritmo de busca em grafos
- **Heurística:** Distância de Manhattan
- **Complexidade:** O(b^d) onde b é o fator de ramificação e d é a profundidade
- **Vantagem:** Garante encontrar o caminho mais curto quando a heurística é admissível

### Exploração BFS

- **Tipo:** Busca em largura para encontrar próxima célula não visitada
- **Uso:** Quando não há células adjacentes não visitadas, encontra a mais próxima

## 🐛 Troubleshooting

### Pathfinder não encontra o serviço `/get_map`

**Problema:** Timeout ao chamar `/get_map`

**Solução:** Certifique-se de que o simulador está rodando **antes** de iniciar o pathfinder.

### Mapper não se move

**Problema:** O robô não está se movendo durante a exploração

**Solução:** 
- Verifique se os sensores estão sendo recebidos: `ros2 topic echo /culling_games/robot_sensors`
- Verifique se o serviço `/move_command` está disponível: `ros2 service list | grep move`

### Mapa não é salvo

**Problema:** `mapped_map.csv` não é criado

**Solução:** O mapa só é salvo quando a exploração está completa. Aguarde a mensagem "Mapeamento completo!" nos logs.

## 📝 Notas

- O pathfinder assume que o mapa está disponível via `/get_map`
- O mapper não usa `/get_map` - mapeia apenas com sensores, funcionando com **qualquer labirinto** (incluindo mapas aleatórios)
- Ambos usam o algoritmo A* para calcular o caminho otimizado
- O mapper pode levar mais tempo pois precisa explorar todo o labirinto primeiro
- **Para comprovar que o algoritmo funciona:** Use `--generate` para criar mapas aleatórios e teste o mapper

## 👤 Autor

Implementação da atividade ponderada de navegação e mapeamento.

## 📄 Licença

Este projeto faz parte de uma atividade acadêmica.

[Vídeo de demonstração](http://youtube.com/)
