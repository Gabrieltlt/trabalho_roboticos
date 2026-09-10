# trabalho_roboticos

Pacote ROS 2 com um nó de navegação autônoma para o TurtleBot3 Burger, desenvolvido
para o **Trabalho 2 — Robô Navegador Gazebo** (Tópicos em Sistemas Robóticos e
Introdução à Robótica Inteligente).

O robô visita quatro estações de coleta marcadas no ambiente, retornando à posição
inicial após alcançar cada uma delas, evitando colisões com os obstáculos do mapa.

## Pré-requisitos

Este pacote **não é autossuficiente**: ele assume o ambiente de simulação
disponibilizado pelo professor (Dockerfile, `setup.sh`, `run.sh`,
`control_terminal.sh`, launch e world do TurtleBot3), disponível em
[repositório do professor](#) (ver Tabela 1 do enunciado).

Este repositório contém **apenas o pacote `trabalho_roboticos`**, que deve ser
colocado dentro de `ros_ws/src/` daquele repositório antes de compilar.

## Estrutura do pacote

```
trabalho_roboticos/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── trabalho_roboticos
└── trabalho_roboticos/
    ├── __init__.py
    └── navigator.py
```

## Instalação

```bash
# dentro do repositório do professor, com o container Docker já configurado
cp -r trabalho_roboticos /caminho/para/ros_ws/src/

# dentro do container:
cd /ros_ws
colcon build --symlink-install --packages-select trabalho_roboticos
source install/setup.bash
```

## Como rodar

Em um terminal, dentro do container, suba a simulação:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage5.launch.py
```

Em outro terminal (`./control_terminal.sh`), com o workspace *sourced*:

```bash
ros2 run trabalho_roboticos navigator
```

O robô inicia a missão automaticamente, sem intervenção do usuário.

## Missão

| Alvo | x (m) | y (m) |
|---|---|---|
| Verde | 2.20 | 2.20 |
| Vermelho | 2.15 | -2.15 |
| Azul | -2.16 | -2.16 |
| Laranja | -2.00 | 1.20 |

Posição inicial: `(-2.0, 2.0)`. Para cada alvo: navega até ele, permanece
parado por um instante, recua um pouco e retorna à posição inicial antes de
seguir para o próximo.

## Estratégia de navegação

O nó é **puramente reativo** (sem mapa/grade de ocupação): usa apenas a leitura
instantânea do `/scan` e a posição estimada via `/odom` a cada ciclo de controle
(10 Hz). A máquina de estados é:

```
SEEKING ──(obstáculo perto)──▶ AVOIDING ──(caminho livre + tempo mínimo)──▶ SEEKING
   │
   └──(alvo alcançado)──▶ ARRIVED_PAUSE ──▶ LEAVING ──▶ SEEKING (próximo destino)
```

- **SEEKING** — controlador proporcional combinado: corrige o ângulo em direção
  ao alvo e avança ao mesmo tempo, reduzindo a velocidade linear quanto maior o
  erro angular (giro puro quando o erro é grande, avanço pleno quando já está
  bem alinhado).
- **AVOIDING** — ao detectar obstáculo no setor frontal (±45°) a menos de
  `safe_distance`, escolhe o lado mais livre (comparando os setores laterais)
  **uma única vez** e mantém essa direção de giro enquanto dura o desvio
  (evita ficar recalculando e trocando de lado a cada ciclo). Só retoma a
  perseguição do alvo depois de um tempo mínimo girando *e* com o caminho
  frontal livre — essa histerese evita oscilar entre os dois estados.
- **ARRIVED_PAUSE / LEAVING** — ao chegar num alvo, o robô para, aguarda um
  tempo fixo e recua um pouco (verificando o setor traseiro do laser 360°)
  antes de seguir para o próximo destino — evita que ele tente navegar
  enquanto ainda está encostado no marcador que acabou de visitar.
- **Detecção de robô travado** — se a posição e a orientação não mudarem por
  alguns segundos seguidos (situação típica de mínimo local em corredores
  estreitos), o robô executa uma manobra de recuperação (recuo + giro
  acentuado) para quebrar o padrão de oscilação.

Todos os parâmetros de controle (velocidades máximas, ganhos proporcionais,
distância de segurança, tempos de histerese) estão centralizados no início da
classe `Navigator`, em `navigator.py`.

## Limitações conhecidas

- Navegação puramente reativa: não há planejamento de caminho global, então em
  cenários com becos muito estreitos o comportamento de desvio pode não
  encontrar a melhor rota (mitigado pela detecção de robô travado).
- Os limiares de distância/tempo foram ajustados empiricamente para o mapa do
  Trabalho 2 e podem precisar de recalibração em outros ambientes.

## Autor

_(preencha com seu nome e turma)_