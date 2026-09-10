# Trabalho Final - Sistemas Robóticos

Pacote ROS2 com um nodo de navegação autônoma, desenvolvido para o tabalho de Tópicos em Sistemas Robóticos

---

## Pré-requisitos

- ROS2 Humble
- Python 3.10+
- Ubuntu 22.04
- Dependências ROS listadas no `package.xml`.

Além disso, o pacote **não é autossuficiente**: ele assume o ambiente de simulação
disponibilizado pelo professor, disponível no [link](https://github.com/bryanumpierremoreira/trabalho2_robotica_2026).

---

## Estrutura do pacote

```
trabalho_roboticos/
│   └── 📁 trabalho_roboticos/      # Algoritmo de navegação
└── package.xml                     # Dependências ROS
```

---

## Instalação

### 1. Configurando a Docker
```bash
./setup.sh # Pule se já rodou anteriormente
```

### 2. Clonando o Repositório

```bash
cd trabalho2_robotica_2026/ros2_ws/src
git clone https://github.com/Gabrieltlt/trabalho_roboticos.git
```

### 3. Instalando as Dependências

```bash
cd ~/ros2_ws
sudo rosdep init  # Pude se já inicializou
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Compilando o Repositório
```bash
cd /ros_ws
colcon build --symlink-install --packages-select trabalho_roboticos
source install/setup.bash
```

---

## Como rodar

### Inicie a Docker

```bash
./run.sh
```

### Iniciando a Simulação

Em um terminal, rode o comando:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage5.launch.py
```

### Iniciando a Navegação

Em outro terminal, inicie novamente a docker:

```bash
./control_terminal.sh
```
 
E rode o nodo de navegação:

```bash
ros2 run trabalho_roboticos navigator
```
