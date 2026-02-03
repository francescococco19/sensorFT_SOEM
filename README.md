# sensorFT_SOEM / ft_sensor_ros — Guida rapida (ROS1 + SOEM)

Questo package ROS1 legge un sensore **ATI Axia Force/Torque** su **EtherCAT** usando **SOEM** e pubblica:

- **Topic:** `/ft_sensor/wrench`
- **Msg:** `geometry_msgs/WrenchStamped`
- **frame_id:** `ft_sensor` (configurabile)

> Requisito: **ROS1 (Noetic) già installato** sul PC.

---

## 1) Installazione SOEM (una sola volta)

Installa dipendenze:
```bash
sudo apt update
sudo apt install -y git cmake build-essential ethtool
```

Clona e compila SOEM:
```bash
cd ~
git clone https://github.com/OpenEtherCATsociety/SOEM.git
cd SOEM
git checkout v1.4.0
mkdir -p build && cd build
cmake ..
make -j"$(nproc)"
```

Verifica che esista:
- `~/SOEM/build/libsoem.a`
- `~/SOEM/soem/ethercat.h`

---

## 2) Preparazione rete EtherCAT (ogni PC / ogni cambio interfaccia)

1) Collega il sensore e individua l’interfaccia di rete:
```bash
ip -br link
```
Esempi comuni: `enx...` (USB-RJ45), `eth0`, `enpXsY`.

2) Disabilita gli offload (consigliato, soprattutto con adattatori USB):
```bash
sudo ethtool -K <iface> gro off gso off tso off sg off
```

3) (Opzionale) Controlla link:
```bash
ethtool <iface> | egrep "Speed|Duplex|Link detected"
```

---

## 3) Build del package nel workspace catkin

Esempio con workspace `~/test_ws` (puoi usare qualsiasi nome):
```bash
mkdir -p ~/test_ws/src
cd ~/test_ws/src
git clone https://github.com/francescococco19/sensorFT_SOEM.git
cd ~/test_ws
source /opt/ros/noetic/setup.bash
catkin_make
```

---

## 4) Avvio del nodo (metodo consigliato)

Per EtherCAT spesso servono privilegi (raw socket). Per evitare problemi con librerie ROS sotto `sudo`,
si usa un avvio “isolato” tramite script.

### 4.1 Avvia roscore (utente normale)
Terminale 1:
```bash
source /opt/ros/noetic/setup.bash
roscore
```

### 4.2 Avvia il sensore tramite script
Terminale 2:
```bash
chmod +x ~/test_ws/src/ft_sensor_ros/scripts/sensorFT.sh
~/test_ws/src/ft_sensor_ros/scripts/sensorFT.sh --iface <iface>
```

Esempio:
```bash
~/test_ws/src/ft_sensor_ros/scripts/sensorFT.sh --iface enx207bd51ab7ad
```

Lo script lancia `roslaunch ft_sensor_ros sensorFT.launch` come root *solo* per il processo EtherCAT,
senza modificare l’ambiente ROS globale.

---

## 5) Verifica pubblicazione dati

Terminale 3:
```bash
source /opt/ros/noetic/setup.bash
rostopic hz /ft_sensor/wrench
rostopic echo /ft_sensor/wrench
```

---

## 6) Parametri utili (facoltativi)

Puoi cambiare parametri passando opzioni allo script:

- Frequenza:
```bash
~/test_ws/src/ft_sensor_ros/scripts/sensorFT.sh --iface <iface> --rate 100
```

- Tara più “robusta” (più campioni):
```bash
~/test_ws/src/ft_sensor_ros/scripts/sensorFT.sh --iface <iface> --tare-samples 500 --warmup 20
```

---

## Troubleshooting rapido

- **`ec_init failed` / nessuno slave trovato**
  - controlla `iface`, cablaggio, alimentazione
  - disabilita offload
  - prova test SOEM:
    ```bash
    cd ~/SOEM/build
    sudo ./test/linux/slaveinfo/slaveinfo <iface>
    ```

- **Offset residuo (es. 0.1–0.3 N) a riposo**
  - cavi in tensione o montaggio meccanico
  - lascia warm-up più lungo (`--warmup 20`)
  - aumenta `--tare-samples`

---
