# TurtleBot Gazebo Otonom Navigasyon Projesi

Bu proje, **Gazebo simülasyon ortamında TurtleBot** kullanılarak geliştirilen, **A*** tabanlı global yol planlama, **MPC (Model Predictive Control)** tabanlı yerel kontrol ve **reaktif engelden kaçış** mekanizmalarını bir arada içeren bir **otonom araç (mobil robot)** uygulamasıdır.

Proje, tipik bir robotik dersinde beklenen mimariyi takip eder ve **ROS (Robot Operating System)** altyapısı üzerine kuruludur.

---

## 1. Proje Amaçları

* Gazebo ortamında TurtleBot’un otonom hareketini sağlamak
* Harita üzerinde **A*** algoritması ile en kısa yolun hesaplanması
* Hesaplanan yolu **MPC** ile yumuşak ve kısıtlı biçimde takip etmek
* Lidar verisi kullanarak **dinamik ve statik engellerden kaçınmak**
* Modüler ve anlaşılabilir bir ROS düğüm (node) mimarisi kurmak

---

## 2. Kullanılan Teknolojiler

* **ROS Noetic**
* **Gazebo**
* **TurtleBot3 (Burger veya Waffle)**
* **Python (rospy)**
* **NumPy / SciPy**
* **nav_msgs / geometry_msgs / sensor_msgs**

---

## 3. Sistem Mimarisi

Sistem üç ana katmandan oluşur:

```
[ Occupancy Grid Map ]
          ↓
   A* Global Planner
          ↓
     Waypoint List
          ↓
   MPC Local Controller
          ↓
     /cmd_vel
          ↑
  Obstacle Avoidance (Lidar)
```

### 3.1 Global Yol Planlama – A*

* Girdi: `nav_msgs/OccupancyGrid`
* Çıktı: Waypoint listesi (global path)
* Robotun başlangıç ve hedef noktası arasında en kısa yolu hesaplar

### 3.2 Yerel Kontrol – MPC

* Robotun kinematik modelini kullanır
* Hız, açısal hız ve ivme kısıtlarını dikkate alır
* A*’tan gelen waypoint’leri yumuşak şekilde takip eder

### 3.3 Engelden Kaçış

* `/scan` (Lidar) verisi kullanılır
* Kritik mesafe altında engel algılanırsa:

  * MPC geçici olarak devre dışı bırakılır
  * Reaktif dönüş veya yavaşlama uygulanır

---

## 4. ROS Düğüm Yapısı

| Node Adı                | Açıklama                            |
| ----------------------- | ----------------------------------- |
| `a_star_planner.py`     | Global A* yol planlama              |
| `mpc_controller.py`     | MPC tabanlı hız kontrolü            |
| `obstacle_avoidance.py` | Lidar ile engelden kaçış            |
| `main_navigation.py`    | Tüm sistemi koordine eden ana düğüm |

---

## 5. Topic ve Mesajlar

### Subscribe Edilen Topic’ler

* `/map` → `nav_msgs/OccupancyGrid`
* `/odom` → `nav_msgs/Odometry`
* `/scan` → `sensor_msgs/LaserScan`

### Publish Edilen Topic’ler

* `/cmd_vel` → `geometry_msgs/Twist`
* `/global_path` → `nav_msgs/Path` (opsiyonel görselleştirme)

---
<img width="1600" height="900" alt="image" src="https://github.com/user-attachments/assets/f7932b37-6ab6-4d6b-b47c-d5f3a877f773" />


## 6. Çalışma Akışı

1. Gazebo ortamı başlatılır
2. Harita yüklenir veya SLAM çalıştırılır
3. Başlangıç ve hedef noktası belirlenir
4. A* algoritması global yolu üretir
5. MPC bu yolu takip eder
6. Engel algılanırsa reaktif kaçış devreye girer
7. Engel ortadan kalkınca MPC devam eder

---

## 7. Klasör Yapısı (Önerilen)

```
catkin_ws/
 └── src/
     └── turtlebot_autonomy/
         ├── scripts/
         │   ├── a_star_planner.py
         │   ├── mpc_controller.py
         │   ├── obstacle_avoidance.py
         │   └── main_navigation.py
         ├── launch/
         │   └── navigation.launch
         ├── README.md
         └── package.xml
```

---

## 8. Değerlendirme Kriterlerine Uygunluk

Bu proje:

* ✔ Otonom karar verme içerir
* ✔ Global + local planner ayrımı vardır
* ✔ Kontrol teorisi (MPC) kullanır
* ✔ Sensör verisiyle çevre algılar
* ✔ Genişletilebilir ve akademik olarak temizdir

---

## 9. Sonraki Adım

Bir sonraki aşamada:

* **A*** algoritmasının Python kodu
* **MPC denklemleri ve ROS implementasyonu**
* **Lidar tabanlı engelden kaçış kodu**
* **Tüm sistemi bağlayan launch dosyası**

adım adım verilecektir.

---

## 10. Kurulum ve Çalıştırma Adımları

Bu bölüm, projeyi **sıfırdan çalışan bir Gazebo + TurtleBot otonom navigasyon sistemi** haline getirmek için gereken tüm adımları içerir.

---

### 10.1 Sistem Gereksinimleri

* **Ubuntu 20.04 LTS** (zorunlu)
* **ROS Noetic**
* Python 3.8+
* En az 8 GB RAM (Gazebo için önerilir)

---

### 10.2 ROS Noetic Kurulumu

```bash
sudo apt update
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt update
sudo apt install ros-noetic-desktop-full
```

ROS ortam değişkenlerini ekleyin:

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### 10.3 Catkin Workspace Oluşturma

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

### 10.4 TurtleBot3 Paketlerinin Kurulumu

```bash
sudo apt install ros-noetic-turtlebot3
sudo apt install ros-noetic-turtlebot3-simulations
```

TurtleBot modelini ayarlayın (Burger önerilir):

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

---

### 10.5 Proje Paketinin Oluşturulması

```bash
cd ~/catkin_ws/src
catkin_create_pkg turtlebot_autonomy rospy std_msgs geometry_msgs nav_msgs sensor_msgs
```

Script klasörünü çalıştırılabilir yapın:

```bash
cd turtlebot_autonomy
mkdir scripts launch
chmod +x scripts/*.py
```

---

### 10.6 Python Bağımlılıkları

```bash
sudo apt install python3-numpy python3-scipy python3-matplotlib
```

(İsteğe bağlı – MPC için QP çözümü)

```bash
pip3 install cvxpy osqp
```

---

### 10.7 Kodların Yerleştirilmesi

Aşağıdaki dosyaları `scripts/` klasörü altına koyun:

* `a_star_planner.py`
* `mpc_controller.py`
* `obstacle_avoidance.py`
* `main_navigation.py`

Launch dosyasını:

* `launch/navigation.launch`

---

### 10.8 Projenin Derlenmesi

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

### 10.9 Gazebo Ortamının Başlatılması

Terminal 1:

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

---

### 10.10 Otonom Navigasyonun Çalıştırılması

Terminal 2:

```bash
roslaunch turtlebot_autonomy navigation.launch
```

---

### 10.11 Çalışmanın Doğrulanması

* Robot hedefe doğru hareket ediyorsa
* Engel konulduğunda yavaşlayıp yön değiştiriyorsa
* Engel kalkınca yoluna devam ediyorsa

sistem doğru çalışıyor demektir.

---

<img width="1600" height="738" alt="image" src="https://github.com/user-attachments/assets/28345143-c24c-4986-a5fb-8158ee58db0c" />


### 10.12 Sık Karşılaşılan Hatalar

* `/cmd_vel` publish edilmiyorsa → node izinlerini (`chmod +x`) kontrol edin
* Gazebo yavaşsa → `real_time_factor` düşürün
* Robot hareket etmiyorsa → `/odom` ve `/scan` topic’lerini kontrol edin

---
