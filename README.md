# AMiRo_Jevois
Ziel dieses Projektes ist es, eine Demo Applikation für die Anbindung der [Jevois smart machine vision camera](http://jevois.org/) für den AMiRo zu implementieren. Dabei wird die Jevois Kamera über eine serielle Schnittstelle mit dem Cognition Board des AMiRo verbunden. Implementiert werden zwei Demo Applikationen:
1. ArUco Marker Tracking (Multi Marker und Single Marker Tracking)
2. Obstacle Avoidance mittels Optical Flow

# Repo Aufbau:
1. [Jevois_AMiRo](https://github.com/kevinp1993/AMiRo_Jevois/tree/master/Jevois_AMiRo): Software für den AMiRo inklusive cmake List
2. [Jevois_Test](https://github.com/kevinp1993/AMiRo_Jevois/tree/master/Jevois_Test): Testprogramme und zugehörige Makefiles für die Inbetriebnahme der Jevois Kamera auf einem Rechner
3. [Jevois_Software_Changes](https://github.com/kevinp1993/AMiRo_Jevois/tree/master/Jevois_Software_Changes): Geändertes Jevois Programm für das Obstacle Avoidance mittels Optical Flow (Änderung von [OpticalFlow](https://github.com/jevois/jevoisbase/blob/master/src/Modules/OpticalFlow/OpticalFlow.C)). 
4. [media](https://github.com/kevinp1993/AMiRo_Jevois/tree/master/media): ArUco Marker und Dokumentationen

# Jevois Vorbereitung
* Jevois Software auf SD-Card flashen: Anleitungen und Software finden sich auf der [Website](http://jevois.org/start/) und im [Repo](https://github.com/jevois/). Dabei die Änderung im [OpticalFlow](https://github.com/jevois/jevoisbase/blob/master/src/Modules/OpticalFlow/OpticalFlow.C) berücksichtigen. 

# Vorbereitung 
1. [AMiRo_Jevois Repo](https://github.com/kevinp1993/AMiRo_Jevois) auf Rechner clonen 
2. Einrichtung der Cross-Compiler Umgebung mit **poky-glibc-x86_64-meta-toolchain-openrobotix-cortexa8hf-vfp-neon-toolchain-openrobotix-1.7.2.sh** auf dem Rechner. Das benötigte shell script liegt im [ks-temp Ordner](sftp://twix.techfak.uni-bielefeld.de/vol/ks-temp/). 
3. Sourcen des Cross-Compilers:
```
source /opt/poky/1.7.2/environment-setup-cortexa8hf-vfp-neon-poky-linux-gnueabi
```
4. Im Ordner [Jevois_AMiRo](https://github.com/kevinp1993/AMiRo_Jevois/tree/master/Jevois_AMiRo) des AMiRo_Jevois Repo das cmake ausführen:
```
cmake .
```
5. Das erstellte Makefile ausführen:
```
make
```
6. Das AMiRo Cognition Board über die serielle Schnittstelle starten (z.B. mittels gtkterm) und einloggen mit 
* **Benutzer**: *root* 
* **pw**: *amiropower*
7. Das AMiRo Cognition Board über ein Micro-USB Kabel mit dem Rechner verbinden und Netzwerk einrichten  
8. Die drei Programme **jevoisAruco_SingleTracker**, **jevoisAruco_MultiStationTracker** und **jevoisOpticalFlow** können nun  auf das Cognition Board über eine SSH-Schnittstelle kopiert werden. Dazu vorher über die serielle Schnittstelle die IP-Adresse des Cognition Boards herausfinden (standardmäßig ist diese *192.168.1.1*):
```
scp jevoisAruco_SingleTracker root@192.168.1.1:~
scp jevoisAruco_MultiStationTracker root@192.168.1.1:~
scp jevoisOpticalFlow root@192.168.1.1:~
```
9. Über eine SSH-Verbidnung mit dem Cognition Board verbinden:
```
ssh root@192.168.1.1
```
10. Programme können nun auf dem AMiRo ausgeführt werden

# ArUco Marker Tracking
In diesem Demo Programm wird ein 4x4 ArUco Marker getrackt ([ArUco Generator](http://chev.me/arucogen/)). Dabei gibt es zwei Programme, welche mit folgenden Parametern gestartet werden:
```
./jevoisAruco_SingleTracker P_linear P_angular desired_dist desired_Marker_ID start_Marker_ID
./jevoisAruco_MultiStationTracker P_linear P_angular desired_dist start_Marker_ID
```
* P_linear: Konstante für Regler zum Annähern an den Marker, bis gewünschte Distanz erreicht ist (Default Single Tracking: 500)
* P_angular: Konstante für Regler zum mittig Ausrichten an den Marker (Default Single Tracking: 5000)
* desired_dist: Abstand in mm, welcher zum getrackten Marker eingehalten wird (Default Single Tracking: 300)
* desired_Marker_ID: Zu Trackende Marker ID (Default Single Tracking: 42)
* start_Marker_ID: Marker ID, welche für den Start des Programms nötig ist (Default Single Tracking: 0)

Das Single Marker Tracking sucht im Umfeld nach einem gezielten Marker (Default ID:42) und hält beim Auffinden dieses, eine konstante Distanz zu diesem ein. Das Multi Station Marker fährt eine bestimmte Anzahl an Marker in einer vorgegebenen Reihenfolge an, bis der letzte Marker erreicht ist. Die anzufahrenden Marker sind dabei mit ihrer ID in einem Array *desired_Marker_ID_Stations* im Programmcode angelegt. 

Die wahre Markergröße (in mm) kann dabei im Code von [jevoisAruco_SingleTracker.cpp](https://github.com/kevinp1993/AMiRo_Jevois/blob/master/Jevois_AMiRo/jevoisAruco_SingleTracker.cpp) und [jevoisAruco_MultiStationTracker.cpp](https://github.com/kevinp1993/AMiRo_Jevois/blob/master/Jevois_AMiRo/jevoisAruco_MultiStationTracker.cpp) in folgender Zeile geändert werden:
```c
system("echo setpar markerlen 94 > /dev/ttyACM0");
```
Das Marker Tracking auf der Jevois Kamera wird mit folgender Zeile konfiguriert:
```c
system("echo setmapping 5 > /dev/ttyACM0");
```
Die Nummer des Mappings, muss dabei der entsprechenden Ausgabe der Jevois Kamera für das Marker Tracking entsprechen. Dies kann herausgefunden werden, indem über die serielle Schnittstelle z.B. mit:
```
screen /dev/ttyACM0
```
Der Befehl:
```
listmappings
```
eingegeben wird. 

# Obstacle Avoidance mittels Optical Flow
In diesem Demo Programm wird eine Hindernisserkennung und -vermeidung mittels optischen Flusses umgesetzt. Als Grundlage dient das Paper [A Bio-Inspired Model for Visual Collision
Avoidance on a Hexapod Walking Robot
](https://pub.uni-bielefeld.de/record/2905346) von Meyer et al. Das Programm kann mit folgenden Parametern ausgeführt werden:
```
./jevoisOpticalFlow n0 g k v alpha
```
* n0: Threshold (Default: 37,5)
* g: Gain (Default: 5,0)
* k: Verstärkungsfaktor für AMiRo Motor (Default: 3000000)
* v: lineare Geschwindigkeit des AMiRo (Default: 120000)
* alpha: Tiefpass Konstante (Default: 0,2)

Das Programm ist in [jevoisOpticalFlow.cpp](https://github.com/kevinp1993/AMiRo_Jevois/blob/master/Jevois_AMiRo/jevoisOpticalFlow.cpp) hinterlegt.

Der Optical Flow Algorithmus auf der Jevois Kamera wird mit folgender Zeile konfiguriert:
```c
system("echo setmapping 9 > /dev/ttyACM0");
```
Die Nummer des Mappings, muss dabei der entsprechenden Ausgabe der Jevois Kamera für den Optical Flow entsprechen. Dies kann herausgefunden werden, indem über die serielle Schnittstelle z.B. mit:
```
screen /dev/ttyACM0
```
Der Befehl:
```
listmappings
```
eingegeben wird. 

# Hinweise
Wenn die Programme auf dem Cognition Board per SSH gestartet werden und das USB Kabel anschließend entfernt wird, kann es zu Programmabbrüchen bei drahtloser Ausführung des Applikation kommen. Um dies zu umgehen, sollten die Programme per serieller Schnittstelle gestartet werden

# Entwickler/Autoren/Verantwortliche

* [kevinp1993](https://github.com/kevinp1993)
* [mrothmann](https://github.com/mrothmann)
* [tik0](https://github.com/tik0)





