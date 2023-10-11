# Optimización con emociones 

Optimización de neurocontroladores, ya sea con emociones o punto de referencia. El neurocontrolador es una red neuronal (perceptrón) que conduce un robot móvil dentro de un entorno doméstico simulado.
El sistema captura las emociones a partir del EEG o por medio de la herramienta SAM (controles de valence y arousal). Para la captura de las emociones con el EEG se utilizan redes neuronales convolucionales.
El entrenamiento de las redes se lleva a cabo con señales EEG, que se capturan durante la ejecución de un protocolo de estimulación con videos musicales y videos del robot en la simulación.


Instalación: 

El sistema está compuesto por varias tecnologías, las cuales a continuación se detalla como instalarlo.

1. Instalar ROS Noetic en Ubuntu 20.04: https://wiki.ros.org/noetic/Installation/Ubuntu
2. Instalar nvm (Node Version Manager): curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.3/install.sh | bash
3. comprobar instalación: nvm --version
4. Instalar NodeJs version 17.5.0: nvm install 17.5.0
5. Instalar pip, el método utilizado es por medio del archivo get-pip.py, ver link para más detalles https://pandas.pydata.org/docs/getting_started/install.html
6. Si aparece un warning después de la instalación de pip indicando que se instaló en una ruta que no es el PATH, se exporta con: export PATH="/home/{usuario}/.local/bin" 
7. Instalar pandas: pip install pandas
8. Descargar el repositorio de brainflow: https://github.com/brainflow-dev/brainflow
9. Renombrar la carpeta principal del repositorio por brainflow y pasarla al /home
10. Ubicarse en cd ~/brainflow/tools
11. instalar con pyhon3: python3 build.py --cmake-install-prefix=~/brainflow
12. Exportar la ruta de brainflow: export CMAKE_PREFIX_PATH="${HOME}/brainflow:${CMAKE_PREFIX_PATH}"
13. Instalar playsound: pip install playsound
14. Instalar pygame: python3 -m pip install -U pygame --user
15. Para graficar los resultados en un notebook: pip install jupyterlab
16. Instalar pdf2image: pip install pdf2image

Para instalar las dependencias de los nodos desarrollados con NodeJS

1. Dirigirse al directorio: cd ~/catkin_ws/src/experiment_control_interfaces/
2. Instalar dependencias: npm install
3. Dirigirse al directorio:  cd ~/catkin_ws/src/user_interaction/
4. Instalar dependencias: npm install

IMPORTANTE:
El nombre del la carpeta del repositorio (previamente descargado) debe renombrase por "catkin_ws" y pasarla al /home
