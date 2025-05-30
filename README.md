# TFG-IoT-Suelo

# Sistema IoT para la Monitorización de Parámetros del Suelo 🌱📡

Este repositorio contiene el código fuente y recursos del Trabajo de Fin de Grado desarrollado por Liming Yang en la Universitat de València. El sistema permite medir parámetros del suelo (temperatura, humedad, pH, NPK, etc.) y visualizar los datos mediante un servidor local.

## 🧩 Componentes del sistema

- ESP32-S3 (LilyGO T-SIM7080G)
- Sensor de suelo 7-en-1 RS485
- Panel solar 20W + batería 18650
- Servidor Ubuntu con Mosquitto, Node-RED, MariaDB y Grafana

## 📁 Estructura del repositorio

- `firmware/`: Código del microcontrolador ESP32-S3
- `server/`: Scripts de creación de base de datos y configuración de servicios
- `nodered/`: Flujo Node-RED exportado (.json)
- `docs/`: Diagramas, capturas de pantalla, documentación adicional

## 🖥️ Visualización

Los datos se visualizan mediante Grafana con una consulta SQL sobre la base de datos MariaDB.

## 📚 Memoria TFG

Este repositorio complementa la memoria del TFG, disponible en formato PDF y presentada en la Escuela Técnica Superior de Ingeniería (ETSE).

