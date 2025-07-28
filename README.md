# Carrinho de drift utilizando ESP32-C3, BLE e FlutterFlow

![[RC car]](https://i.imgur.com/usEm5dq.jpeg)

Um projeto de carrinho de controle remoto miniatura 1:48 para drifts utilizando BLE do esp32c3 pelo ESP-IDF.

Neste repositório estara apenas o firmware do carrinho, sem arquivos de PCB ou aplicativo.

---

## Features Principais

-   **Controle Preciso:** Direção e aceleração proporcionais para manobras de drift controladas.
-   **Conectividade Moderna:** Comunicação via Bluetooth Low Energy (BLE) para baixo consumo e resposta rápida.
-   **Hardware Customizado:** Uma PCB projetada especificamente para este projeto, otimizando espaço e conexões.
---

## Tecnologias Utilizadas

-   **Hardware:**
    -   Microcontrolador: **ESP32-C3**
    -   Motor coreless: `[2010 12-6v 18000RPM]`
    -   Bateria: `[Ex: LiPo 1S 3.7V]`
-   **Firmware (ESP32):**
    -   Linguagem: **C**
    -   Framework: **ESP-IDF**
    -   IDE: **ESP-IDF** com Visual Studio Code
-   **Design da PCB:**
    -   Software: `[Ex: Proteus 8]`
-   **Aplicativo Mobile:**
    -   Plataforma: **FlutterFlow**
    -   Linguagem: **Dart**
    -   Framework: **Flutter**

---
### Placa de Circuito Impresso (PCB)

A PCB foi desenhada no `Proteus` para integrar todos os componentes de forma compacta.
Design customizado baseado na placa do esp32c3 super mini.

![pcb proteus](https://i.imgur.com/oKrwqQO.png)
![pcb montada](https://i.imgur.com/hwDu0bI.png)
---

##  Firmware (ESP32-C3)

O firmware é responsável por receber os comandos do aplicativo via BLE e controlar os motores e o servo.


##  Melhorias Futuras (To-Do)

-   [ ]  Refazer placa para chip comportar Bluetooth classic
-   [ ]  Melhora na modelagem 3D e impressão de resina
-   [ ]  Diminuir para 1:64 e virar um kit de montagem de RC car para hotwheels
-   [ ]  Otimizar o consumo de energia da bateria.
---

##  Contato
`[Leonardo Borbon]` - `[leo.rborbon@gmail.com]`
Link do Projeto: `[https://github.com/seu_usuario/seu_repositorio]`
