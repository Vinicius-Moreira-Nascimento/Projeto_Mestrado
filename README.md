# ⚡ Embedded Multi-Sensor Reader - ESP32 com INA219 + ADC Interno

Projeto embarcado desenvolvido em C para o **ESP32-S3**, que realiza **leitura de tensão e corrente** com o sensor **INA219**, além de leitura com o **ADC interno** e exibição dos dados via **UART**.

---

## 🚀 Funcionalidades

- ✅ Leitura de tensão da linha (bus voltage) com o **INA219**
- ✅ Leitura de corrente com o INA219 (via shunt)
- ✅ Leitura de tensão via **ADC interno calibrado**
- ✅ Indicação de atividade com **4 LEDs**
- ✅ Comunicação com PC via **UART1**
- ✅ Arquitetura baseada em **FreeRTOS**

---

## 📦 Estrutura do Projeto

```
├── main.c              # Código principal com leitura e comunicação  
├── include/            # (separar headers aqui é recomendável em projetos grandes)  
├── CMakeLists.txt      # Configuração para build no ESP-IDF  
├── README.md           # Este arquivo  
```

---

## 🧠 Visão Geral do Sistema

- **LEDs (GPIOs 19, 20, 21 e 47):** piscam em sequência para indicar o funcionamento da tarefa principal.  
- **INA219:** lê tensão de barramento, tensão de shunt e corrente (em mA).  
- **ADC Interno:** realiza leitura bruta e calibrada da tensão em um canal analógico.  
- **UART1 (GPIOs 43/44):** transmite os dados lidos para o terminal serial.  

---

## ⚙️ Definições de Pinos

| Função            | GPIO       |
|------------------|------------|
| LED1             | 19         |
| LED2             | 20         |
| LED3             | 21         |
| LED4             | 47         |
| UART1 TX         | 43         |
| UART1 RX         | 44         |
| I2C SDA (INA219) | 5          |
| I2C SCL (INA219) | 4          |
| ADC Interno      | Canal 2    |

---

## 📡 Comunicação UART

Os dados são transmitidos via UART no seguinte formato:

```
Tensão convertida: 1.6500V  
Tensão Calibrada: 1650mV  
Bus Voltage: 3.28 V  
Shunt Voltage: 1.20 mV  
Current: 25.00 mA  
```

---

## 🛠️ Compilação e Upload

### Pré-requisitos

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html)  
- Placa **ESP32-S3**  
- Sensor **INA219**  
- Terminal serial (PuTTY, TeraTerm ou `idf.py monitor`)  

### Build

```
idf.py set-target esp32s3  
idf.py build  
idf.py flash monitor  
```

---

## ⏱️ Tarefas RTOS

| Nome da Tarefa       | Prioridade | Função                                |
|----------------------|------------|----------------------------------------|
| `xLED`               | 2          | Piscar LEDs sequencialmente            |
| `xLeituraINA333`     | 1          | Leitura via ADC interno                |
| `xLeituraINA219`     | 2          | Leitura via sensor INA219              |

---

## 🌱 Expansões Futuras

- ☑ Registro em cartão SD  
- ☑ Envio via MQTT (Wi-Fi)  
- ☑ Integração com display OLED  
- ☑ Suporte a sensores de temperatura  

---

## 📄 Licença

Distribuído sob a licença MIT.

---

## ✍️ Autor

Desenvolvido por **[Vinicius Moreira Nascimento]**  
Entre em contato e contribua!
