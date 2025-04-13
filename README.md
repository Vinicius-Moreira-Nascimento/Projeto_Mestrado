🔌 Projeto de Mestrado: Monitoramento Inteligente de Energia com ESP32
ESP32 Banner

📝 Descrição do Projeto
Este projeto implementa um sistema avançado de monitoramento de energia utilizando o microcontrolador ESP32, com capacidade de:

Leitura precisa de tensão e corrente via sensor INA219

Conversão analógica-digital de alta precisão

Comunicação serial para debug e monitoramento

Controle de LEDs indicadores

Operação em tempo real com FreeRTOS

🛠️ Componentes Principais
Componente	Função
ESP32	Microcontrolador principal
INA219	Sensor de corrente/tensão
ADC interno	Leitura de sinais analógicos
UART	Comunicação serial
LEDs GPIO	Indicadores visuais
📊 Funcionalidades
c
Copy
void xLeituraINA219(void *arg) {
    // Implementa leitura precisa de:
    // - Tensão do barramento
    // - Tensão do shunt
    // - Corrente elétrica
    // Envia dados via UART
}
🚀 Como Usar
Hardware

Conecte o INA219 via I2C (GPIO4-SCL, GPIO5-SDA)

Configure os LEDs nos GPIOs 19, 20, 21 e 47

Conecte o UART1 (GPIO43-TX, GPIO44-RX)

Software

bash
Copy
# Clone o repositório
git clone https://github.com/Vinicius-Moreira-Nascimento/Projeto_Mestrado.git

# Compile e flash (usando ESP-IDF)
idf.py build flash monitor
📈 Estrutura do Código
mermaid
Copy
graph TD
    A[app_main] --> B[Inicializa I2C]
    A --> C[Configura GPIOs]
    A --> D[Configura UART]
    A --> E[Cria Tasks RTOS]
    E --> F[xLED]
    E --> G[xLeituraINA333]
    E --> H[xLeituraINA219]
📌 Especificações Técnicas
Parâmetro	Valor
Resolução ADC	12 bits
Taxa I2C	100kHz
Baud Rate UART	115200
Frequência CPU	240MHz
🌟 Recursos Avançados
Multi-core processing com FreeRTOS

Calibração digital do ADC

Comunicação assíncrona entre tasks

Gerenciamento de energia eficiente

📚 Documentação Adicional
Datasheet ESP32

Manual INA219

Guia FreeRTOS

👨‍💻 Autor
Vinícius Moreira Nascimento
GitHub
Email

📜 Licença
Este projeto está licenciado sob a licença MIT - veja o arquivo LICENSE para detalhes.

<div align="center"> <img src="https://raw.githubusercontent.com/Vinicius-Moreira-Nascimento/Projeto_Mestrado/main/assets/esp32-diagram.png" width="400"> <p>Diagrama simplificado da arquitetura</p> </div>
💡 Dica profissional: Use idf.py monitor para ver os dados de tensão/corrente em tempo real!