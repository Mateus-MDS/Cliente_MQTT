LarConectado 2.0
Automação Residencial Robusta com Raspberry Pi Pico W e MQTT

Descrição
LarConectado é um sistema de automação residencial inteligente e flexível, desenvolvido com a Raspberry Pi Pico W e o protocolo MQTT (Message Queuing Telemetry Transport). Este projeto aprimora a automação original baseada em interface web, utilizando um broker MQTT centralizado para uma comunicação mais escalável e reativa.

O sistema permite o controle remoto de lâmpadas e de um dispositivo doméstico (simulado por mensagens em um display OLED), além de gerenciar um sistema de alarme completo com detecção de abertura de portas (via joystick) e presença de pessoas próximas (via sensor ultrassônico). O alarme pode ser ativado/desativado de forma remota (via MQTT) ou manual (via botão físico), emitindo uma sirene sonora em caso de violação.

Para feedback visual e monitoramento, o LarConectado integra:

Um display OLED SSD1306 para exibir status (TV, alarme) e notificações.
Sensores ambientais como LDR (luminosidade) e dois sensores ultrassônicos HC-SR04 (proximidade para automação de luzes e detecção de intrusão para o alarme).
Uma matriz de LEDs 5x5 controlada via PIO para representar visualmente o estado das luzes dos cômodos.
LEDs RGB frontais para simular a iluminação de entrada com base nos sensores.
Monitoramento da temperatura interna do chip RP2040.
A interação com o sistema é realizada através de um aplicativo de dashboard MQTT (ex: "IoT MQTT Dashboard"), que fornece uma interface com botões de controle e visualizações de dados em tempo real, como um gráfico da temperatura.

Objetivos:
Controlar remotamente o estado de lâmpadas em múltiplos cômodos e um dispositivo simulado (TV) via mensagens MQTT.
Exibir dados de telemetria (temperatura interna da Pico W, status de alarme) para clientes MQTT.
Monitorar condições ambientais (luminosidade com LDR e presença com sensor ultrassônico) para automações de luzes.
Representar graficamente o status das luzes dos cômodos em uma matriz de LEDs 5x5 via PIO.
Exibir status e notificações no display OLED SSD1306.
Implementar um sistema de alarme com ativação/desativação remota (MQTT) e manual (botão físico), acionando uma sirene (buzzer) em caso de detecção de movimento (joystick) ou presença (sensor ultrassônico).
Utilizar um broker MQTT (Mosquitto) para desacoplar a interface de controle do dispositivo embarcado.
Prover uma interface de usuário rica através de um aplicativo de dashboard MQTT com botões interativos e gráficos de dados.

Componentes Utilizados:
Componente	Pinos	Função
Raspberry Pi Pico W	-	Microcontrolador com Wi-Fi, executando o cliente MQTT.
Display OLED SSD1306	I2C1 ? GP14 (SDA), GP15 (SCL)	Exibição local de status (TV, alarme: ligado/desligado/acionado) e notificações.
Sensor Ultrassônico_1 HC-SR04	GP8 (TRIG), GP9 (ECHO)	Detecção de proximidade para automação das luzes RGB frontais.
Sensor Ultrassônico_2 HC-SR04	GP18 (TRIG), GP19 (ECHO)	Detecção de proximidade para acionamento do alarme (se ativado).
Sensor de Luminosidade (LDR)	GPIO16	Detecção de luz ambiente para automação das luzes RGB frontais.
Matriz de LEDs 5×5	PIO ? GP7	Visualização gráfica em tempo real do status das luzes dos cômodos.
Buzzer	GP21	Emite a sirene sonora do alarme quando acionado.
Botão A	GP5	Ativação e desativação manual do alarme.
Joystick	GP26 (ADC0 - Eixo X), GP27 (ADC1 - Eixo Y)	Simula a detecção de movimento/abertura em portas para o sistema de alarme.
LEDs RGB de Status	GPIO11, GPIO12, GPIO13	Representam uma lâmpada externa de entrada, com acendimento automático baseado em condições de escuro (LDR) e proximidade (Ultrassônico_1).
LED WiFi	CYW43 embutido	Feedback visual do status da conexão Wi-Fi.
Mosquitto Broker	Host de rede (ex: 10.20.20.16)	Servidor MQTT central que gerencia e distribui as mensagens entre o Pico W e as interfaces de controle (aplicativos/dashboards). Neste projeto, simulado via Termux em Android.
IoT MQTT Dashboard App	Smartphone/Tablet	Interface de usuário móvel para controle e monitoramento remoto. Permite acionar dispositivos via botões e visualizar dados de sensores (como temperatura em gráfico) em tempo real, comunicando-se via MQTT.

Funcionamento:
Inicialização
Configuração de GPIOs, ADCs, PIO e periféricos (ssd1306, buzzer, joystick).
Inicialização do display OLED.
Conexão à rede Wi-Fi.
Resolução DNS do endereço do broker MQTT.
Inicialização do cliente MQTT e conexão ao broker.
Inscrição nos tópicos de comando (/luz_sala, /alarme, etc.).
Publicação do estado online = 1 (Last Will).
Interação MQTT
Comandos de Controle: O Raspberry Pi Pico W assina tópicos como /luz_sala, /display, /alarme. Quando um cliente MQTT (ex: o aplicativo IoT MQTT Dashboard) publica mensagens nesses tópicos (e.g., "On", "Off", "1"), o Pico W recebe o comando e atualiza o estado do dispositivo correspondente (liga/desliga LED, altera status do display/alarme).

Publicação de Status: O Pico W publica o estado atual de seus dispositivos e sensores em tópicos específicos:
/luz_[comodo]/state: "On" ou "Off" para cada LED de cômodo.
/display/state: "On" ou "Off" para o display (representando a TV).
/alarme/state: "On" (ligado) ou "Off" (desligado) para o alarme.
/alarme/acionado: "VIOLACAO" ou "SEM VIOLACAO" em caso de detecção de intrusão.
/temperature: Valor da temperatura interna do RP2040.
/online: "1" (quando conectado) ou "0" (Last Will, se desconectado inesperadamente).
/uptime: Tempo de atividade do sistema.

Interface de Controle (IoT MQTT Dashboard): O aplicativo no smartphone/tablet se conecta ao mesmo broker Mosquitto. Ele possui botões configurados para publicar comandos e painéis/gráficos configurados para assinar os tópicos de status, exibindo os dados em tempo real.
Leitura e Monitoramento
Sensor Ultrassônico_1: Mede a distância periodicamente para automação das luzes RGB frontais (combinação com LDR).
Sensor Ultrassônico_2: Mede a distância periodicamente para o sistema de alarme, detectando presença próxima.
Sensor LDR: Leitura analógica para avaliar a luminosidade ambiente, influenciando o acendimento das luzes RGB frontais.
Joystick: Leitura dos valores analógicos dos eixos X e Y para detectar mudanças de posição, simulando a abertura de portas para o alarme.
Temperatura Interna: Leitura do sensor térmico do RP2040 publicada via MQTT.

Estrutura do Código:
Wi-Fi & lwIP Setup:
Inicializa a interface de rede (CYW43).
Conecta-se à rede Wi-Fi.
Realiza a resolução DNS do broker MQTT.

MQTT Client:
Cria e gerencia a conexão MQTT.
Define callbacks para conexão, recebimento de mensagens (tópico e dados) e confirmação de publicações/inscrições.
Gerencia a publicação de estados e telemetria.

Drivers:
OLED SSD1306 via I²C.
Matriz de LEDs via PIO.
Leitura de ADCs (LDR, Joystick, Temperatura Interna) e sensores digitais (Ultrassônicos, Botão).

Lógica de Controle:
Funções para ligar/desligar LEDs, TV (display).
Funções para o sistema de alarme (ativação/desativação, detecção de violação, acionamento do buzzer).
Lógica para luzes automáticas frontais.

Main Loop:
Processa eventos de rede (CYW43_arch_poll).
Atualiza leituras de sensores.
Atualiza OLED e LED matrix.
Verifica e processa a lógica do Alarme.
Publica periodicamente estados e dados de sensores.

Como Executar o Projeto:
Montagem: Monte os componentes eletrônicos conforme a tabela de pinos e as especificações do seu hardware.
Configuração do Mosquitto Broker:
Em um dispositivo Android, instale o aplicativo Termux.
No Termux, instale o Mosquitto: pkg install mosquitto. Inicio e configure.
Anote o endereço IP do seu dispositivo Android para configurar o MQTT_SERVER no código da Pico W.

Configuração do Firmware da Pico W:
Configure suas credenciais de Wi-Fi (WIFI_SSID, WIFI_PASSWORD) no código-fonte.
Configure o endereço IP do seu Mosquitto Broker (MQTT_SERVER, MQTT_USERNAME, MQTT_PASSWORD) no código.

Compilação e Gravação:
Certifique-se de ter o SDK do Raspberry Pi Pico configurado.
Compile o firmware (.uf2) do projeto.
Grave o firmware compilado na sua Raspberry Pi Pico W.

Configuração do IoT MQTT Dashboard:
Instale o aplicativo "IoT MQTT Dashboard" (ou similar) em seu smartphone/tablet.
Configure uma nova conexão apontando para o IP do seu Mosquitto Broker.
Crie botões e painéis/gráficos no aplicativo, configurando-os para publicar e assinar os tópicos MQTT conforme a lógica do seu projeto (ex: publicar "On" em /luz_sala, assinar /temperature para o gráfico).

Interação:
Aguarde a Pico W conectar-se ao Wi-Fi e ao broker MQTT.
Use o aplicativo IoT MQTT Dashboard para controlar LEDs, acionar/desativar o alarme e visualizar os dados dos sensores em tempo real.

Requisitos:
Raspberry Pi Pico W
SDK do Raspberry Pi Pico configurado
Display OLED compatível com SSD1306
Sensor LDR
Sensor Ultrassônico HC-SR04 (x2)
Matriz de LEDs 5×5
Buzzer
Botão
Joystick
Dispositivo Android com Termux para rodar o Mosquitto Broker (ou um Mosquitto Broker em outro PC/servidor)
Aplicativo "IoT MQTT Dashboard" (ou similar) para smartphone/tablet

Repositório:
GitHub: 

Autor
Nome: Mateus Moreira da Silva