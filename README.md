LarConectado 2.0
Automa��o Residencial Robusta com Raspberry Pi Pico W e MQTT

Descri��o
LarConectado � um sistema de automa��o residencial inteligente e flex�vel, desenvolvido com a Raspberry Pi Pico W e o protocolo MQTT (Message Queuing Telemetry Transport). Este projeto aprimora a automa��o original baseada em interface web, utilizando um broker MQTT centralizado para uma comunica��o mais escal�vel e reativa.

O sistema permite o controle remoto de l�mpadas e de um dispositivo dom�stico (simulado por mensagens em um display OLED), al�m de gerenciar um sistema de alarme completo com detec��o de abertura de portas (via joystick) e presen�a de pessoas pr�ximas (via sensor ultrass�nico). O alarme pode ser ativado/desativado de forma remota (via MQTT) ou manual (via bot�o f�sico), emitindo uma sirene sonora em caso de viola��o.

Para feedback visual e monitoramento, o LarConectado integra:

Um display OLED SSD1306 para exibir status (TV, alarme) e notifica��es.
Sensores ambientais como LDR (luminosidade) e dois sensores ultrass�nicos HC-SR04 (proximidade para automa��o de luzes e detec��o de intrus�o para o alarme).
Uma matriz de LEDs 5x5 controlada via PIO para representar visualmente o estado das luzes dos c�modos.
LEDs RGB frontais para simular a ilumina��o de entrada com base nos sensores.
Monitoramento da temperatura interna do chip RP2040.
A intera��o com o sistema � realizada atrav�s de um aplicativo de dashboard MQTT (ex: "IoT MQTT Dashboard"), que fornece uma interface com bot�es de controle e visualiza��es de dados em tempo real, como um gr�fico da temperatura.

Objetivos:
Controlar remotamente o estado de l�mpadas em m�ltiplos c�modos e um dispositivo simulado (TV) via mensagens MQTT.
Exibir dados de telemetria (temperatura interna da Pico W, status de alarme) para clientes MQTT.
Monitorar condi��es ambientais (luminosidade com LDR e presen�a com sensor ultrass�nico) para automa��es de luzes.
Representar graficamente o status das luzes dos c�modos em uma matriz de LEDs 5x5 via PIO.
Exibir status e notifica��es no display OLED SSD1306.
Implementar um sistema de alarme com ativa��o/desativa��o remota (MQTT) e manual (bot�o f�sico), acionando uma sirene (buzzer) em caso de detec��o de movimento (joystick) ou presen�a (sensor ultrass�nico).
Utilizar um broker MQTT (Mosquitto) para desacoplar a interface de controle do dispositivo embarcado.
Prover uma interface de usu�rio rica atrav�s de um aplicativo de dashboard MQTT com bot�es interativos e gr�ficos de dados.

Componentes Utilizados:
Componente	Pinos	Fun��o
Raspberry Pi Pico W	-	Microcontrolador com Wi-Fi, executando o cliente MQTT.
Display OLED SSD1306	I2C1 ? GP14 (SDA), GP15 (SCL)	Exibi��o local de status (TV, alarme: ligado/desligado/acionado) e notifica��es.
Sensor Ultrass�nico_1 HC-SR04	GP8 (TRIG), GP9 (ECHO)	Detec��o de proximidade para automa��o das luzes RGB frontais.
Sensor Ultrass�nico_2 HC-SR04	GP18 (TRIG), GP19 (ECHO)	Detec��o de proximidade para acionamento do alarme (se ativado).
Sensor de Luminosidade (LDR)	GPIO16	Detec��o de luz ambiente para automa��o das luzes RGB frontais.
Matriz de LEDs 5�5	PIO ? GP7	Visualiza��o gr�fica em tempo real do status das luzes dos c�modos.
Buzzer	GP21	Emite a sirene sonora do alarme quando acionado.
Bot�o A	GP5	Ativa��o e desativa��o manual do alarme.
Joystick	GP26 (ADC0 - Eixo X), GP27 (ADC1 - Eixo Y)	Simula a detec��o de movimento/abertura em portas para o sistema de alarme.
LEDs RGB de Status	GPIO11, GPIO12, GPIO13	Representam uma l�mpada externa de entrada, com acendimento autom�tico baseado em condi��es de escuro (LDR) e proximidade (Ultrass�nico_1).
LED WiFi	CYW43 embutido	Feedback visual do status da conex�o Wi-Fi.
Mosquitto Broker	Host de rede (ex: 10.20.20.16)	Servidor MQTT central que gerencia e distribui as mensagens entre o Pico W e as interfaces de controle (aplicativos/dashboards). Neste projeto, simulado via Termux em Android.
IoT MQTT Dashboard App	Smartphone/Tablet	Interface de usu�rio m�vel para controle e monitoramento remoto. Permite acionar dispositivos via bot�es e visualizar dados de sensores (como temperatura em gr�fico) em tempo real, comunicando-se via MQTT.

Funcionamento:
Inicializa��o
Configura��o de GPIOs, ADCs, PIO e perif�ricos (ssd1306, buzzer, joystick).
Inicializa��o do display OLED.
Conex�o � rede Wi-Fi.
Resolu��o DNS do endere�o do broker MQTT.
Inicializa��o do cliente MQTT e conex�o ao broker.
Inscri��o nos t�picos de comando (/luz_sala, /alarme, etc.).
Publica��o do estado online = 1 (Last Will).
Intera��o MQTT
Comandos de Controle: O Raspberry Pi Pico W assina t�picos como /luz_sala, /display, /alarme. Quando um cliente MQTT (ex: o aplicativo IoT MQTT Dashboard) publica mensagens nesses t�picos (e.g., "On", "Off", "1"), o Pico W recebe o comando e atualiza o estado do dispositivo correspondente (liga/desliga LED, altera status do display/alarme).

Publica��o de Status: O Pico W publica o estado atual de seus dispositivos e sensores em t�picos espec�ficos:
/luz_[comodo]/state: "On" ou "Off" para cada LED de c�modo.
/display/state: "On" ou "Off" para o display (representando a TV).
/alarme/state: "On" (ligado) ou "Off" (desligado) para o alarme.
/alarme/acionado: "VIOLACAO" ou "SEM VIOLACAO" em caso de detec��o de intrus�o.
/temperature: Valor da temperatura interna do RP2040.
/online: "1" (quando conectado) ou "0" (Last Will, se desconectado inesperadamente).
/uptime: Tempo de atividade do sistema.

Interface de Controle (IoT MQTT Dashboard): O aplicativo no smartphone/tablet se conecta ao mesmo broker Mosquitto. Ele possui bot�es configurados para publicar comandos e pain�is/gr�ficos configurados para assinar os t�picos de status, exibindo os dados em tempo real.
Leitura e Monitoramento
Sensor Ultrass�nico_1: Mede a dist�ncia periodicamente para automa��o das luzes RGB frontais (combina��o com LDR).
Sensor Ultrass�nico_2: Mede a dist�ncia periodicamente para o sistema de alarme, detectando presen�a pr�xima.
Sensor LDR: Leitura anal�gica para avaliar a luminosidade ambiente, influenciando o acendimento das luzes RGB frontais.
Joystick: Leitura dos valores anal�gicos dos eixos X e Y para detectar mudan�as de posi��o, simulando a abertura de portas para o alarme.
Temperatura Interna: Leitura do sensor t�rmico do RP2040 publicada via MQTT.

Estrutura do C�digo:
Wi-Fi & lwIP Setup:
Inicializa a interface de rede (CYW43).
Conecta-se � rede Wi-Fi.
Realiza a resolu��o DNS do broker MQTT.

MQTT Client:
Cria e gerencia a conex�o MQTT.
Define callbacks para conex�o, recebimento de mensagens (t�pico e dados) e confirma��o de publica��es/inscri��es.
Gerencia a publica��o de estados e telemetria.

Drivers:
OLED SSD1306 via I�C.
Matriz de LEDs via PIO.
Leitura de ADCs (LDR, Joystick, Temperatura Interna) e sensores digitais (Ultrass�nicos, Bot�o).

L�gica de Controle:
Fun��es para ligar/desligar LEDs, TV (display).
Fun��es para o sistema de alarme (ativa��o/desativa��o, detec��o de viola��o, acionamento do buzzer).
L�gica para luzes autom�ticas frontais.

Main Loop:
Processa eventos de rede (CYW43_arch_poll).
Atualiza leituras de sensores.
Atualiza OLED e LED matrix.
Verifica e processa a l�gica do Alarme.
Publica periodicamente estados e dados de sensores.

Como Executar o Projeto:
Montagem: Monte os componentes eletr�nicos conforme a tabela de pinos e as especifica��es do seu hardware.
Configura��o do Mosquitto Broker:
Em um dispositivo Android, instale o aplicativo Termux.
No Termux, instale o Mosquitto: pkg install mosquitto. Inicio e configure.
Anote o endere�o IP do seu dispositivo Android para configurar o MQTT_SERVER no c�digo da Pico W.

Configura��o do Firmware da Pico W:
Configure suas credenciais de Wi-Fi (WIFI_SSID, WIFI_PASSWORD) no c�digo-fonte.
Configure o endere�o IP do seu Mosquitto Broker (MQTT_SERVER, MQTT_USERNAME, MQTT_PASSWORD) no c�digo.

Compila��o e Grava��o:
Certifique-se de ter o SDK do Raspberry Pi Pico configurado.
Compile o firmware (.uf2) do projeto.
Grave o firmware compilado na sua Raspberry Pi Pico W.

Configura��o do IoT MQTT Dashboard:
Instale o aplicativo "IoT MQTT Dashboard" (ou similar) em seu smartphone/tablet.
Configure uma nova conex�o apontando para o IP do seu Mosquitto Broker.
Crie bot�es e pain�is/gr�ficos no aplicativo, configurando-os para publicar e assinar os t�picos MQTT conforme a l�gica do seu projeto (ex: publicar "On" em /luz_sala, assinar /temperature para o gr�fico).

Intera��o:
Aguarde a Pico W conectar-se ao Wi-Fi e ao broker MQTT.
Use o aplicativo IoT MQTT Dashboard para controlar LEDs, acionar/desativar o alarme e visualizar os dados dos sensores em tempo real.

Requisitos:
Raspberry Pi Pico W
SDK do Raspberry Pi Pico configurado
Display OLED compat�vel com SSD1306
Sensor LDR
Sensor Ultrass�nico HC-SR04 (x2)
Matriz de LEDs 5�5
Buzzer
Bot�o
Joystick
Dispositivo Android com Termux para rodar o Mosquitto Broker (ou um Mosquitto Broker em outro PC/servidor)
Aplicativo "IoT MQTT Dashboard" (ou similar) para smartphone/tablet

Reposit�rio:
GitHub: 

Autor
Nome: Mateus Moreira da Silva