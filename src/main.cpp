/*
 * FrioLog Soluções em Logística
 * Nó de Borda Inteligente — câmara-piloto (protótipo / prova de conceito)
 *
 * Sensores:
 *   1) DHT22  -> temperatura (°C) e umidade relativa (%)      [digital, 1-wire]
 *   2) LDR (sensor de luminosidade) -> detecta entrada de luz [analógico]
 *      A câmara é fechada e escura; se a porta abrir, entra luz.
 *      Por isso o LDR é usado como detector indireto de "porta aberta".
 *
 * Atuadores:
 *   1) Relé -> aciona o sistema de refrigeração/compressor
 *   2) Buzzer -> alarme sonoro local
 *
 * Toda a decisão ocorre no ESP32, localmente, sem rede/internet.
 */

#include <Arduino.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ----------------------- Pinos -----------------------
#define PIN_DHT           15
#define DHT_TYPE          DHT22

#define PIN_LDR           34    // ADC1 (entrada apenas) - leitura analógica de luz
#define PIN_RELAY_COOLING 26    // Atuador 1: refrigeração
#define PIN_BUZZER        27    // Atuador 2: alarme sonoro

// ----------------------- Parâmetros de decisão -----------------------
const float TEMP_LIMITE_ALTA   = 8.0f;   // °C  -> acima disso, câmara está esquentando
const float TEMP_CRITICA       = 12.0f;  // °C  -> temperatura crítica
const float UMID_LIMITE_ALTA   = 85.0f;  // %   -> umidade alta
const float UMID_CRITICA       = 95.0f;  // %   -> umidade crítica

// Limiar do LDR (0-4095 no ESP32). Ajuste conforme calibração na simulação:
// observem o valor impresso no Serial Monitor com a câmara "fechada" (escura)
// e "aberta" (clara) no Wokwi e definam o ponto de corte entre os dois.
const int LIMIAR_LUZ_PORTA_ABERTA = 2500;

const unsigned long TEMPO_PORTA_ABERTA_LIMITE = 10000; // ms (10s) porta aberta = alerta
const unsigned long INTERVALO_LEITURA         = 2000;  // ms entre leituras dos sensores

// O LCD 16x2 não tem espaço pra mostrar tudo que o Serial Monitor mostra de
// uma vez só. Por isso ele fica alternando entre "telas" (igual um painel),
// cada uma com uma parte da informação, repetindo o ciclo continuamente.
const unsigned long INTERVALO_TELA_LCD = 3000; // ms entre troca de tela no LCD
const int QTD_TELAS_LCD = 3;

// ----------------------- Estados operacionais -----------------------
enum EstadoSistema {
  ESTADO_NORMAL,          // tudo dentro do esperado
  ESTADO_RESFRIANDO,      // temperatura acima do limite -> aciona refrigeração
  ESTADO_ALERTA_UMIDADE,  // umidade acima do limite (temperatura normal)
  ESTADO_PORTA_ABERTA,    // luz detectada (porta aberta) além do tempo limite
  ESTADO_CRITICO          // condição crítica: limite excedido (temp e/ou umidade) simultaneamente,
                          // ou valores em nível crítico isolado
};

EstadoSistema estadoAtual = ESTADO_NORMAL;
EstadoSistema estadoAnterior = ESTADO_NORMAL;

// ----------------------- Objetos e variáveis globais -----------------------
DHT dht(PIN_DHT, DHT_TYPE);

// Endereço I2C mais comum dos módulos de LCD 16x2 (PCF8574). Se o texto não
// aparecer no LCD, tente trocar para 0x3F, que é o outro endereço comum.
LiquidCrystal_I2C lcd(0x27, 16, 2);

float temperatura = 0.0f;
float umidade = 0.0f;
int leituraLuz = 0;
bool portaAberta = false; // inferida pela luminosidade (ver decidirEstado/lerSensores)

unsigned long instanteUltimaLeitura = 0;
unsigned long instanteInicioPortaAberta = 0;
bool contandoTempoPorta = false;

unsigned long instanteUltimaTelaLCD = 0;
int telaAtualLCD = 0; // 0 = temp/umid, 1 = luz/porta, 2 = estado

// ----------------------- Protótipos -----------------------
void lerSensores();
EstadoSistema decidirEstado(float t, float h, bool porta, unsigned long tempoPortaAberta);
void atualizarAtuadores(EstadoSistema estado);
void registrarMudancaDeEstado(EstadoSistema novo);
const char* nomeEstado(EstadoSistema e);
void atualizarLCD();

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_LDR, INPUT);
  pinMode(PIN_RELAY_COOLING, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  digitalWrite(PIN_RELAY_COOLING, LOW);
  digitalWrite(PIN_BUZZER, LOW);

  dht.begin();

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("FrioLog EDCO");
  lcd.setCursor(0, 1);
  lcd.print("Inicializando...");

  Serial.println("=== FrioLog - No de Borda Inteligente ===");
  Serial.println("Inicializando... decisao 100% local no ESP32, sem rede.");
}

void loop() {
  unsigned long agora = millis();

  // Leitura periódica dos sensores (não bloqueante)
  if (agora - instanteUltimaLeitura >= INTERVALO_LEITURA) {
    instanteUltimaLeitura = agora;
    lerSensores();

    // Controle do tempo de porta aberta
    if (portaAberta) {
      if (!contandoTempoPorta) {
        contandoTempoPorta = true;
        instanteInicioPortaAberta = agora;
      }
    } else {
      contandoTempoPorta = false;
    }

    unsigned long tempoPortaAberta = contandoTempoPorta ? (agora - instanteInicioPortaAberta) : 0;

    // ---- Decisão local (função dedicada) ----
    EstadoSistema novoEstado = decidirEstado(temperatura, umidade, portaAberta, tempoPortaAberta);

    if (novoEstado != estadoAtual) {
      estadoAnterior = estadoAtual;
      estadoAtual = novoEstado;
      registrarMudancaDeEstado(estadoAtual);
    }

    // ---- Atualização dos atuadores conforme o estado decidido ----
    atualizarAtuadores(estadoAtual);

    // ---- Log contínuo (estado atual + leituras) ----
    Serial.print("[LEITURA] Temp: ");
    Serial.print(temperatura, 1);
    Serial.print(" C | Umid: ");
    Serial.print(umidade, 1);
    Serial.print(" % | Luz (LDR): ");
    Serial.print(leituraLuz);
    Serial.print(" | Porta: ");
    Serial.print(portaAberta ? "ABERTA" : "fechada");
    Serial.print(" | Estado: ");
    Serial.println(nomeEstado(estadoAtual));

    // Atualiza o conteúdo da tela do LCD que estiver ativa no momento,
    // com os valores que acabaram de ser lidos.
    atualizarLCD();
  }

  // Troca a tela exibida no LCD periodicamente, de forma independente da
  // leitura dos sensores, para o painel ir mostrando todas as informações
  // em sequência (igual o Serial Monitor mostrava tudo em uma linha só).
  if (agora - instanteUltimaTelaLCD >= INTERVALO_TELA_LCD) {
    instanteUltimaTelaLCD = agora;
    telaAtualLCD = (telaAtualLCD + 1) % QTD_TELAS_LCD;
    atualizarLCD();
  }
}

// Lê os dois sensores do nó de borda
void lerSensores() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();

  // DHT22 retorna NaN em caso de falha de leitura; mantém último valor válido
  if (!isnan(t)) temperatura = t;
  if (!isnan(h)) umidade = h;

  // Sensor analógico: quanto mais luz entra na câmara, maior a leitura do LDR
  // (ajuste a lógica de comparação conforme o módulo/polaridade usados na simulação)
  leituraLuz = analogRead(PIN_LDR);
  portaAberta = (leituraLuz > LIMIAR_LUZ_PORTA_ABERTA);
}

// Mostra no LCD 16x2 a mesma informação que ia só pro Serial Monitor.
// Como não cabe tudo de uma vez, o conteúdo alterna entre 3 telas
// (ver telaAtualLCD / INTERVALO_TELA_LCD), formando um ciclo contínuo.
void atualizarLCD() {
  lcd.clear();
  switch (telaAtualLCD) {
    case 0: // Tela 1: temperatura e umidade
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(temperatura, 1);
      lcd.print(char(223)); // símbolo de grau
      lcd.print("C");
      lcd.setCursor(0, 1);
      lcd.print("Umid: ");
      lcd.print(umidade, 1);
      lcd.print(" %");
      break;

    case 1: // Tela 2: luz (LDR) e status da porta
      lcd.setCursor(0, 0);
      lcd.print("Luz(LDR): ");
      lcd.print(leituraLuz);
      lcd.setCursor(0, 1);
      lcd.print("Porta: ");
      lcd.print(portaAberta ? "ABERTA" : "fechada");
      break;

    case 2: // Tela 3: estado geral do sistema
    default:
      lcd.setCursor(0, 0);
      lcd.print("Estado atual:");
      lcd.setCursor(0, 1);
      lcd.print(nomeEstado(estadoAtual));
      break;
  }
}

/*
 * Função de decisão local — é o "cérebro" do nó de borda.
 * Recebe as leituras dos dois sensores e devolve o estado que deve prevalecer.
 *
 * Prioridade (da mais alta para a mais baixa), quando mais de uma condição é
 * verdadeira ao mesmo tempo:
 *   1) ESTADO_CRITICO         (segurança do produto em risco)
 *   2) ESTADO_PORTA_ABERTA    (perda de estanqueidade térmica prolongada)
 *   3) ESTADO_ALERTA_UMIDADE  (risco de contaminação/condensação)
 *   4) ESTADO_RESFRIANDO      (ação corretiva de rotina)
 *   5) ESTADO_NORMAL          (default)
 */
EstadoSistema decidirEstado(float t, float h, bool porta, unsigned long tempoPortaAberta) {
  // Regra que combina os dois sensores simultaneamente:
  // temperatura E umidade acima do limite alto ao mesmo tempo -> crítico
  bool combinadaCritica = (t > TEMP_LIMITE_ALTA) && (h > UMID_LIMITE_ALTA);

  if (t > TEMP_CRITICA || h > UMID_CRITICA || combinadaCritica) {
    return ESTADO_CRITICO;
  }

  if (porta && tempoPortaAberta >= TEMPO_PORTA_ABERTA_LIMITE) {
    return ESTADO_PORTA_ABERTA;
  }

  if (h > UMID_LIMITE_ALTA) {
    return ESTADO_ALERTA_UMIDADE;
  }

  if (t > TEMP_LIMITE_ALTA) {
    return ESTADO_RESFRIANDO;
  }

  return ESTADO_NORMAL;
}

// Aciona os atuadores de acordo com o estado decidido
void atualizarAtuadores(EstadoSistema estado) {
  switch (estado) {
    case ESTADO_NORMAL:
      digitalWrite(PIN_RELAY_COOLING, LOW);
      digitalWrite(PIN_BUZZER, LOW);
      break;

    case ESTADO_RESFRIANDO:
      digitalWrite(PIN_RELAY_COOLING, HIGH);
      digitalWrite(PIN_BUZZER, LOW);
      break;

    case ESTADO_ALERTA_UMIDADE:
      digitalWrite(PIN_RELAY_COOLING, LOW);
      // alarme intermitente para chamar atenção sem ser tão agressivo quanto o crítico
      digitalWrite(PIN_BUZZER, (millis() / 500) % 2 == 0 ? HIGH : LOW);
      break;

    case ESTADO_PORTA_ABERTA:
      digitalWrite(PIN_RELAY_COOLING, LOW); // resfriar com a porta aberta é ineficiente
      digitalWrite(PIN_BUZZER, (millis() / 300) % 2 == 0 ? HIGH : LOW);
      break;

    case ESTADO_CRITICO:
      digitalWrite(PIN_RELAY_COOLING, HIGH); // refrigeração máxima
      digitalWrite(PIN_BUZZER, HIGH);        // alarme contínuo
      break;
  }
}

void registrarMudancaDeEstado(EstadoSistema novo) {
  Serial.println("------------------------------------------------------");
  Serial.print(">>> MUDANCA DE ESTADO: ");
  Serial.print(nomeEstado(estadoAnterior));
  Serial.print(" -> ");
  Serial.println(nomeEstado(novo));
  Serial.println("------------------------------------------------------");
}

const char* nomeEstado(EstadoSistema e) {
  switch (e) {
    case ESTADO_NORMAL:         return "NORMAL";
    case ESTADO_RESFRIANDO:     return "RESFRIANDO";
    case ESTADO_ALERTA_UMIDADE: return "ALERTA_UMIDADE";
    case ESTADO_PORTA_ABERTA:   return "PORTA_ABERTA";
    case ESTADO_CRITICO:        return "CRITICO";
    default:                    return "DESCONHECIDO";
  }
}
