/*
 * CODE DOMINO
 * por Daniel Almeida Chagas
 * prof.daniel.chagas@gmail.com
 * 
 * Funcionamento:
 * O robô inicia parado, aguardando a leitura de uma peça RFID para ou iniciar a execução de um programa ou gravar um programa.
 * O loop dessa chamada está na função loop(), e é feita a cada passar da variável passo. Esta inicia alta (50) mas é diminuida 
 * até o valor 4 gradativamente, para implementar a aceleração e desaceleração do robô (mais precisão no movimento).
 * Os botões também podem ser usados para operar: 
 * [D] [E] [C] [A] [N] [O]
 * 
 * [O]  - Inicia gravação de programa no endereço D da memória. Com o robô em movimento, este botão vira STOP.
 *      - Toque longo formata a memória
 * [N]  - Toque curto executa programa no endereço N.
 *      - Toque longo grava no endereço N.
 * [A]  - Toque curto executa programa no endereço A. 
 *      - Toque longo grava no endereço A.
 * [C]  - Toque curto executa programa no endereço C. 
 *      - Toque longo grava no endereço C.
 * [E]  - Toque curto executa programa no endereço E. 
 *      - Toque longo grava no endereço E.
 * [D]  - Toque curto executa programa no endereço Play. 
 *      - Toque longo executa o programa de calibração padrão.
 * 
 * Todos os programas estão gravados na EEPROM, em 128 bytes de espaço. Os 64 primeiros são os comandos, os 64 seguintes são parâmetros.
 * Ao gravar um novo programa, ele é colocado nos vetores programa[] e parametro[] na memória RAM, e só no final é gravado na EEPROM.
 * 
 * A cada 1 segundo a função loop() chama outras funções acessórias: leRfid() e medeDistancia().
 * 
 * Ao ler um RFID o robô chama as funções executaInstrução() ou gravaInstrução() dependendo da variável 'executando' (marcada 
 * como FALSE ele irá gravar o comando). A função também chama a outra iniciaGravacaoProgramaRAM(), caso a peça seja 1 (inicia gravação). 
 * 
 * A função iniciaGravacaoProgramaRAM() faz o robô andar para frente buscando novas peças para gravar. Ele anda por um tempo, e se não
 * encontrar uma peça final, ele para e emite um erro.
 * 
 * A função gravaInstrucao() recebe o ID do comando e o valor do parâmetro. Ela incrementa a variável passosCaminhar afim de que 
 * a cada peça encontrada, o robô ande um pouco mais buscando novas peças (parando quando encontra uma peça fim). 
 * 
 * A função executaInstrução() contém o comportamento do robô para cada peça. 
 * - 1 e 2 teoricamente nunca são executadas aqui;
 * - 3 é a peça executar (play). 
 * - 4 é a peça frente (forward). Se vier sem parâmetro, usa uma variável com uma distância padrão. Multiplica o parâmetro por um 
 *   valor para valer os passos curtos do motor. 
 * - 5 e 6 são as de giro no próprio eixo. Se vierem sem parâmetro, gira 90 graus. A multiplicação é para transformar os graus em 
 *   passos do motor.
 * - 7 é espere (break). Espera 1 segundo ou o número de segundos dado como parâmetro. 
 * - 8 e 9 são as peças repetir e fim repetir. 
 * - 10 e 11 controlam a caneta (primeira desce a caneta, segunda sobe).
 * 
 * A função fim() reseta as variáveis, voltando o robô para o estado de repouso inicial. A leitura de RFID a cada segundo continua. 
 * 
 * A função caminhar() controla a variável passo para permitir aceleração e desaceleração do robô. Ela também chama as funções 
 * direita() e esquerda() igualmente, fazendo que o robô ande para frente. O padrão de passo gerado por essas funções é agrupado e 
 * enviado ao chip Shift Register. 
 * 
 * As funções direita() e esquerda() constroem o padrão de passos (nas variáveis binarioDir e binarioEsq). Os passos seguem o padrão
 * 8 passos. 
 * 
 * A função girar() recebe o parâmetro da direção a fazer (1 para horário, 0 para anti-horário), e também faz a aceleração e 
 * desaceleração do passo (porém com passo mínimo 6, para não ir tão rápido). 
 * 
 * A função erro() toca um tom negativo no buzzer e para o robô. 
 */
#include <SPI.h>
#include <MFRC522.h>
#include <Ultrasonic.h>
#include <EEPROM.h>
#include <Servo.h>
#include <ArduinoUniqueID.h>
#include "U8glib.h"

U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NO_ACK);  // Display which does not send ACK

//pinos da shield
int servo = 16;
int sck = 13; //SPI
int miso = 12; //SPI
int mosi = 11; //SPI
#define RST_PIN         9
#define SS_PIN          10
int latchPin = 8; //Pin connected to ST_CP of 74HC595
int clockPin = 7; //Pin connected to SH_CP of 74HC595
int dataPin = 6; ////Pin connected to DS of 74HC595
int trig = 5; //ultrasom
int echo = 4; //ultrasom
int buzzer = 3; //som
int led = 2;
//A0 são botões
//Analógicos de A1 a A5 são sensores de linha

//Instanciação de Objetos
//---------RFID
MFRC522 mfrc522(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;
MFRC522::StatusCode status; //variável para pegar status do cartão
byte buffer[18];  //data transfer buffer (16+2 bytes data+CRC)
byte size = sizeof(buffer);
uint8_t pageAddr = 0x06;

//Ultrassom
Ultrasonic ultrasonic(5, 4);

//Servo
Servo myservo;

//Variáveis
unsigned long millisAnterior = 0;
unsigned long millisAtual = 0;
boolean caminhando = false; //define se o robô está em movimento ou não
//boolean gravando = false; //define se o robô está gravando comandos ou não.
boolean executando = false; //define se o robô está executando comandos ou não.
int programa[64];
int parametros[64]; //vetor dos parâmetros
const int calibProg[] = {3, 10, 8, 4, 5, 9, 11, 5, 4, 6, 10, 8, 4, 5, 9, 11, 5, 4, 6, 2};
const int calibParam[] = {0, 0, 4, 0, 0, 0, 0, 17, 5, 14, 0, 24, 1, 15, 0, 0, 0, 4, 95, 0}; 
int ponteirosRepetir[7]; //vetor que armazena os ponteiros de repetição
int repeticaoAninhada = 0; //marca o uso de repetições dentro de repetições;
int ponteiro = 0; //ponteiro indicando em que passo está do programa
int passo = 3; //milissegundos entre cada instrução de movimento.
int setDistanciaBasica = 10; //distancia básica de movimento do robô em linha reta
int passosCaminhar = 0; //quantidade de passos que o robô deve percorrer até a próxima instrução.
int passosCurva = 0; //quantidade de passos em curvas
int grausGirar = 0; //quantidade de passos que o robô deve percorrer até a próxima instrução.
int movimentacaoX = 0; //variáveis que gravam o deslocamento total do robô, para a instrução voltar ao início. 
int movimentacaoY = 0;
int passoDir = 1; //vai de 1 a 4
int passoEsq = 1;
int valorBotao = 0; //variável de leitura dos botões no Analogic 0 (A0)
int ultimoEstadoBotoes = LOW; //variável de debounce do botão
int ultimoValorBotoes = 0;
long timerBotao = 0;
long timerLongoPressionar = 1000;
int enderecoEepromGravacao = 0;
int distanciaUltrassom = 150; //distância medida pelo ultrassom
int limiteUltrassom = 10;
int raio = 10; //variável raio para comando curvas
byte binarioDir = B00000000;
byte binarioEsq = B00000000;
String tempMsg = ""; //variável temporária de mensagens para serial debug
int canetaAcima = 65;
int canetaAbaixo = 100;
String entradaString = "";         // String para receber dados seriais
boolean stringCompleta = false;  // marca se a string está completa
boolean rfidAlwaysOn = true; //define se o RFID estará funcinando ao executar códigos
boolean gyroPresent = true; //define se o giroscópio está presente
boolean ultrasoundPresent = true; //define se o sensor de distância ultrassonico está presente
boolean oledPresent = true; //define se o a tela OLED I2C está presente
boolean invertLeftMotor = false;
boolean invertRightMotor = false;
boolean ledPlay = false;
boolean ledE = false;
boolean ledC = false;
boolean ledA = false;
boolean ledN = false;
boolean ledRec = false;
int leds;
char decabotName[5] = "A01  ";                     //<<<--- put your robot name here!
char decabotOwner[50] = "anybody@decabot.com";     //<<<--- put your e-mail here!
char term1[25] = "> l1"; //variáveis do terminal OLED
char term2[25] = "> l2";
char term3[25] = "> l3";
char term4[25] = "> l4";
char term5[25] = "> l5";



//imagens para OLED
const uint8_t decano_logo[] PROGMEM = {
0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFE, 0xF0,
0x00, 0x07, 0xFF, 0x80, 0x0F, 0xFC, 0x00, 0x04, 0x00, 0x06, 0x01, 0x80, 0x3F, 0xFE, 0x3C, 0x00,
0x07, 0xFF, 0x80, 0x1F, 0xFC, 0x00, 0x0E, 0x00, 0x07, 0x01, 0x80, 0x3E, 0x3E, 0x0F, 0x00, 0x00,
0x00, 0x00, 0x38, 0x00, 0x00, 0x0E, 0x00, 0x07, 0x81, 0x80, 0x38, 0x0E, 0x83, 0xC0, 0x00, 0x00,
0x00, 0x30, 0x00, 0x00, 0x1F, 0x00, 0x07, 0xC1, 0x80, 0x38, 0x0E, 0xC1, 0xE0, 0x07, 0xFF, 0x80,
0x30, 0x00, 0x00, 0x3B, 0x80, 0x06, 0xE1, 0x80, 0x30, 0x06, 0xC3, 0xC0, 0x07, 0xFF, 0x80, 0x30,
0x00, 0x00, 0x3B, 0x80, 0x06, 0x71, 0x80, 0x30, 0x06, 0xCF, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00,
0x00, 0x71, 0xC0, 0x06, 0x39, 0x80, 0x38, 0x0E, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00,
0xE4, 0xE0, 0x06, 0x1D, 0x80, 0x38, 0x0E, 0xF0, 0x00, 0x07, 0xFF, 0x80, 0x1F, 0xFC, 0x00, 0xEE,
0xE0, 0x06, 0x0F, 0x80, 0x3E, 0x3E, 0xC0, 0x00, 0x07, 0xFF, 0x80, 0x0F, 0xFC, 0x01, 0xC4, 0x70,
0x06, 0x07, 0x80, 0x3F, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x3F, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0xF8, 0x00, 0x3E, 0x00, 0x3E, 0x00, 0x0F, 0x80, 0x0F, 0xE0, 0x08, 0x00, 0x7C, 0x00,
0x7E, 0x84, 0x00, 0x41, 0x00, 0x21, 0x00, 0x10, 0x40, 0x01, 0x00, 0x08, 0x00, 0x82, 0x00, 0x80,
0x84, 0x00, 0x41, 0x00, 0x3F, 0x00, 0x10, 0x40, 0x01, 0x00, 0x08, 0x00, 0x80, 0x00, 0x7C, 0xF8,
0x00, 0x41, 0x00, 0x20, 0x80, 0x10, 0x40, 0x01, 0x00, 0x08, 0x00, 0x80, 0x00, 0x02, 0x84, 0x00,
0x41, 0x00, 0x20, 0x80, 0x10, 0x40, 0x01, 0x00, 0x08, 0x00, 0x82, 0x00, 0x82, 0x84, 0x00, 0x3E,
0x00, 0x3F, 0x00, 0x0F, 0x80, 0x01, 0x00, 0x08, 0x00, 0x7C, 0x00, 0x7C, 
};  //15 20

const uint8_t bat_4 [] PROGMEM = {
0x00, 0x00, 0x3F, 0xFE, 0x40, 0x01, 0x5B, 0x6D, 0xDB, 0x6D, 0xDB, 0x6D, 0xDB, 0x6D, 0x5B, 0x6D,
0x40, 0x01, 0x3F, 0xFE, 0x00, 0x00, 
}; //2 11

const uint8_t decabot_icon [] PROGMEM = {
0xFF, 0x80, 0x80, 0x80, 0xBE, 0x80, 0xFF, 0x80, 0xEB, 0x80, 0xBE, 0x80, 0x80, 0x80, 0xFF, 0x80,
0xC1, 0x80, 
};  //2,9

const uint8_t decano_small_logo [] PROGMEM = {
0xC3, 0xE7, 0x88, 0x8B, 0xE0, 0x30, 0x08, 0x14, 0xCB, 0x60, 0x8B, 0xE8, 0x14, 0xAA, 0x20, 0xB0,
0x08, 0x22, 0x9B, 0x60, 0xC3, 0xE7, 0xAA, 0x8B, 0xE0, 
}; //5 5

const uint8_t gyro_icon [] PROGMEM = {
0x04, 0x00, 0x0A, 0x00, 0x0A, 0x00, 0x1B, 0x00, 0x68, 0xC0, 0x88, 0x20, 0x61, 0xC0, 0x1F, 0x00,
0x0A, 0x00, 0x0A, 0x00, 0x04, 0x00, 
}; //2 11

const uint8_t rec_icon [] PROGMEM = {
0x00, 0x00, 0x00, 0x7F, 0xFF, 0xF8, 0xFF, 0xFF, 0xBC, 0xFF, 0xFE, 0x0C, 0x98, 0xCE, 0x0C, 0x99,
0xBC, 0x04, 0xA8, 0xCE, 0x0C, 0xFF, 0xFE, 0x0C, 0xFF, 0xFF, 0xBC, 0x7F, 0xFF, 0xF8, 0x00, 0x00,
0x00,  
}; //3 11

const uint8_t rfid_icon [] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x02, 0x00, 0x19, 0x00, 0x05, 0xCE, 0xB5, 0xCC, 0xA8,
0xA8, 0xB0, 0x00, 0x00, 0x00, 0x00, 
}; //2 11

const uint8_t ultrasound_icon [] PROGMEM = {
0x00, 0x00, 0x01, 0x00, 0x04, 0x80, 0x12, 0x80, 0x4A, 0x40, 0xEA, 0x40, 0x4A, 0x40, 0x12, 0x80,
0x04, 0x80, 0x01, 0x00, 0x00, 0x00,  
}; //2 11

void setup() {
  Serial.begin(9600);
  Serial.println(F("DECANO Robotics"));
  if(oledPresent){
    u8g.firstPage();  
    do {
      u8g.drawBitmapP( 4, 22, 15, 20, decano_logo);
    } while( u8g.nextPage() );
  }
  entradaString.reserve(200);
  myservo.attach(servo);
  myservo.write(canetaAcima);
  pinMode(buzzer, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(14, INPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  SPI.begin(); 
  mfrc522.PCD_Init();
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, B00000000); //envia resultado binário para o shift register
  digitalWrite(latchPin, HIGH);
  //gravaProgramaNaEeprom(0); //apaga o bloco inicial
  digitalWrite(led, LOW);
  somInicio();
  whoami();
  digitalWrite(led, HIGH);
  u8g.setFont(u8g_font_5x7);
  u8g.firstPage();  
  mensagemDebug(F("Boot OK!"));
}

void loop() {
  millisAtual = millis();
  //A cada tempo do passo, execute
  if(millisAtual - millisAnterior >= passo) {
    millisAnterior = millisAtual;
    if(caminhando) {
      caminhar();
    } else {
      if(executando){
        ponteiro++;
        executaPrograma(ponteiro);
      }
      leds = 0 | (ledN << 1) | (ledA << 2) | (ledC << 3) | (ledE << 4) | (ledPlay << 5) | (ledRec << 6) | (0 << 7); 
      //mostra status leds
      digitalWrite(latchPin, LOW);
      shiftOut(dataPin, clockPin, MSBFIRST, leds);
      shiftOut(dataPin, clockPin, MSBFIRST, B00000000); //desliga os coilers dos motores para não esquentar
      digitalWrite(latchPin, HIGH);
    }
  }
  if(millisAtual%1000==0){
    //De um em um segundo lê o RFID e mede a distância
    if(!executando||rfidAlwaysOn){
      //mensagemDebug(F("Lendo RFID..."));
      leRfid();
    }
    if(ultrasoundPresent) medeDistancia();
  }
  if((millisAtual%3000==0)&&!executando&&caminhando){
    bipeFino(); //emite um bipe fino de 3 em 3 segundos informando que está gravando peças na memória
  }
  if(millisAtual%50==0){
    leBotao();
    //executando = true;
    //executaPrograma(0);
  }
  if (stringCompleta) {
    String t1 = entradaString.substring(0,entradaString.indexOf(":"));
    String t2 = entradaString.substring(entradaString.indexOf(":")+1);
    if(t1 == "16"||t1 == "17") {
      executaInstrucao(t1.toInt(),t2);
    } else {
      executaInstrucao(t1.toInt(),t2.toInt());
    }
    // clear the string:
    entradaString = "";
    stringCompleta = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    // pega o novo byte:
    char inChar = (char)Serial.read();
    // adiciona-o a String de entrada:
    entradaString += inChar;
    // se o caractere for uma linha nova, sinaliza:
    if (inChar == '\n') {
      stringCompleta = true;
    }
  }
}

void whoami() {
  UniqueIDdump(Serial);
  String tmp1 = F("Decabot Name: ");
  for(int i=896;i<=900;i++){
    tmp1.concat((char) EEPROM.read(i));
    decabotName[i - 896] = EEPROM.read(i);
  }
  mensagemDebug(tmp1);
  String tmp2 = F("Own:");
  for(int i=901;i<950;i++){
    tmp2.concat((char) EEPROM.read(i));
  }
  mensagemDebug(tmp2);
}

void leBotao() {
  valorBotao = analogRead(A0);
  if((valorBotao > 200)&&(timerBotao + timerLongoPressionar < millis())){
    tone(buzzer, 440, 200); //bipa ao segurar
  }
  if((valorBotao > 200)&&!ultimoEstadoBotoes){
    bipe();
    ultimoEstadoBotoes = HIGH;
    ultimoValorBotoes = valorBotao;
    timerBotao = millis();
    digitalWrite(led, HIGH);
  }
  if((valorBotao <= 200)&&ultimoEstadoBotoes){
    noTone(buzzer);
    ultimoEstadoBotoes = LOW;
    digitalWrite(led, LOW);
    long timerAtual = millis();
    if(timerBotao + timerLongoPressionar > timerAtual){
      //Serial.println(timerBotao);
      //Serial.println(valorBotao);
      if(ultimoValorBotoes<709) {
        //botão grava/para
        mensagemDebug(F("botao REC")); 
        ledRec = true;
        if(executando) {
          erro();
        } else {
          enderecoEepromGravacao = 512;
          iniciaGravacaoProgramaRAM();
        }
      } else if(ultimoValorBotoes < 761) {
        //botão A
        mensagemDebug(F("executar N"));
        ledN = true;
        //carregaProgramadaEeprom(0);
        ponteiro = 0;
        executando = true;
        executaPrograma(ponteiro);
      } else if(ultimoValorBotoes < 822) {
        //botão B
        mensagemDebug(F("executar A")); 
        ledA = true;
        //carregaProgramadaEeprom(128); //está resetando o arduino
        ponteiro = 128;
        executando = true;
        executaPrograma(ponteiro);
      } else if(ultimoValorBotoes < 894) {
        //botão C
        mensagemDebug(F("executar C")); 
        ledC = true;
        //carregaProgramadaEeprom(256);
        ponteiro = 256;
        executando = true;
        executaPrograma(ponteiro);
      } else if(ultimoValorBotoes < 977) {
        //botão D
        mensagemDebug(F("executar E")); 
        ledE = true;
        //carregaProgramadaEeprom(384);
        ponteiro = 384;
        executando = true;
        executaPrograma(ponteiro);
      } else {
        //botão E
        mensagemDebug(F("botao PLAY")); 
        ledPlay = true;
        //carregaProgramadaEeprom(512); //está resetando o arduino
        ponteiro = 512;
        executando = true;
        executaPrograma(ponteiro);
      }
    } else {
      if(ultimoValorBotoes<709) {
        //botão grava/para
        mensagemDebug(F("botao FORMATAR")); 
        //enderecoEepromGravacao = 0;
        //iniciaGravacaoProgramaRAM();
        apagaEeprom();
      } else if(ultimoValorBotoes < 761) {
        //botão N
        mensagemDebug(F("gravar em N")); 
        enderecoEepromGravacao = 0;
        iniciaGravacaoProgramaRAM();
      } else if(ultimoValorBotoes < 822) {
        //botão A
        mensagemDebug(F("gravar em A")); 
        enderecoEepromGravacao = 128;
        iniciaGravacaoProgramaRAM();
      } else if(ultimoValorBotoes < 894) {
        //botão C
        mensagemDebug(F("gravar em C")); 
        enderecoEepromGravacao = 256;
        iniciaGravacaoProgramaRAM();
      } else if(ultimoValorBotoes < 977) {
        //botão E
        mensagemDebug(F("gravar em E")); 
        enderecoEepromGravacao = 384;
        iniciaGravacaoProgramaRAM();
      } else {
        //botão Play
        mensagemDebug(F("gravar em D")); 
        //enderecoEepromGravacao = 512;
        //iniciaGravacaoProgramaRAM();
        debugEeprom();
      }
    }
  }
}

void calibracao() {
  mensagemDebug(F("carregando Hello World"));
  for(int i=0;i<sizeof(calibProg)/sizeof(int);i++){
    programa[i] = calibProg[i];
    parametros[i] = calibParam[i];
    Serial.print(i);
    Serial.print("-");
  }
  mensagemDebug(F("Hello World na mem E"));
}

void carregaProgramadaEeprom(int enderecoInicial) {
  mensagemDebug(F("lendo prog EEPROM"));
  for(int i=enderecoInicial;i<enderecoInicial+64;i++){
    programa[i] = EEPROM.read(i);
    parametros[i] = EEPROM.read(i+64);
    Serial.print(i);
    Serial.print("-");
  }
  Serial.println("Carregado!");
}

void gravaProgramaNaEeprom(int enderecoInicial) {
  mensagemDebug(F("salvando prog EEPROM"));
  for(int i=0;i<64;i++){
    EEPROM.write(i+enderecoInicial, programa[i]);
    EEPROM.write(i+64+enderecoInicial, parametros[i]);
    Serial.print(i);
    Serial.print("|");
    if(programa[i]==2) break;
  }
  somGravando();
  mensagemDebug(F("Gravado!"));
}

void apagaEeprom() {
  delay(1500);
  //apaga EEPROM
  //espera 5 segundos até confirmar
  Serial.println(F("memória EEPROM será apagada em 5 segundos! Desligue a alimentação para cancelar!"));
  mensagemDebug(F("5 seg p apagar EEPROM"));
  for(int i=0;i<30;i++){
    if(i<20){
      tone(buzzer, 494, 50);
      delay(50);
      noTone(buzzer);
      delay(200);
    } else {
      tone(buzzer, 494, 50);
      delay(25);
      noTone(buzzer);
      delay(100);
    }
    if(i%5==0) mensagemDebug(F("..."));
  }
  somGravando();
  mensagemDebug(F("Apagando EEPROM"));
  for(int i=0;i<896;i++){       //reserva o último bloco para dados que não são apagados
    EEPROM.write(i, 0);
  }
  for(int i=0;i<896;i=i+128){   //inicia os blocos sempre como parâmetro 2 de parada
    EEPROM.write(i, 2);
  }
  mensagemDebug(F("Apagado!"));
  calibracao();
  gravaProgramaNaEeprom(384); //grava calibração no botão E (desenho)
  somAfirmativo();
}

void debugEeprom() {
  mensagemDebug(F("Dump EEPROM p Debug:"));
  for(int i=0;i<1024;i++){
    if(i%64==0) Serial.println("- - - - - -");
    Serial.print(i);
    Serial.print("\t");
    Serial.println(EEPROM.read(i));
  }
  somAfirmativo();
}

void debugRam() {
  mensagemDebug(F("Dump RAM p Debug:"));
  for(int i=0;i<64;i++){
    Serial.print(i);
    Serial.print("->");
    Serial.print(programa[i]);
    Serial.print(":");
    Serial.println(parametros[i]);
  }
  somAfirmativo();
}

void leRfid() {
  //Códigos de leitura do RFID
  //Se não houver tag, não faz nada.
  if (!mfrc522.PICC_IsNewCardPresent()) return;
  //Chama o leitor de RFID e lê uma tag. 
  if (!mfrc522.PICC_ReadCardSerial()) return;
  bipe();
  status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(pageAddr, buffer, &size);
  if (status != MFRC522::STATUS_OK) {
    Serial.print(F("MIFARE_Read() falhou: "));
    Serial.println(mfrc522.GetStatusCodeName(status));
    erro();
    return;
  }
  mfrc522.PICC_HaltA();
  int instrucao = block(0);
  int parametro = block(4);
  //Se for a instrução 1-gravar, gravar programa, inicia a gravação imediata e não passa para as outras funções.
  if(instrucao==1){
    enderecoEepromGravacao = parametro * 128;
    iniciaGravacaoProgramaRAM();
    return;
  }
  if(instrucao==2){ //Se a instrução for 2-parar, ele para a gravação. 
    if(!executando) {
      gravaProgramaNaEeprom(enderecoEepromGravacao); //grava o programa no endereço EEPROM dado. 
    }
    fim();
    return;
  } 
  if(instrucao==3){ //Se a instrução for 3-play, executa.
    //carregaProgramadaEeprom(0);
    executando = true;
    executaPrograma(0);
  } else {
    //Se estiver no modo executando, executa a tag. Senão grava a tag
    if((caminhando && executando)||(!caminhando && !executando)){
      executaInstrucao(instrucao,parametro);
    } else if(!executando){
      gravaInstrucao(instrucao,parametro);
    } 
  }
  tempMsg  = F("leRfid() ");
  tempMsg.concat(instrucao);
  tempMsg.concat(":");
  tempMsg.concat(parametro);
  mensagemDebug(tempMsg);
}

void iniciaGravacaoProgramaRAM() {
  mensagemDebug(F("Salvando prog na RAM"));
  somGravando();
  passosCaminhar=2000; //inicia a caminhada do robô buscando peças
  myservo.write(canetaAcima);
  caminhando = true;
  executando = false;
  ponteiro = 0;
  for(int i = 0;i<64;i++){ //apaga programa anterior da RAM
    programa[i] = 2;
    parametros[i] = 0;
  }
}

void gravaInstrucao(int instrucao,int parametro){
  mensagemDebug(F("grava +1 peça no programa..."));
  //A cada nova instrução de gravação, aumenta a distância de movimento de busca do robô.
  passosCaminhar+=1500;
  //Grava a instrução recebida no vetor programa
  programa[ponteiro] = instrucao;
  parametros[ponteiro] = parametro;
  ponteiro++;
}

void executaPrograma(int ponteiro) {
  leds = 0 | (ledN << 1) | (ledA << 2) | (ledC << 3) | (ledE << 4) | (ledPlay << 5) | (ledRec << 6) | (0 << 7); 
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, leds);
  shiftOut(dataPin, clockPin, MSBFIRST, B00000000 );
  digitalWrite(latchPin, HIGH);
  int comando = EEPROM.read(ponteiro);
  int parametro = EEPROM.read(ponteiro + 64);
  if(comando!=0) { //não executa numericos
    int ponteiroSeguinte = ponteiro + 1;
    int quantidNumericas = 0;
    int novoParametro = 0;
    while(EEPROM.read(ponteiroSeguinte)==0) { //procura as peças numéricas de parâmetro
      ponteiroSeguinte++;
      quantidNumericas++;
      //if(ponteiroTemp>9) break;
    }
    int j=0;
    for(int i=quantidNumericas;i>0;i--){
      novoParametro = novoParametro + (EEPROM.read(ponteiro+64+i) * potencia(10,j)); //transforma as peças numéricas em um parâmetro novo
      j++;
    }
    tempMsg  = F("run EEPROM ");
    tempMsg.concat(ponteiro);
    tempMsg.concat("-> ");
    tempMsg.concat(comando);
    tempMsg.concat(":"); 
    if(quantidNumericas == 0) { //se houver peças numéricas, elas serão o parâmetro. Se não, será o parâmetro padrão. 
      tempMsg.concat(parametro);
      mensagemDebug(tempMsg);
      executaInstrucao(comando,parametro);
    } else {
      tempMsg.concat(novoParametro);
      mensagemDebug(tempMsg);
      executaInstrucao(comando,novoParametro);
    }
  }
}


void executaInstrucao(int instrucao,String parametro){ //se o parâmetro for uma string
  switch (instrucao) {
    case 16: //your name
      yourNameIs(parametro);
      break;
    case 17: //your name
      yourOwnerIs(parametro);
      break;
    default:
      mensagemDebug(F("executaInstrucao() peça desconhecida!"));
      erro();
      break;
  }
}

void executaInstrucao(int instrucao,int parametro){
  switch (instrucao) {
    case 1:
      iniciaGravacaoProgramaRAM(); //Se a tag for gravar programa, inicia a gravação.
      break;
    case 2:
      if(ponteirosRepetir[repeticaoAninhada+3]==0){
        somFimExecucao();
        fim(); //Se a tag for parar, para o movimento e a gravação.
      } else {
        ponteirosRepetir[repeticaoAninhada+3]--; 
        ponteiro = ponteirosRepetir[repeticaoAninhada];
        mensagemDebug(F("back to child prog"));
      }
      break;
    case 3:
      somAfirmativo();
      executando = true; //se a tag for executar, inicia a execução do programa.
      break;
    case 4: //forward
      if(parametro==0) parametro=setDistanciaBasica;
      passosCaminhar = passosCaminhar + (parametro * 100);
      caminhando = true;
      break;
    case 5: //right
      if(parametro==0) parametro=90;
      grausGirar = round(parametro * 7.8);
      caminhando = true;
      break;
    case 6: //left
      if(parametro==0) parametro=90;
      grausGirar = round(parametro * 7.8) * (-1);
      caminhando = true;
      break;
    case 7: //break
      passo = parametro*1000;
      break;
    case 8: //repetir
      repeticaoAninhada++;
      if(parametro==0) parametro=1; //se o usuário não colocar um número após a peça repetir, ele executa a rotina uma vez.
      ponteirosRepetir[repeticaoAninhada] = ponteiro; //array impar que guarda o ponteiro que marca o repetir, afim do código poder voltar à instrução.
      ponteirosRepetir[repeticaoAninhada+3] = parametro; //array par que guarda o contador do for.
      if(true){ 
        tempMsg  = F("Bl ");
        tempMsg.concat(repeticaoAninhada);
        tempMsg.concat(F(" loop "));
        tempMsg.concat(ponteirosRepetir[repeticaoAninhada+3]);
        tempMsg.concat(F("x"));
        mensagemDebug(tempMsg);
      }
      break;
    case 9: //fim repetir
      ponteirosRepetir[repeticaoAninhada+3]--;
      if(ponteirosRepetir[repeticaoAninhada+3]>0){ 
        ponteiro = ponteirosRepetir[repeticaoAninhada];
        tempMsg  = "Loop ";
        tempMsg.concat(ponteirosRepetir[repeticaoAninhada+3]);
        mensagemDebug(tempMsg);
      } else {
        mensagemDebug(F("End of loop."));
        repeticaoAninhada--;
      }
      break;
    case 10: //desce caneta
      myservo.write(canetaAbaixo);
      mensagemDebug(F("Caneta abaixada."));
      break;
    case 11: //sobe caneta
      myservo.write(canetaAcima);
      mensagemDebug(F("Caneta levantada."));
      break;
    case 12: //define global raio
      raio = parametro;
      tempMsg  = F("Raio definido p ");
      tempMsg.concat(raio);
      mensagemDebug(tempMsg);
      break;
    case 13: //curva direita
      passosCurva = passosCurva + (parametro * 100);
      caminhando = true;
      tempMsg  = F("Curva a direita por ");
      tempMsg.concat(parametro);
      tempMsg.concat("cm");
      mensagemDebug(tempMsg);
      break;
    case 14: //curva esquerda
      passosCurva = passosCurva + (parametro * 100);
      caminhando = true;
      tempMsg  = F("Curva a esqueda por ");
      tempMsg.concat(parametro);
      tempMsg.concat("cm");
      mensagemDebug(tempMsg);
      break;
    case 15: //who am I?
      whoami();
      mensagemDebug(tempMsg);
      break;
    case 16: //your name
      //yourNameIs(parametro);
      break;
    case 20:
      tempMsg  = F("pausado por ");
      tempMsg.concat(parametro);
      tempMsg.concat("s...");
      mensagemDebug(tempMsg);
      delay(parametro * 1000);
      break;
    case 21: //executa bloco de memória 1 (play)
      repeticaoAninhada++;
      ponteirosRepetir[repeticaoAninhada] = ponteiro; //array impar que guarda o ponteiro que marca o repetir, afim do código poder voltar à instrução.
      ponteirosRepetir[repeticaoAninhada+3] = 1; //array par que guarda o contador do for.
      ponteiro = 512;
      executando = true;
      executaPrograma(ponteiro);
      break;
    case 22: //executa bloco de memória 2 (botão E)
      break;
    case 23: //executa bloco de memória 3 (botão C)
      break;
    case 24: //executa bloco de memória 4 (botão A)
      break;
    case 25: //executa bloco de memória 5 (botão N)
      break;
    default:
      mensagemDebug(F("comando desconhecido"));
      erro();
      break;
  }
}

void fim(){
  mensagemDebug(F("Fim de execucao."));
  //passo = 50;
  passosCaminhar = 0;
  grausGirar = 0;
  caminhando = false;
  executando = false;
  ledPlay = false;
  ledE = false;
  ledC = false;
  ledA = false;
  ledN = false;
  ledRec = false;
  leds = 0 | (ledN << 1) | (ledA << 2) | (ledC << 3) | (ledE << 4) | (ledPlay << 5) | (ledRec << 6) | (0 << 7); 
  ponteiro = 0;
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, leds);
  shiftOut(dataPin, clockPin, MSBFIRST, B00000000); //desliga os coilers dos motores para não esquentar
  digitalWrite(latchPin, HIGH);
  digitalWrite(led, HIGH);
  myservo.write(canetaAcima);
}

void yourNameIs(String parametro){
  somGravando();
  parametro.toCharArray(decabotName,6);
  for(int i=0;i<=5;i++){
    EEPROM.write(i + 896,decabotName[i]);
  }
  mensagemDebug(F("mudado nome para "));
  mensagemDebug(decabotName);
}

void yourOwnerIs(String parametro){
  somGravando();
  parametro.toCharArray(decabotOwner,50);
  for(int i=0;i<=50;i++){
    EEPROM.write(i + 902,decabotOwner[i]);
  }
  mensagemDebug(F("Decabot owner: "));
  mensagemDebug(decabotOwner);
}

void medeDistancia(){
  //Mede distância na frente do robô com o sensor ultrasom
  distanciaUltrassom = ultrasonic.distanceRead();
  if(distanciaUltrassom < limiteUltrassom){
    mensagemDebug(F("Parada por sensor de distância"));
    erro();
    fim();
  }
}

void caminhar(){
  digitalWrite(latchPin, LOW);
  if(passosCaminhar!=0){
    //debug
    if(passosCaminhar%100==0){
      tempMsg  = F("Caminhando ");
      tempMsg.concat(passosCaminhar);
      mensagemDebug(tempMsg); 
    }
    direita(1);
    esquerda(1);
    /*
    if(passosCaminhar>50){ //acelera e freia o passo
      passo = round(passo*0.90);
      if(passo<3) passo=3;
    } else {
      passo = passo + 1;
    }*/
    passosCaminhar--;
    if(passosCaminhar==0 && executando==false) erro(); //se o robô andou buscando peças até acabar os passos, significa que ele não achou a peça final, o que é um erro.
  } else if(grausGirar!=0){
    //debug
    if(grausGirar%100==0){
      tempMsg  = F("Girando ");
      tempMsg.concat(grausGirar);
      mensagemDebug(tempMsg); 
    }
    girar();
  } else if(passosCurva!=0){
    if(passosCurva%100==0){
      tempMsg  = F("Fazendo a curva ");
      tempMsg.concat(passosCurva);
      mensagemDebug(tempMsg); 
    }
    curva();
  } else {
    caminhando = false;
    //passo = 50;
  }
  shiftOut(dataPin, clockPin, MSBFIRST, leds);
  shiftOut(dataPin, clockPin, MSBFIRST, binarioEsq | binarioDir ); //envia resultado binário para o shift register
  digitalWrite(latchPin, HIGH);
}

void esquerda(int direcao) {
  if(direcao==1){
    passoEsq++;
  } else {
    passoEsq--;
  }
  if(passoEsq>4) passoEsq = 1;
  if(passoEsq<1) passoEsq = 4;
  switch (passoEsq) {
    case 1:
      binarioEsq = B10010000;
      break;
    case 2:
      if(invertLeftMotor) {
        binarioEsq = B11000000;
      } else {
        binarioEsq = B00110000;
      }
      break;
    case 3:
      if(invertLeftMotor) {
        binarioEsq = B01100000;
      } else {
        binarioEsq = B01100000;
      }
      break;
    case 4:
      if(invertLeftMotor) {
        binarioEsq = B00110000;
      } else {
        binarioEsq = B11000000;
      }
      break;
    /*
    case 5:
      binarioEsq = B00100000;
      break;
    case 6:
      binarioEsq = B00110000;
      break;
    case 7:
      binarioEsq = B00010000;
      break;
    case 8:
      binarioEsq = B10010000;
      break;
      */
  }
}

void direita(int direcao) {
  if(direcao==1){
    passoDir++;
  } else {
    passoDir--;
  }
  if(passoDir>4) passoDir = 1;
  if(passoDir<1) passoDir = 4;
  switch (passoDir) {
    /*
    case 8:
      binarioDir = B00001000;
      break;
    case 7:
      binarioDir = B00001100;
      break;
    case 6:
      binarioDir = B00000100;
      break;
    case 5:
      binarioDir = B00000110;
      break;
      */
    case 4:
      binarioDir = B00001100;
      break;
    case 3:
      if(invertRightMotor) {
        binarioDir = B00000110;
      } else {
        binarioDir = B00001001;
      }
      break;
    case 2:
      if(invertRightMotor) {
        binarioDir = B00000011;
      } else {
        binarioDir = B00000011;
      }
      break;
    case 1:
      if(invertRightMotor) {
        binarioDir = B00001001;
      } else {
        binarioDir = B00000110;
      }
      break;
  }
}

void girar(){
  if(grausGirar > 0){
    direita(1);
    esquerda(0);
    grausGirar--;
  } else {
    direita(0);
    esquerda(1); 
    grausGirar++;
  }
  /*
  if((grausGirar>50)or(grausGirar<50)){ //acelera e freia o passo
    passo = round(passo*0.90);
    if(passo<6) passo=6;
  } else {
    passo = passo + 1;
  }*/
  if(grausGirar == 0) caminhando = false;
}

void curva(){
  if(passosCurva > 0){
    direita(1);
    //esquerda(0);
    passosCurva--;
  } else {
    //direita(0);
    esquerda(1); 
    passosCurva++;
  }
  passo = 6;
  if(passosCurva == 0) caminhando = false;
}

void somInicio() {
  digitalWrite(led, HIGH);
  for(int i=400;i<1000;i++){
    tone(buzzer, i, 3);
    delay(3);
  }
  noTone(buzzer);
  delay(50);
  tone(buzzer, 1000, 50);
  delay(50);
  noTone(buzzer);
  delay(50);
  tone(buzzer, 1000, 50);
  delay(50);
  noTone(buzzer);
  digitalWrite(led, LOW);
}

void erro(){
  mensagemDebug(F("ERRO!"));
  tone(buzzer, 391, 800);
  delay(150);
  noTone(buzzer);
  delay(30);
  tone(buzzer, 261, 1500);
  delay(400);
  noTone(buzzer);
  fim();
}

void bipe() {
  tone(buzzer, 391, 100);
  delay(100);
  noTone(buzzer);
}

void bipeFino() {
  tone(buzzer, 1047, 30);
  delay(30);
  noTone(buzzer);
}

void somAfirmativo() {
  tone(buzzer, 440, 200);
  delay(200);
  noTone(buzzer);
  delay(100);
  tone(buzzer, 494, 200);
  delay(200);
  noTone(buzzer);
  delay(100);
  tone(buzzer, 523, 400);
  delay(400);
  noTone(buzzer);
}

void somFimExecucao() {
  tone(buzzer, 440, 50);
  delay(50);
  tone(buzzer, 494, 50);
  delay(50);
  tone(buzzer, 523, 400);
  delay(400);
  tone(buzzer, 494, 50);
  delay(50);
  tone(buzzer, 440, 50);
  delay(50);
  tone(buzzer, 391, 400);
  delay(400);
  noTone(buzzer);
}

void somGravando() {
  tone(buzzer, 1047, 30);
  delay(30);
  noTone(buzzer);
  delay(100);
  tone(buzzer, 1047, 30);
  delay(30);
  noTone(buzzer);
  delay(100);
  tone(buzzer, 1047, 30);
  delay(30);
  noTone(buzzer);
}

int block(int b){ 
  Serial.print(F("Lendo RFID bloco "));
  Serial.print(b);
  Serial.print(":");
  String c="";
  for (byte i = b; i < b+4; i++) {
    int a = buffer[i];
    Serial.print(" ");
    Serial.print(a);
    if (a>47 && a<58)
    c.concat(char(buffer[i]));
  }
  Serial.print(" -> ");
  return stringToInt(c);
}

int stringToInt(String minhaString) { //recebe uma string e transforma em inteiro
  int  x;
  int tam = minhaString.length() - 1;
  int numero = 0;
  for(int i = tam; i >= 0; i--) {
    x = minhaString.charAt(i) - 48;
    numero += x * potencia(10, tam - i);
  }
  Serial.println(numero);
  return numero;
}

void mensagemDebug(String mensagem){
  /*
  Serial.print(millisAtual);
  for(int i=0; i<repeticaoAninhada;i++){
    Serial.print(F("\t"));
  }
  Serial.print(" - ");
  if(caminhando) Serial.print("<C> ");
  if(executando) {
    Serial.print(F("|E| "));
  }  else {
    Serial.print(F("|G| "));
  }
  Serial.println(mensagem);
  */
  String msg = "";
  msg.concat(String(millis()/1000.0,2));
  msg.concat(">");
  msg.concat(mensagem);
  Serial.println(msg);
  if(oledPresent){
    u8g.firstPage(); 
    strcpy(term1, term2); 
    strcpy(term2, term3);
    strcpy(term3, term4);
    strcpy(term4, term5);
    msg.toCharArray(term5, 24);
    do {
      u8g.drawBitmapP( 0, 0, 2, 9, decabot_icon);
      u8g.setPrintPos(12,8);
      u8g.print(String(decabotName).substring(0,4));
      if(ultrasoundPresent) u8g.drawBitmapP( 63, 0, 2, 11, ultrasound_icon);
      if(rfidAlwaysOn) u8g.drawBitmapP( 77, 0, 2, 11, rfid_icon);
      if(gyroPresent) u8g.drawBitmapP( 96, 0, 2, 11, gyro_icon);
      u8g.drawBitmapP( 110, 0, 2, 11, bat_4);
      u8g.drawBitmapP( 46, 59, 5, 5, decano_small_logo);
      u8g.setPrintPos(0,18);
      u8g.print(term1);
      u8g.setPrintPos(0,28);
      u8g.print(term2);
      u8g.setPrintPos(0,38);
      u8g.print(term3);
      u8g.setPrintPos(0,48);
      u8g.print(term4);
      u8g.setPrintPos(0,58);
      u8g.print(term5);
    } while( u8g.nextPage() );
  }
  
}

int potencia(int base, int expoente){
  if(expoente == 0)
  return 1;
  else   
  return base * potencia(base, expoente - 1);
}
