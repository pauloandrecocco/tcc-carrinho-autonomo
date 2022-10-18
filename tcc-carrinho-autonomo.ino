// --- Inclusão das bibliotecas ---
#include <Servo.h>
#include <math.h>

// --- Declaração das variáveis ---
const bool // Variáveis que determinam o sentido de rotação dos motores
           estado_pin_I1 = HIGH,
           estado_pin_I2 = LOW,
           estado_pin_I3 = LOW,
           estado_pin_I4 = HIGH;

const byte // Variáveis que determinam as distâncias máxima e mínima de detecção
           d_max = 40,
           d_min = 6,
           // Magnitude do vetor F_t (constante) (definido de acordo com testes no robô)
           magnitude_F_t = 40,
           // Valor do delay usado no final da medição de um sensor ultrassônico
           delay_ultrass = 10;

int // Variáveis usadas para mover o servomotor
    posicao_desejada,
    posicao_atual,
    // Variáveis usadas pra mudar a velocidade do motor
    vel_desejada,
    vel_atual;

float // Armazena o ângulo do vetor resultante
      angulo,
      // Matriz dos vetores (pares ordenados)
      // Linhas:
      // [0]: Vetor esquerda; [1]: Vetor frente; [2]: Vetor direita; [3]: F_r; [4]: F_t; [5]: Vetor resultante
      // Colunas:
      // [0]: x; [1]: y
      vetores[6][2],
      // Magnitude do vetor resultante R
      magnitude_R;

const float // Angulação dos vetores (radianos)
            // [0]: Angulação do vetor gerado pelo sensor da esquerda; [1]: '' '' da frente; [2]: '' '' da direita
            ang_vetores[3] = {(120-180)*(3.14159265/180)/*120º*/, (90-180)*(3.14159265/180)/*90º*/, (60-180)*(3.14159265/180)/*60º*/};

long // Variáveis usadas para o cálculo da distância nos sensores ultrassônicos
     duracao,
     distancia,
     // Variável que armazena os valores das distâncias medidas dos sensores (cm)
     // Linha:
     // [0]: Medição do sensor da esquerda; [1]: Medição da frente; [2]: Medição da direita
     // Coluna:
     // [0]: Medição N-2; [1]: Medição N-1; [2]: Medição N; [3]: Média das 3 medições
     sensores[3][4];

// Portas do Arduino
const byte pin_trigger_esq = 2,
           pin_echo_esq = 3,
           pin_trigger_frente = 4,
           pin_echo_frente = 5,
           pin_trigger_dir = 7,
           pin_echo_dir = 8,
           pin_vel = 6, // Os dois motores estão ligados ao mesmo pino de velocidade
           pin_servo = 13, // Controle do servo
           pin_IN1 = A0,
           pin_IN2 = A1,
           pin_IN3 = A2,
           pin_IN4 = A3;

Servo servo; // Cria um objeto servo



// --- Início do programa --- --- Início do programa --- --- Início do programa --- 

// Rotina de Interrupção do Timer2 (8 bits)
// Será usado para mover o servomotor em etapas, de forma mais suave
// Também usado para mudar a rotação dos motores, de forma suave
ISR(TIMER2_OVF_vect) // Interrupt Service Routine (Timer2 por overflow)
{
  // Reinicializa o registrador do Timer2, o valor de contagem do Timer2
  // Conta de 100 até 256 (8 bits)
  TCNT2=100;

  
  // Aqui faz o que tem que fazer a cada interrupção
  
  // Move o servo em etapas, da posição atual até a posição desejada, a cada estouro do Timer2
  if(posicao_atual < posicao_desejada) // Tem que ir mais à direita
  {
    posicao_atual++;
    servo.write(180-posicao_atual);
  }
  else if(posicao_atual > posicao_desejada) // Tem que ir mais à esquerda
  {
    posicao_atual--;
    servo.write(180-posicao_atual);
  }
  // Se for igual, não faz nada, já está no ângulo certo

  // Mudança da velocidade
  // 0 - - - - - - - 255
  if(vel_atual < vel_desejada)
  {
    vel_atual = vel_atual + 5;
    analogWrite(pin_vel, vel_atual);
  }
  else if(vel_atual > vel_desejada)
  {
    vel_atual = vel_atual - 5;
    analogWrite(pin_vel, vel_atual);
  }
  // Se for igual, não faz nada, já está na velocidade certa  
}

// Procedimento que faz a medição de distância de um sensor ultrassônico (cm)
void SonarSensor(int trigPin,int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // Manda +5V por 10 microsegundos para ativar o Trigger
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  duracao = pulseIn(echoPin, HIGH); // Conta o tempo para o sensor receber o sinal do Trigger
  distancia = duracao*340/20000; // Em centímetros
  // Distancia = ("travel time"/2)*velocidade do som
  // "Travel time" = pulseIn(), dá o resultado em microssegundos. Converter para segundos: dividir por 1.000.000 (1s = 1000000us)
  // Queremos a distância em cm, em vez de m. Temos que multiplicar por 1000 (1m = 1000cm)
  // (340/2)/(1000000)*(1000) = 340/20000
}

void medir_distancias()
{
  // Zona de detecção: entre 0 e d_max
  // Se obstáculo está mais longe que d_max, o sensor vai medir só até d_max, assume valor máximo (d_max)
  // Faz média das 3 últimas medições
  
  // Sensor Esquerda:
  SonarSensor(pin_trigger_esq, pin_echo_esq);
  if(distancia > d_max)
  {
    distancia = d_max;
  }
  sensores[0][2] = distancia; // Coluna [2]: Medição atual
  // Fazer média e salvar na coluna [3] (média das medições)
  sensores[0][3] = (sensores[0][0] + sensores[0][1] + sensores[0][2])/3;
  // Deslocar valores medidos (medição atual vira última medição, ultima vira penúltima...), libera o lugar da medição atual para a próxima
  for(int i=0; i<2; i++)
  {
    sensores[0][i] = sensores[0][i+1];
  }
  delay(delay_ultrass);

  // Sensor Frente:
  SonarSensor(pin_trigger_frente, pin_echo_frente);
  if(distancia > d_max)
  {
    distancia = d_max;
  }
  sensores[1][2] = distancia; // Coluna [2]: Medição atual
  // Fazer média e salvar na coluna [3] (média das medições)
  sensores[1][3] = (sensores[1][0] + sensores[1][1] + sensores[1][2])/3;
  // Deslocar valores medidos (medição atual vira última medição, ultima vira penúltima...), libera o lugar da medição atual para a próxima
  for(int i=0; i<2; i++)
  {
    sensores[1][i] = sensores[1][i+1];
  }
  delay(delay_ultrass);

  // Sensor Direita:
  SonarSensor(pin_trigger_dir, pin_echo_dir);
  if(distancia > d_max)
  {
    distancia = d_max;
  }
  sensores[2][2] = distancia; // Coluna [2]: Medição atual
  // Fazer média e salvar na coluna [3] (média das medições)
  sensores[2][3] = (sensores[2][0] + sensores[2][1] + sensores[2][2])/3;
  // Deslocar valores medidos (medição atual vira última medição, ultima vira penúltima...), libera o lugar da medição atual para a próxima
  for(int i=0; i<2; i++)
  {
    sensores[2][i] = sensores[2][i+1];
  }
  delay(delay_ultrass);
}



// SETUP SETUP SETUP
void setup()
{
  Serial.begin (9600);
  
  pinMode(pin_trigger_esq, OUTPUT);
  pinMode(pin_trigger_frente, OUTPUT);
  pinMode(pin_trigger_dir, OUTPUT);
  pinMode(pin_echo_esq, INPUT);
  pinMode(pin_echo_frente, INPUT);
  pinMode(pin_echo_dir, INPUT);
  pinMode(pin_IN1, OUTPUT);
  pinMode(pin_IN2, OUTPUT);
  pinMode(pin_vel, OUTPUT);
  pinMode(pin_IN3, OUTPUT);
  pinMode(pin_IN4, OUTPUT);

  servo.attach(pin_servo); // Anexa o servomotor (físico), no pino "pin_servo", ao objeto "servo" (lógico)

  // Servomotor começa posicionado no meio
  posicao_desejada = 90;
  posicao_atual = posicao_desejada;
  servo.write(posicao_atual);

  // Começa parado, pronto pra andar pra frente
  vel_desejada = 0;
  vel_atual = vel_desejada;
  // Programa o sentido de rotação dos motores
  digitalWrite(pin_IN1, estado_pin_I1);
  digitalWrite(pin_IN2, estado_pin_I2);
  digitalWrite(pin_IN3, estado_pin_I3);
  digitalWrite(pin_IN4, estado_pin_I4);

  // Cofiguração de registradores (estouro do Timer2, a cada 10 ms)
  TCCR2A = 0x00;   // Timer operando em modo normal
  TCCR2B = 0x07;   // Prescaler 1:1024
  TCNT2  = 100;    // Valor inicial de contagem, até chegar em 256 e estourar, dá 10 ms
  TIMSK2 = 0x01;   // Habilita interrupção do Timer2
  // Estouro = Timer2_cont x prescaler x ciclo de máquina
  // Ciclo de máquina = 1/Fosc = 1/16E6 = 62,5ns = 62,5E-9s
  // Estouro = (256 - 100) x 1024 x 62,5E-9 = 9,98ms

  // Realiza as duas primeiras medições dos sensores
  for(int i=0; i<2; i++)
  {
    // Esquerda
    SonarSensor(pin_trigger_esq, pin_echo_esq);
    if(distancia > d_max)
    {
      distancia = d_max;
    }
    sensores[0][i] = distancia;
    delay(delay_ultrass);
    // Frente
    SonarSensor(pin_trigger_frente, pin_echo_frente);
    if(distancia > d_max)
    {
      distancia = d_max;
    }
    sensores[1][i] = distancia;
    delay(delay_ultrass);
    // Direita
    SonarSensor(pin_trigger_dir, pin_echo_dir);
    if(distancia > d_max)
    {
      distancia = d_max;
    }
    sensores[2][i] = distancia;
    delay(delay_ultrass);
  }

  // Cria o vetor F_t (vetor de atração para frente - o carrinho procura sempre ir pra frente)
  vetores[4][0] = magnitude_F_t*cos(90*(3.14159265/180)/*90º*/); // Componente x do vetor
  vetores[4][1] = magnitude_F_t*sin(90*(3.14159265/180)/*90º*/); // Componente y
}
 
void loop()
{
  // Medir distância dos obstáculos
  medir_distancias();
   
  /*Serial.print(sensores[0][3]);
  Serial.print(" - ");
  Serial.print(sensores[1][3]);
  Serial.print(" - ");
  Serial.println(sensores[2][3]);*/

  // Controle de direção //

  // O carrinho se movimenta baseado na ideia de forças puxando ele para os lados, indo na direção da força resultante
  // Os sensores geram vetores de repulsão: quanto mais próximo o obstáculo (menor a medição), maior o vetor de repulsa, na direção do sensor
  // Vetores em coordenadas cartesianas (pares ordenados)
  for(int i = 0; i < 3; i++) // 1 a 3 (sensores)
  {
    vetores[i][0] = (d_max - sensores[i][3])*cos(ang_vetores[i]); // Componente x do vetor
    vetores[i][1] = (d_max - sensores[i][3])*sin(ang_vetores[i]); // Componente y
  }
  
  // Deve ser criado o vetor F_r (soma vetorial de todos os vetores de repulsa)
  vetores[3][0] =  vetores[0][0] + vetores[1][0] + vetores[2][0];
  vetores[3][1] =  vetores[0][1] + vetores[1][1] + vetores[2][1];

  // Vetor F_t já foi criado no SETUP (vetor de atração para frente - o carrinho procura sempre ir pra frente)

  // Cálculo do vetor resultante entre F_r e F_t
  vetores[5][0] =  vetores[3][0] + vetores[4][0];
  vetores[5][1] =  vetores[3][1] + vetores[4][1];
  
  // Calcular ângulo da direção do vetor resultante da soma vetorial
  angulo = atan2(vetores[5][1], vetores[5][0]) * (180/3.14159265); // Arco tagente de vetores[3][1]/vetores[3][0] * conversão p/ graus
  // -180 <= angulo < 180
  // Se o valor do ângulo resultante estiver entre -180 e -90, está no terceiro quadrante, o carro tem que virar para a esquerda
  if(angulo < -90) // Terceiro quadrante
  {
    angulo = angulo + 360;
  }
  
  // Carrinho só pode virar entre 45º e 135º
  if(angulo > 135)
  {
    posicao_desejada = 135;
  }
  else if(angulo < 45)
  {
    posicao_desejada = 45;
  }
  else
  {
    posicao_desejada = angulo;
  }
  
  /*Serial.print(angulo);
  Serial.print(" - ");
  Serial.println(posicao_desejada);*/

  // Controle da velocidade //
  
  if((sensores[0][3] + sensores[1][3] + sensores[2][3]) < 3*d_min)
  {
    vel_desejada = 0; // Velocidade 0
    // Carrinho freia
    digitalWrite(pin_IN1, HIGH);
    digitalWrite(pin_IN2, HIGH);
    digitalWrite(pin_IN3, HIGH);
    digitalWrite(pin_IN4, HIGH);
  }
  else
  {
    if((angulo < 180) && (angulo > 0)) // Vetor positivo
    {
      magnitude_R = sqrt( sq(vetores[5][0]) + sq(vetores[5][1]) );
      vel_desejada = map(magnitude_R, 0, magnitude_F_t, 60, 150); // Velocidade é "proporcional" ao tamanho do vetor resultante*/
    }
    else // Vetor negativo
    {
      // Velocidade mínima
      vel_desejada = 60;
    }
    // Motores rodam no sentido programado
    digitalWrite(pin_IN1, estado_pin_I1);
    digitalWrite(pin_IN2, estado_pin_I2);
    digitalWrite(pin_IN3, estado_pin_I3);
    digitalWrite(pin_IN4, estado_pin_I4);
  }
  /*Serial.println(min(sensores[0][3], min(sensores[1][3], sensores[2][3])));*/
}
