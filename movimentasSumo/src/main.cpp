#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

// Forma que Jadson opera os BITS
#define SET_BIT(VALUE, BIT) (VALUE |= (1 << BIT))
#define CLEAR_BIT(VALUE, BIT) (VALUE &= ~(1 << BIT))
#define TOGGLE_BIT(VALUE, BIT) (VALUE ^= (1 << BIT))
#define GET_BIT(VALUE, BIT) ((VALUE >> BIT) & 1)

// Para usar a Serial
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1 // Sei muito bem o que é não, vi na net
char str[4]; // Buffer pra armazenar os números em String

// Pulsos lidos
volatile uint16_t timer_value_0 = 0; // Para o PD2 (INT0) (Dig 2) (FRONTAL)
volatile uint16_t timer_value_1 = 0; // Para o PD3 (INT1) (Dig 3) (LATERAL)
// Pulsos convertidos
uint16_t inputThrottle = 0; // Para frente (Baseado no INT0)
uint16_t inputLateral = 0; // Para os lados (Baseado no INT1)

// Variáveis mágicas pra leitura do ppm
uint8_t flag_0 = 0, flag_1 = 0;

// Variáveis para o output do PWM
uint8_t TOP = 255; // Se for usar Timer1 isso deve ser = a ICR1 e tem que ser 16bit
uint16_t inputFreio = 1000;

// Variáveis float para o throttle
double inputThrottle_double = 0.0;
double inputLateral_double = 0.0;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// PARTE DA LEITURA

// Interrupção do INT0 (PD2)
ISR(INT0_vect){
  if (GET_BIT(PIND, 2)){ // Detecta 0 -> 1
    TCNT1 = 0x0000; // Zerando o Timer pra medir o pulso
  }
  else{
    timer_value_0 = TCNT1; // Armazena o valor do Timer (Ticks)
    flag_0 = 1;
  }
}

// Interrupção do INT1 (PD3)
ISR(INT1_vect){
  if (GET_BIT(PIND, 3)){ // Detecta 0 -> 1
    TCNT1 = 0x0000; // Zerando o Timer para medir o pulso
  }
  else{
    timer_value_1 = TCNT1; // Armazena o valor do Timer (Ticks)
    flag_1 = 1;
  }
}


// Função para inicializar a USART
void USART_Init(unsigned int ubrr) {
    // Definir o baud rate
    UBRR0H = (unsigned char)(ubrr >> 8);  // Parte alta
    UBRR0L = (unsigned char)ubrr;         // Parte baixa

    // Habilitar transmissão e recepção
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);

    // Definir o formato da frame: 8 bits de dados, 1 bit de parada
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); 
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
// PARTE DA SERIAL

// Função que envia os dados na Serial
void USART_Transmit(unsigned char data){
  while (!(UCSR0A & (1 << UDRE0))); 
  UDR0 = data;
}

// Função que desempacota a String pra transmitir na Serial
void serialPrint(const char *str){
  while (*str){
    USART_Transmit(*str++);
  }
}

// A de cima só que modificada pra fazer linha nova kkk
void serialPrintln(const char *str){
  while(*str){
    USART_Transmit(*str++);
  }

  USART_Transmit('\r');
  USART_Transmit('\n');
}


// Func de conversão de Ticks -> us
uint16_t ticks_to_us(uint16_t ticks){
  uint16_t us = (float)ticks/1000;
  us = us*62.5; 
  return (uint16_t)us;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// PARTE DO OUTPUT PWM

// Funçãozinha marota só pra ficar elegante lá embaixo
float limitasValores(float valor, float minimo, float maximo){
  if (valor < minimo){
    return minimo;
  }
  else if (valor > maximo){
    return maximo;
  }
  else{
    return valor;
  }
}

// Boas anotações pra daqui a 3 meses quando eu revisitar esse código (ass: Taylas)
// =============================
  // PARA O TIMER1
  // Frequências possíveis com Fast PWM (N=Prescale)
  // f_pwm = f_clk/(N*256)
  //       = 16mHz/256N
  // Como mudar o prescale:
  // CS12 CS11 CS10 -> Prescaler -> f_pwm (considerando 16MHz de entrada)
  //   0   0    0   -> Inativo   -> -
  //   0   0    1   ->   1       -> 16 MHz
  //   0   1    0   ->   8       -> 2 MHz
  //   0   1    1   ->   64      -> 250 KHz
  //   1   0    0   ->   256     -> 62,5 KHz
  //   1   0    1   ->   1024    -> 15,625 KHz
  // Essa config trás a FREQUÊNCIA BASE do PWM, então dá pra alterar com o TOP qual vai ser
  // A freq final do pwm
  // Lembra sempre de conferir o WGM1[x] para ver qual o modo do PWM

  // O TOP do PWM (IMPORTANTE ISSO HEIN) pode ser calculado como:
  // TOP = (f_clk/(N*f_pwm)) - 1
  // Dessa vez, o f_pwm vai ser a frequência final do PWM, a que vai ser gerada
  // O valor do ICR1 deve ser definido com o valor do TOP
  // Ex: Freq de 10416Hz: 
  // Seleciona-se o Prescale de 256, no fim o cálculo de TOP entregará TOP=5
  // Equivalendo a 10,416 KHz


  // =============================
  // PARA O TIMER0
  // f_pwm = f_clk/(N*256)
  //       = 16mHz/256N
  // Como mudar o prescale:
  // CS12 CS11 CS10 -> Prescaler -> f_pwm (considerando 16MHz de entrada)
  //   0   0    0   -> Inativo   -> -
  //   0   0    1   ->   1       -> 62,5 kHz
  //   0   1    0   ->   8       -> 7,8125 kHz
  //   0   1    1   ->   64      -> 976,5625 Hz
  //   1   0    0   ->   256     -> 244,1406 Hz
  //   1   0    1   ->   1024    -> 61,035 Hz

  // No Timer0 o TOP é fixo em 255 (de 0 a 255), então a freq é sempre baseada nesse prescaler


void setup_fastPWM(){
  // Pinos D5 e D6 (configurando como output)
  DDRD |= (1 << DDD5 | 1 << DDD6);

  // Configurando o Timer0 para Fast PWM não-invertido
  TCCR0A |= (1 << WGM00 | 1 << WGM01 | 1 << COM0A1 | 1 << COM0B1);
  TCCR0B |= (1 << CS11 | 1 << CS10); // Prescalers

  // Iniciando com o Duty Cycle zerado
  OCR0A = 0; 
  OCR0B = 0;
}

uint8_t setDutyCycle(float porcentPWM){
  // porcentPWM deve estar entre 0-1
  // Essa função lê valores de 0-1 e faz o Duty Cycle adotar esse valor

  // Isso é para limitar o valor mínimo do Duty Cycle
  porcentPWM = limitasValores(porcentPWM, 0, 1);

  uint8_t dutyCycle = porcentPWM * TOP;
  return dutyCycle;
}




int main(void){
  // Ativando a Serial
	USART_Init(MYUBRR);

  //////////////////////////////////////////////////////////////////////////////////
  // Ativando as configs para as leituras do PPM
  // Configs do Timer1
  TCCR1A = 0x00; // Modo normal
  TCCR1B = (1 << CS10); // Prescaler 1 (Cada tick equivale 62.5ns)
  TCNT1 = 0x0000; // Iniciando o Timer1 zerado

  // Configurando pinos de entrada (PD2 e PD3)
  CLEAR_BIT(DDRD, 2);
  CLEAR_BIT(DDRD, 3);

  // Habilitando interrupções externas
  EICRA = (1 << ISC00) | (1 << ISC10); // Pra qualquer mudança dos pinos INT0 e INT1
  EIMSK = (1 << INT0) | (1 << INT1); // Habilitnado INT0 e INT1
  sei(); 

  //////////////////////////////////////////////////////////////////////////////////
  // Ativando as configs para a geração do PWM e controle da Ponte H
  setup_fastPWM();
  
  // BASEADO NA PONTE H L298N
  // TABELA VERDADE:
  // IN1   IN2   -> SENTIDO
  //  0     1    -> FREIO DE MÃO
  //  1     0    -> Frente
  //  0     1    -> Ré
  //  1     1    -> FREIO DE MÃO

  // PINOS UTILIZADOS PRA DIREÇÃO (configurando como OUTPUT)
  // Motor direito:
  DDRD |= (1 << DDD7); // D7
  DDRB |= (1 << DDB0); // D8
  // IN1 = D7, IN2 = D8

  // Motor esquerdo:
  DDRB |= (1 << DDB4 | 1 << DDB5); // D12 e D13
  // IN3 = D12, IN4 = D13

  // Inicializando ambos freados
  // Motor DIREITO
  PORTD |= (1 << PD7);
  PORTB |= (1 << PB0);

  // Motor ESQUERDO
  PORTB |= (1 << PB4);
  PORTB |= (1 << PB5);
	
	while(1){
    //////////////////////////////////////////////////////////////////////////////////
    // Leituras de PPM abaixo
		if (flag_0){
      inputThrottle = ticks_to_us(timer_value_0);
      // Pra verificar to enviando via Serial
      /*
      sprintf(str, "%d", inputThrottle);
      serialPrintln(str);
      */
      flag_0 = 0; 
    }
		
    if (flag_1){
      inputLateral = ticks_to_us(timer_value_1);
      // Pra verificar to enviando via serial
      /*
      sprintf(str, "%d", inputLateral);
      serialPrintln(str);
      */
      flag_1=0;
    }

    //////////////////////////////////////////////////////////////////////////////////
    // Controle do robô e geração de PWM abaixo
    // Aqui configura uma chave de ativação no controle, quando ela for ativa vai ter um valor x (botei >=1800 ficticio), e engatilha a operação
    // em us, e a partir daí faz a operação de frenagem do robô (a ser pensada)
    
    if (inputFreio >= 1800){
      // por enquanto tá só travando os motores
      PORTD &= ~(1 << PD7);
      PORTB &= ~(1 << PB0 | 1 << PB4 | 1 << PB5);
      // Se MADS passar algo gradual pra frenagem, cria um Flag pra ser acionado enquanto o gatilho estiver ativado
      // Faz a frenagem gradual dos motores, e mantem o estado 0 até que o Flag seja liberado (gatilho desativado)
    }

    // Se não ativar a chave:
    else{
      // Essa etapa leva em consideração valor entre 1000 e 2000 us
      // PROCESSAMENTO DO COMANDO FRENTE/TRÁS
      // Aqui entra o valor lido do radiocontrole (1000 a 2000)
      if (inputThrottle >= 1500){
        inputThrottle_double = limitasValores((inputThrottle - 1500) * 0.002, 0, 1); // Traz o valor para o range 0 a 1 (float)
        // ASSUMINDO AS ROTAÇÕES AÍ
        // Motor DIREITO
        PORTD &= ~(1 << PD7); // IN1
        PORTB |= (1 << PB0); // IN2
        

        // Motor ESQUERDO
        PORTB &= ~(1 << PB4); // IN3
        PORTB |= (1 << PB5); // IN4
      } 

      else if (inputThrottle < 1500){
        inputThrottle_double = limitasValores((1500 - inputThrottle) * 0.002, 0, 1);
        // ASSUMINDO AS ROTAÇÕES AÍ
        // Motor DIREITO
        PORTD |= (1 << PD7); // IN1
        PORTB &= ~(1 << PB0); // IN2
        

        // Motor ESQUERDO
        PORTB |= (1 << PB4); // IN3
        PORTB &= ~(1 << PB5); // IN4
      }
      

      // Tô levando em consideração que:
      // OCR0A -> Motor DIREITO (ENA)
      // OCR0B -> Motor ESQUERDO (ENB)

      // PROCESSAMENTO DO COMANDO LATERAL
      // DIREITA
      if (inputLateral >= 1500){
        inputLateral_double = limitasValores((inputLateral - 1500) * 0.002, 0, 1);
        OCR0A = setDutyCycle(inputThrottle_double - inputLateral_double);
        OCR0B = setDutyCycle(inputThrottle_double);
      }
      // ESQUERDA
      else if (inputLateral < 1500){
        inputLateral_double = limitasValores((1500 - inputLateral) * 0.002, 0, 1);
        OCR0A = setDutyCycle(inputThrottle_double);
        OCR0B = setDutyCycle(inputThrottle_double - inputLateral_double);
      }
    }
  }
}