/********1*********2*********3*********4*********5*********6*********7*********8
* File : i2c_master_byte_ctrl.v
*_______________________________________________________________________________
*
* Revision history
*
* Name   Marc, Erwin       Date   05/04/2024     Observations
* ------------------------------------------------------------------------------
*_______________________________________________________________________________
*
* Description
* Byte Command Control
*_______________________________________________________________________________

* (c) Copyright Universitat de Barcelona, 2024
*
*********1*********2*********3*********4*********5*********6*********7*********/

/*---------
  Includes
----------*/
`include "../misc/timescale.v"  
`include "i2c_master_defines.v" 

module i2c_master_byte_ctrl(
  /*-----
    I/O
  ------*/
  /*SYSTEM*/
  //1b wire inputs. Inputs must be set as wires
  input Clk,   //Rellotge del sistema
  input Rst_n, //Reset asincron actiu per flanc de baixada

  /*COMMAND REGISTER*/
  //1b wire inputs. Inputs must be set as wires
  input Start, //CR[7]. Bit per indicar que es vol generar o repetir una condicio d'start. 0: no generar condicio, 1: generar condicio
  input Stop,  //CR[6]. Bit de generacio de la condicio d'Stop.                            0: no generar condicio, 1: generar condicio
  input Read,  //CR[5]. Bit que indica si es vol fer una transferencia de lectura.         0: no es vol llegir, 1: es vol llegir de l'esclau
  input Write, //CR[4]. Bit que indica si es vol fer una transferencia d'escriptura.       0: no es vol escriure, 1: es vol escriure de l'esclau

  /*STATUS*/ 
  //1b wire inputs. Inputs must be set as wires
  input      Tx_ack,   //Acknowledge en mode receptor que el master envia a l'slave.         0: enviat a l'esclau, 1: NACK per l'ultim byte
  input      I2C_al,   //Flag d'arbitrarion lost.                                            0: no hi ha col.lisio i no s'ha perdut l'arbitrarietat, 1: col.lisio detectada i s'ha perdut l'arbitrarietat
  //LHS outputs inside always must be set as reg <=
  output reg Rx_ack,   //SR[7]. Ackowledge en mode transmissor que el master rep de l'slave. 0: rebut de l'esclau, 1: no rebut de l'esclau
  output reg I2C_done, //S'ha completat una operacio de byte.                                0: no s'ha completat una operacio de byte, 1: s'ha completat una operacio de byte i used to clear command register I2C bus arbitration lost CR[7:4]

  /*SHIFT REGISTER*/
  //1b wire inputs. Inputs must be set as wires
  input      SR_sout,  //Sortida de dades serie del shift reg. Es connecta al byte controller per rebotar la dada cap al bit controller i aquest treure-la cap al SDA bit a bit mitjancant les sequencies
  //LHS outputs inside always must be set as reg <=
  output reg SR_load,  //Habilitara la carrega de dades en paral.lel. Alhora serveix per recarregar el timer
  output reg SR_shift, //Habilitara el desplacament cap a l'esquerra del shiftreg. Alhora serveix per restar una unitat al counter

  /*BIT CONTROLLER*/
  //1b wire inputs. Inputs must be set as wires
  input            Bit_ack, //command complete acknowledge, flag que arriba del bit controller que marca que ha acabat una sequencia d'instruccions
  input            Bit_rxd, //Lectura serie. Data received from bit controller also received from bus SDA. Els bits van al shiftreg (dades) i l'ack va al Bit_ack
  //LHS outputs inside always must be set as reg <=
  output reg [3:0] Bit_cmd, //Instruccio que s'envia al bit controller. Cada comanda es la sequencia que es genera a les linies SCL i SDA
  output reg       Bit_txd  //Escriptura serie. Data to transmit al bit controller que alhora ho enviara al bus SDA bit a bit
);

  /*------------------------------------
   internal. must be set in lower case
  -------------------------------------*/
  /*BYTE COUNTER*/
  //Nb internal reg
  reg [2:0] counter; //Per una unica operacio de byte (lectura o escriptura) el bit controller rep del byte controller 8 comandes separades de write o read. La primera comanda es generara a l'estat anterior a read o write, de manera que nomes quedaran 7b per tractar
  reg counter_done;  //Indica que s'ha acabat una operacio de byte. 0: encara no s'han comptat els 7b restants, 1: s'han comptat els 7b restants.

  /*STATE MACHINE*/
  //Definicio variables d'estat
  reg [2:0] curr_state, next_state; //Tenim 6 estats, codificacio binaria => necessitem 3 bits
  //State machine FSM decoder
  parameter [2:0] //synopsys enum code
                  IDLE  = 3'd0,
                  START = 3'd1,
                  WRITE = 3'd2,
                  ACK   = 3'd3,
                  STOP  = 3'd4,
                  READ  = 3'd5;

  /*------------
   Byte counter
  -------------*/
  /*--------------------------------------------------------------------------------------------------------------------------------------------------------------------
   * El byte controller ha d'enviar al bit controller 8 vegades les instruccions que requereixen tractar amb unitats de byte, es a dir, lectura i escriptura. Notis com
   * el nostre comptador es decreixent de 7 a 0, de manera que el primer bit que s'escriura o es llegira es fara en l'estat anterior al read o write i es fa el reload
   * del timer en aquell mateix moment. Per tant, com ja s'haura tractat 1b de manera 'emmascarada' (a l'ultim FF del shiftreg ja que tenim la dada serie a la sortida)
   * nomes quedaran altres 7 iteracions que es controlaran a través del timer.
   *
   * if Rst_n    == 0: buidem el valor del comptador
   * if SR_load  == 1: posem el comptador al valor maxim per fer la lectura o escriptura. Es posa a 1 un estat abans de read o write, es a dir, al START o ACK. En aquests mateixos estats enviarem o rebrem el 1r bit
   * if SR_shift == 1: cada vegada que s'hagi completat una sequencia en el bit controller restem 1b ja que aquest pasara el Bit_ack al byte controller com a confirmacio
   * else: com el clk va mes rapid que el Bit_ack hi hauran moments on simplement s'haura de mantenir el valor de counter
  --------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
  /*Assign*/
  //fem un assign per no perdre un altre cicle de rellotge en actualitzar l'out, ja que en fer.ho dins l'always s'actualitzaria en el seguent clock en cas de ser out reg
  assign counter_done = ~(|counter); //quan counter hagi comptat 7b activem la flag counter_done

  always @(posedge Clk or negedge Rst_n) begin
    if      (!Rst_n)   begin counter <= 3'd0;           end
    else if (SR_load)  begin counter <= 3'd7;           end
    else if (SR_shift) begin counter <= counter - 1'd1; end
    else               begin counter <= counter;        end
  end

  /*--------------
    State machine
  ---------------*/
  /*--------------------------------------------------------------------------------------------------------------------------------------------------------------------
   * IDLE:  estat on les linies SDA i SCL estan en pullup esperant a que es doni una condicio d'Start. Si es dona aquesta condicio el seguent estat es START.
   * START: en la transicio de l'IDLE a l'START s'ha enviat la comanda d'Start al bit controller perque generi la sequencia corresponent. Quan s'hagi completat aquesta
   *        comanda el bit controller ens retornara un Bit_ack com a confirmacio, de manera que si a mes el senyal Write esta activat canviarem d'estat. El seguent
   *        estat nomes pot ser WRITE, ja que enviar el header es considera operacio d'escriuptura.
   * WRITE: en la transicio del START/ACK al WRITE s'ha enviat al bit controller la comanda d'escriure el 1r bit (sortida serie de l'ultim FF). En aquest moment es
   *        carrega al valor maxim el counter i mentre no s'hagi acabat d'escriure el header ens quedarem en aquest estat. Una vegada finalitzat i rebut el Bit_ack de
   *        de l'ultima sequencia canviarem d'estat. El seguent estat nomes pot ser ACK, ja que despres d'una escriptura a l'esclau sempre esperarem un bit d'ACK
   * ACK:   en la transicio del WRITE al ACK s'ha enviat una comanda de lectura per llegir el bit d'ack que ens ha enviat el bit controller que alhora ha rebut de la
   *        linia SDA. Aquesta dada es volcara a l'status register. El seguent estat pot ser una escriptura o lectura en cas de que es tracti del primer Ack rebut
   *        (header) o d'un Stop en cas de que sigui el segon Ack rebut de l'Slave (referint-nos a una trama I2C normal)
   * READ:  en la transicio del ACK al READ s'ha enviat al bit controller la comanda de llegir el 1r bit procedent del bit controller alhora del SDA. COUNTER???
   * STOP:  en la transicio de l'estat ACK al STOP despres del segon bit d'Ack, ja siqui havent fet una escriptura(ACK) o lectura(NACK), s'ha enviat la comanda d'Stop
   *        al bit controller, on d'espres d'executar la sequencia en retornara el Bit_ack i passarem a l'estat IDLE.
  --------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
  
  /*NEXT STATE LOGIC*/
  //Definim bloc combinacional, calcul estat futur
  always @(*) begin
    case(curr_state) //
      IDLE:  if     (Start)                   begin next_state = START; end
             else if(Read)                    begin next_state = READ;  end
             else if(Write)                   begin next_state = WRITE; end
             else if(Stop)                    begin next_state = STOP;  end
             else                             begin next_state = IDLE;  end
      START: if     (Bit_ack && Read)         begin next_state = READ;  end
             else if(Bit_ack && Write)        begin next_state = WRITE; end
             else                             begin next_state = START; end
      READ:  if     (Bit_ack && counter_done) begin next_state = ACK;   end
             else                             begin next_state = READ;  end
      WRITE: if     (Bit_ack && counter_done) begin next_state = ACK;   end
             else                             begin next_state = WRITE; end
      ACK:   if     (Bit_ack && Stop)         begin next_state = STOP;  end
             else if(Bit_ack && ~Stop)        begin next_state = IDLE;  end
             else                             begin next_state = ACK;   end
      STOP:  if     (Bit_ack)                 begin next_state = IDLE;  end
             else                             begin next_state = STOP;  end
      default:                                begin next_state = IDLE;  end
    endcase
  end

  /*PRESENT STATE FF's*/
  //Definim logica sequencial (estat futur => estat actual)
  always @(posedge Clk or negedge Rst_n) begin
    if (!Rst_n) begin curr_state <= IDLE;       end //En cas de Rst_n portem la FSM a l'estat IDLE.
    else        begin curr_state <= next_state; end //Actualitzacio de l'estat a cada cicle de rellotge.
  end

  /*OUTPUT LOGIC*/
  //Definim logica combinacional registrada de sortida
  always @(posedge Clk or negedge Rst_n) begin
    if (!Rst_n) begin
      Bit_cmd  <= `I2C_CMD_NOP;
      SR_load  <= 1'b0;
      SR_shift <= 1'b0;
      I2C_done <= 1'b0;
      Bit_txd  <= 1'b0;
      Rx_ack   <= 1'b1;
    end
    else if (I2C_al) begin
      Bit_cmd  <= `I2C_CMD_NOP;
      SR_load  <= 1'b0;
      SR_shift <= 1'b0;
      I2C_done <= 1'b0;
      Bit_txd  <= 1'b0;
      Rx_ack   <= 1'b1;
    end
    else begin
      Bit_cmd  <= Bit_cmd;
      Bit_txd  <= SR_sout;
      I2C_done <= 1'b0;
      SR_load  <= 1'b0;
      SR_shift <= 1'b0;
      Rx_ack   <= 1'b1; //Rx_ack es actiu per nivell baix, de moment no s'ha rebut cap ACK del slave

      case(curr_state)
        IDLE: begin
          SR_load <= 1'b1; //volquem en paral.lel les dades cap al shiftreg (W) o simplement per carregar el valor al maxim del counter (R)
          
          if      (Start) begin 
            Bit_cmd <= `I2C_CMD_START;
          end
          else if (Read)  begin 
            Bit_cmd <= `I2C_CMD_READ;
          end
          else if (Write) begin 
            Bit_cmd <= `I2C_CMD_WRITE;
          end
          else if (Stop)  begin 
            Bit_cmd <= `I2C_CMD_STOP;
          end
        end
        START: begin
          SR_load <= 1'b1; //refresh del counter, encara q realment es carreguin dades acabarant sent desplacades //volquem les dades en paral.lel al shift reg i fem refresh counter
 
          if     (Bit_ack && Read) begin
            Bit_cmd <= `I2C_CMD_READ; //enviem la sequencia de lectura per llegir el bit de la linia SDA
          end
          else if(Bit_ack && Write) begin
            Bit_cmd <= `I2C_CMD_WRITE; //enviem la sequencia d'escriptura per escriure el bit que surt de l'ultim FF del shift reg i aixi nomes fer el counter de 7b
          end
        end
        READ: begin
          Bit_txd  <= Tx_ack; //despres de la lectura responem amb un l'ACK corresponent com a master

          if(Bit_ack && counter_done) begin
            Bit_cmd <= `I2C_CMD_WRITE; //despres de llegir enviem la comanda al bit controller d'escriptura per enviar l'ack del master
            SR_shift <= 1'b1; //anem desplacant cap al fons les dades que van entrant en serie pel shiftreg
          end
          else if (Bit_ack) begin
            Bit_cmd <= `I2C_CMD_READ; //cada vegada que s'hagi completat una instruccio de lectura al bit controller enviem la seguent per recorrer tots els bits
            SR_shift <= 1'b1; //anem desplacant cap al fons les dades que van entrant en serie pel shiftreg
          end
        end
        WRITE: begin
          if(Bit_ack && counter_done) begin
            Bit_cmd  <= `I2C_CMD_READ; //despres d'escriure enviem la comanda al bit controller de lectura per rebre l'ack del slave 
          end
          else if (Bit_ack) begin
            Bit_cmd  <= `I2C_CMD_WRITE; //seguim enviant sequencies d'escriptura cap al bit controller mentre no s'hagi buidat el counter
            SR_shift <= 1'b1;           //shift de les dades, es van treient pel shift reg en serie
          end
        end
        ACK: begin
          if(Bit_ack && Stop) begin
            Bit_cmd  <= `I2C_CMD_STOP;
            Bit_txd  <= 1'b1; //SEND NACK
            Rx_ack   <= Bit_rxd; //llegim el bit d'ack que ha enviat l'slave i el volquem cap a Rx_ack (que el passara cap al SR[7])
          end
          else if(Bit_ack && ~Stop) begin
            Bit_cmd  <= `I2C_CMD_NOP; //
            I2C_done <= 1'b1; //clear de CR[7:4]
            Bit_txd  <= 1'b0;
            Rx_ack   <= Bit_rxd; //llegim el bit d'ack que ha enviat l'slave i el volquem cap a Rx_ack (que el passara cap al SR[7])
          end
          else begin
            Bit_txd <= Tx_ack;
          end
        end
        STOP: begin
          if(Bit_ack) begin
            Bit_cmd  <= `I2C_CMD_NOP;
            I2C_done <= 1'b1;
          end
        end
        default: begin
          Bit_cmd  <= `I2C_CMD_NOP;
          SR_load  <= 1'b0;
          SR_shift <= 1'b0;
          I2C_done <= 1'b0;
          Bit_txd  <= 1'b0;
          Rx_ack   <= 1'b1;
        end
      endcase
    end
  end

endmodule
