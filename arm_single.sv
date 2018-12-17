// TRABALHO DE ARQUITETURA -- RAVENA, D�BORA E BRUNO
// arm_single.sv
// David_Harris@hmc.edu and Sarah_Harris@hmc.edu 25 June 2013
// Single-cycle implementation of a subset of ARMv4
// run 210
// Expect simulator to print "Simulation succeeded"
// when the value 7 is written to address 100 (0x64)

// 16 32-bit registers
// Data-processing instructions
//   ADD, SUB, AND, ORR
//   INSTR<cond><S> rd, rn, #immediate
//   INSTR<cond><S> rd, rn, rm
//    rd <- rn INSTR rm	      if (S) Update Status Flags
//    rd <- rn INSTR immediate	if (S) Update Status Flags
//   Instr[31:28] = cond
//   Instr[27:26] = op = 00
//   Instr[25:20] = funct
//                  [25]:    1 for immediate, 0 for register
//                  [24:21]: 0100 (ADD) / 0010 (SUB) /
//                           0000 (AND) / 1100 (ORR)
//                  [20]:    S (1 = update CPSR status Flags)
//   Instr[19:16] = rn
//   Instr[15:12] = rd
//   Instr[11:8]  = 0000
//   Instr[7:0]   = imm8      (for #immediate type) / 
//                  {0000,rm} (for register type)
//   
// Load/Store instructions
//   LDR, STR
//   INSTR rd, [rn, #offset]
//    LDR: rd <- Mem[rn+offset]
//    STR: Mem[rn+offset] <- rd
//   Instr[31:28] = cond
//   Instr[27:26] = op = 01 
//   Instr[25:20] = funct
//                  [25]:    0 (A)
//                  [24:21]: 1100 (P/U/B/W)
//                  [20]:    L (1 for LDR, 0 for STR)
//   Instr[19:16] = rn
//   Instr[15:12] = rd
//   Instr[11:0]  = imm12 (zero extended)
//
// Branch instruction (PC <= PC + offset, PC holds 8 bytes past Branch Instr)
//   B
//   B target
//    PC <- PC + 8 + imm24 << 2
//   Instr[31:28] = cond
//   Instr[27:25] = op = 10
//   Instr[25:24] = funct
//                  [25]: 1 (Branch)
//                  [24]: 0 (link)
//   Instr[23:0]  = imm24 (sign extend, shift left 2)
//   Note: no Branch delay slot on ARM
//
// Other:
//   R15 reads as PC+8
//   Conditional Encoding
//    cond  Meaning                       Flag
//    0000  Equal                         Z = 1
//    0001  Not Equal                     Z = 0
//    0010  Carry Set                     C = 1
//    0011  Carry Clear                   C = 0
//    0100  Minus                         N = 1
//    0101  Plus                          N = 0
//    0110  Overflow                      V = 1
//    0111  No Overflow                   V = 0
//    1000  Unsigned Higher               C = 1 & Z = 0
//    1001  Unsigned Lower/Same           C = 0 | Z = 1
//    1010  Signed greater/equal          N = V
//    1011  Signed less                   N != V
//    1100  Signed greater                N = V & Z = 0
//    1101  Signed less/equal             N != V | Z = 1
//    1110  Always                        any

module testbench();

  logic        clk;
  logic        reset;

  logic [31:0] WriteData, DataAdr;
  logic        MemWrite;
  logic [6:0] cont = 7'b0;

  // instantiate device to be tested
  top dut(clk, reset, WriteData, DataAdr, MemWrite);
  
  // initialize test
  initial
    begin
      reset <= 1; # 22; reset <= 0;
	//reset <= 0;
    end

  // generate clock to sequence tests
  always
    begin
      clk <= 1; # 5; 
      clk <= 0; # 5;
      cont <= cont + 1;
    end

  // check results
  always @(negedge clk)
    begin
      if(reset === 0) begin
        if(cont === 29 ) begin
          $display("Simulation succeeded");
          $stop;
        end
      end
    end
endmodule

module top(input  logic        clk, reset, 
           output logic [31:0] WriteData, DataAdr, 
           output logic        MemWrite);

  logic [31:0] PC, Instr, ReadData;
  logic MemByte;
  
  // instantiate processor and memories
  arm arm(clk, reset, PC, Instr, MemWrite, MemByte, DataAdr, WriteData, ReadData);
  imem imem(PC, Instr);
  dmem dmem(clk, MemByte, MemWrite, DataAdr, WriteData, ReadData); // Memory
endmodule


module dmem(input  logic        clk, memByte, we,
            input  logic [31:0] a, wd,
            output logic [31:0] rd);

  logic [31:0] RAM[63:0];

  always_comb
    casex (a[1:0]) // Carrega da memoria como BYTE de posi��o 
      2'b00: 
	if(memByte)	rd = {24'b0, RAM[a[31:2]][7:0]}; // Carrega da memoria como BYTE de posi��o 1
	else		rd = RAM[a[31:2]]; // Carrega da memoria como bit
      2'b01: rd = {24'b0, RAM[a[31:2]][15:8]}; // Carrega da memoria como BYTE de posi��o 2
      2'b10: rd = {24'b0, RAM[a[31:2]][23:16]};// Carrega da memoria como BYTE de posi��o 3
      2'b11: rd = {24'b0, RAM[a[31:2]][31:24]};// Carrega da memoria como BYTE de posi��o 4
    endcase

  always_ff @(posedge clk)
    if (we)
    	casex (a[1:0]) // Grava na memoria como BYTE de posi��o
		2'b00: 
			if(memByte)	RAM[a[31:2]][7:0] <= wd; // Grava na memoria como BYTE de posi��o  1
			else 		RAM[a[31:2]] <= wd; // Grava na memoria como bit
      		2'b01: RAM[a[31:2]][15:8] <= wd; // Grava na memoria como BYTE de posi��o  2
      		2'b10: RAM[a[31:2]][23:16] <= wd;// Grava na memoria como BYTE de posi��o  3
      		2'b11: RAM[a[31:2]][31:24] <= wd;// Grava na memoria como BYTE de posi��o  4
    	endcase
endmodule

module imem(input  logic [31:0] a,
            output logic [31:0] rd);

  logic [31:0] RAM[63:0];

  initial
      $readmemh("memfile2.dat",RAM); // Contem as instru��es para executar

  assign rd = RAM[a[31:2]]; // word aligned
endmodule

module arm(input  logic        clk, reset,
           output logic [31:0] PC,
           input  logic [31:0] Instr,
           output logic        MemWrite, MemByte,
           output logic [31:0] ALUResult, WriteData,
           input  logic [31:0] ReadData);

  logic [3:0] ALUFlags;
  logic       RegWrite, 
              ALUSrcA, ALUSrcB ,MemtoReg, PCSrc;
  logic [1:0] RegSrc, ImmSrc;
  logic [2:0] ALUControl;

  controller c(clk, reset, Instr[31:12], ALUFlags, 
               RegSrc, RegWrite, ImmSrc, 
               ALUSrcA, ALUSrcB, ALUControl,
               MemWrite, MemByte, MemtoReg, PCSrc);
  datapath dp(clk, reset, 
              RegSrc, RegWrite, ImmSrc,
              ALUSrcA, ALUSrcB, ALUControl,
              MemtoReg, PCSrc,
              ALUFlags, PC, Instr,
              ALUResult, WriteData, ReadData);
endmodule

module controller(input  logic         clk, reset,
                  input  logic [31:12] Instr,
                  input  logic [3:0]   ALUFlags,
                  output logic [1:0]   RegSrc,
                  output logic         RegWrite,
                  output logic [1:0]   ImmSrc,
                  output logic         ALUSrcA, ALUSrcB,
                  output logic [2:0]   ALUControl,
                  output logic         MemWrite, MemByte, MemtoReg,
                  output logic         PCSrc);

  logic [1:0] FlagW;
  logic       PCS, RegW, MemW;
  
  decoder dec(Instr[27:26], Instr[25:20], Instr[15:12],
              FlagW, PCS, RegW, MemW, MemByte, // LDRB
              MemtoReg, ALUSrcA, ALUSrcB, ImmSrc, RegSrc, ALUControl); // LDRB,MOV,EOR,CMP,TST


  condlogic cl(clk, reset, Instr[31:28], ALUFlags,
               FlagW, PCS, RegW, MemW,
               PCSrc, RegWrite, MemWrite);
endmodule

module decoder (
    input  logic [1:0] Op,
    input  logic [5:0] Funct,
    input  logic [3:0] Rd,
    output logic [1:0] FlagW,
    output logic       PCS, RegW, MemW, MemB,
    output logic       MemtoReg, ALUSrcA, ALUSrcB,
    output logic [1:0] ImmSrc, RegSrc,
    output logic [2:0] ALUControl
);

    logic [11:0] controls;
    logic       Branch, ALUOp;

    // Main Decoder

    always_comb
        case(Op)
            // DATA PROCESSING
            2'b00:
                case(Funct[4:1])

                    4'b1101 :  //MOV e LSL com IMMEDIATE                       
                        if (Funct[5])   controls = 12'b000011010000;                        
                        else            controls = 12'b000010010000; //MOV e LSL  com REGISTER

                    4'b1010 :                        
                        if (Funct[5])   controls = 12'b000001000001; //CMP com IMMEDIATE                        
                        else            controls = 12'b000000000001; //CMP com REGISTER

                    4'b1000 :                        
                        if (Funct[5])   controls = 12'b000001000001; //TST com IMMEDIATE                        
                        else            controls = 12'b000000000001; //TST com REGISTER

                    default: //ALU FUNCTIONS ADD|SUB|AND|ORR|EOR                       
                        if (Funct[5])   controls = 12'b000001010001; 
                        else            controls = 12'b000000010001;
                endcase  

            2'b01:  // MEMORY
                if (Funct[0]) 
                    if (~Funct[5]) //Imediato
                        if(Funct[2])    controls = 12'b000101110100; //LDRB
                        else            controls = 12'b000101110000; //LDR
                    else //Registrador                
                        if(Funct[2])    controls = 12'b000100110100; //LDRB                        
                        else            controls = 12'b000100110000; //LDR
                                       
                else // STR                    
                    if (~Funct[5]) controls = 12'b100101101000; //Imediato                    
                    else           controls = 12'b000100101000; //Registrador
            
                                 
            2'b10:                 controls = 12'b011001000010; // BRENCH  
            
            default:               controls = 12'bx; // Unimplemented
        endcase

        assign {RegSrc, ImmSrc, ALUSrcA, ALUSrcB, MemtoReg,
            RegW, MemW, MemB, Branch, ALUOp} = controls;

    // ALU Decoder
    always_comb
        if (ALUOp) begin                 // which DP Instr?
            case(Funct[4:1])
                4'b0100: ALUControl = 3'b000; // ADD
                4'b0010: ALUControl = 3'b001; // SUB
                4'b0000: ALUControl = 3'b010; // AND
                4'b1100: ALUControl = 3'b011; // ORR
                4'b0001: ALUControl = 3'b100; // EOR 
                4'b1000: ALUControl = 3'b010; // TST              
                4'b1010: ALUControl = 3'b001; // CMP                 
                default: ALUControl = 3'bx;  // unimplemented
            endcase
            // update flags if S bit is set
            // (C & V only updated for arith instructions)
            FlagW[1]      = Funct[0]; // FlagW[1] = S-bit
            // FlagW[0] = S-bit & (ADD | SUB)
            FlagW[0]      = Funct[0] & (ALUControl == 3'b000 | ALUControl == 3'b001);
        end else begin
            ALUControl = 3'b000; // add for non-DP instructions
            FlagW      = 2'b00; // don't update Flags
        end

    // PC Logic
    assign PCS = ((Rd == 4'b1111) & RegW) | Branch;
endmodule

module condlogic(input  logic       clk, reset,
                 input  logic [3:0] Cond,
                 input  logic [3:0] ALUFlags,
                 input  logic [1:0] FlagW,
                 input  logic       PCS, RegW, MemW,
                 output logic       PCSrc, RegWrite, MemWrite);
                 
  logic [1:0] FlagWrite;
  logic [3:0] Flags;
  logic       CondEx;

  flopenr #(2)flagreg1(clk, reset, FlagWrite[1], 
                       ALUFlags[3:2], Flags[3:2]);
  flopenr #(2)flagreg0(clk, reset, FlagWrite[0], 
                       ALUFlags[1:0], Flags[1:0]);

  // write controls are conditional
  condcheck cc(Cond, Flags, CondEx);
  assign FlagWrite = FlagW & {2{CondEx}};
  assign RegWrite  = RegW  & CondEx;
  assign MemWrite  = MemW  & CondEx;
  assign PCSrc     = PCS   & CondEx;
endmodule


module condcheck(input  logic [3:0] Cond,
                 input  logic [3:0] Flags,
                 output logic       CondEx);
  
  logic neg, zero, carry, overflow, ge;
  
  assign {neg, zero, carry, overflow} = Flags;
  assign ge = (neg == overflow);
                  
  always_comb
    case(Cond)
      4'b0000: CondEx = zero;             // EQ
      4'b0001: CondEx = ~zero;            // NE
      4'b0010: CondEx = carry;            // CS
      4'b0011: CondEx = ~carry;           // CC
      4'b0100: CondEx = neg;              // MI
      4'b0101: CondEx = ~neg;             // PL
      4'b0110: CondEx = overflow;         // VS
      4'b0111: CondEx = ~overflow;        // VC
      4'b1000: CondEx = carry & ~zero;    // HI
      4'b1001: CondEx = ~(carry & ~zero); // LS
      4'b1010: CondEx = ge;               // GE
      4'b1011: CondEx = ~ge;              // LT
      4'b1100: CondEx = ~zero & ge;       // GT
      4'b1101: CondEx = ~(~zero & ge);    // LE
      4'b1110: CondEx = 1'b1;             // Always
      default: CondEx = 1'bx;             // undefined
    endcase
endmodule

module datapath(input  logic        clk, reset,
                input  logic [1:0]  RegSrc,
                input  logic        RegWrite,
                input  logic [1:0]  ImmSrc,
                input  logic        ALUSrcA, ALUSrcB,
                input  logic [2:0]  ALUControl,
                input  logic        MemtoReg,
                input  logic        PCSrc,
                output logic [3:0]  ALUFlags,
                output logic [31:0] PC,
                input  logic [31:0] Instr,
                output logic [31:0] ALUResult, WriteData,
                input  logic [31:0] ReadData);

  logic [31:0] PCNext, PCPlus4, PCPlus8;
  logic [31:0] ExtImm, SrcA, SrcB, rd2, SrcAn, Result, WriteDataShifted,shamt;
  logic [3:0] RA1, RA2;
  

  // next PC logic
  mux2 #(32)  pcmux(PCPlus4, Result, PCSrc, PCNext); //primeiro registrador (seleciona PCplus ou result)
  flopr #(32) pcreg(clk, reset, PCNext, PC); // gerencia o reset para colocar o pc = 0 se o reset for acionado
  adder #(32) pcadd1(PC, 32'b100, PCPlus4); // soma pc+4
  adder #(32) pcadd2(PCPlus4, 32'b100, PCPlus8); //soma pc+4= pc+8

  // register file logic
  mux2 #(4)   ra1mux(Instr[19:16], 4'b1111, RegSrc[0], RA1); // seleciona as duas primeiras entradas de acordo com o RegSRC
  mux2 #(4)   ra2mux(Instr[3:0], Instr[15:12], RegSrc[1], RA2); // seleciona as duas primeiras entradas de acordo com o RegSRC
  

    regfile     rf(clk, RegWrite, RA1, RA2,
                 Instr[15:12], Result, PCPlus8, 
                 SrcA, rd2, WriteData
                 );
  
shifter       shift(Instr[11:4],rd2, WriteDataShifted); // Modulo de deslocamento para o LSL

  mux2 #(32)  resmux(ALUResult, ReadData, MemtoReg, Result);
  extend      ext(Instr[23:0], ImmSrc, ExtImm);

  // ALU logic
  mux2 #(32)  srcbmux(SrcA, 32'b0, ALUSrcA, SrcAn); // Seletor do Rn ou o valor 4'b0000 --> remove lixo da ULA
  mux2 #(32)  srcbmux2(WriteDataShifted, ExtImm, ALUSrcB, SrcB);  // Seletor do RM ou Imeadiato

  alu         alu(SrcAn, SrcB, ALUControl, ALUResult, ALUFlags); // Operador ULA
endmodule

// Register File
module regfile(input  logic        clk, 
               input  logic        we3, 
               input  logic [3:0]  ra1, ra2, ra3, 
               input  logic [31:0] wd3, r15,
               output logic [31:0] rd1, rd2, rd3
               );

  logic [31:0] rf[14:0];

  // three ported register file
  // read two ports combinationally
  // write third port on rising edge of clock
  // register 15 reads PC+8 instead

  always_ff @(posedge clk)
    if (we3) rf[ra3] <= wd3;	

  assign rd1 = (ra1 == 4'b1111) ? r15 : rf[ra1]; // Registrador Rn
  assign rd2 = (ra2 == 4'b1111) ? r15 : rf[ra2]; // Registrador Rm
  assign rd3 = (ra3 == 4'b1111) ? r15 : rf[ra3]; // Registrador Rd
endmodule

module shifter(input  logic[7:0] instruction, // Scr2[11:4]
	       input  logic [31:0] WriteData, // Rd2
               output logic [31:0] Result); // Writedatashifted

  assign shamt = {27'b0, instruction[7:3]}; // Scr2[11:7]

  always_comb
    casex (instruction[2:1]) // sh = Scr2[6:5]
      2'b00: Result = WriteData << shamt; //LSL
      default: Result = 32'bx;
    endcase
endmodule
// Modulo do imeadiato
module extend(input  logic [23:0] Instr, 
              input  logic [1:0]  ImmSrc,
              output logic [31:0] ExtImm);
 
  always_comb
    case(ImmSrc) 
               // 8-bit unsigned immediate
      2'b00:   ExtImm = {24'b0, Instr[7:0]};  
               // 12-bit unsigned immediate 
      2'b01:   ExtImm = {20'b0, Instr[11:0]}; 
               // 24-bit two's complement shifted branch 
      2'b10:   ExtImm = {{6{Instr[23]}}, Instr[23:0], 2'b00}; 
      default: ExtImm = 32'bx; // undefined
    endcase             
endmodule


module adder #(parameter WIDTH=8)
              (input  logic [WIDTH-1:0] a, b,
               output logic [WIDTH-1:0] y);
             
  assign y = a + b;
endmodule

module flopenr #(parameter WIDTH = 8)
                (input  logic             clk, reset, en,
                 input  logic [WIDTH-1:0] d, 
                 output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset)   q <= 0;
    else if (en) q <= d;
endmodule


module flopr #(parameter WIDTH = 8)
              (input  logic             clk, reset,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;
endmodule


module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

module alu(input  logic [31:0] a, b,
           input  logic [2:0]  ALUControl,
           output logic [31:0] Result,
           output logic [3:0]  ALUFlags);

  logic        neg, zero, carry, overflow;
  logic [31:0] condinvb;
  logic [32:0] sum;

  assign condinvb = ALUControl[0] ? ~b : b;
  assign sum = a + condinvb + ALUControl[0];

  always_comb
    casex (ALUControl[2:0])
      3'b00?: Result = sum;
      3'b010: Result = a & b;
      3'b011: Result = a | b;
      3'b100: Result = a ^ b;  // XOR
      default: Result = 32'bx;             // undefined
    endcase

  assign neg      = Result[31];
  assign zero     = (Result == 32'b0); //| (ALUControl[1] == 1'b1 & Result == 32'b1);
  assign carry    = (ALUControl[1] == 1'b0) & sum[32];
  assign overflow = (ALUControl[1] == 1'b0) & ~(a[31] ^ b[31] ^ ALUControl[0]) & (a[31] ^ sum[31]); 
  assign ALUFlags    = {neg, zero, carry, overflow};
endmodule

