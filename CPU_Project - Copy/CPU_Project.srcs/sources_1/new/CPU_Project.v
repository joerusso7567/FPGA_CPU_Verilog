`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Joseph Russo
// 
// Create Date: 04/20/2025 03:12:26 PM
// Design Name: 
// Module Name: Lab5_Datapath
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module CPU_Project(
    input  CLK,                  // Clock signal

    output [31:0] PC,        // Program Counter output
    output [31:0] instOut,       // Instruction fetched
    //output [31:0] aluResult,      // Final ALU output is this used? added for lab4?
    output ewreg, // why 32bit?
    output em2reg, // why 32bit?
    output ewmem, // why 32bit?
    output [3:0] ealuc,
    output ealuimm,
    output [4:0] edestReg,
    output [31:0] eqa,
    output [31:0] eqb,
    output [31:0] eimm32,
    /// added for lab 4
    output wwreg, wm2reg, mwreg, mm2reg, mwmem, //these arent outputs? 
    output [31:0] wdo, wr,  mr, mqb,  //not outputs?       
    output [4:0] wdestReg, mdestReg, //not an output? 
    ///added for lab 5
    //output [31:0] wbData,
    output [31:0] qa, qb,
    output [1:0] fwdA, fwdB,
    output [31:0] fwvA, fwvB
    
);

    // Internal Wires for Interconnection
    wire [31:0] Next_Pc, instruction;
    wire [31:0] imm32; //qa, qb,
    wire [4:0] destReg;
    //wire [5:0] op, func;
    wire [3:0] ALUC;
    wire Write_reg, Mem2reg, Write_mem, ALUimm, Register_rt;
    //wire CLK2; //??
    
////////////////////wires added for lab 4:
    wire [31:0] alu_out, b, mdo;
    //wire mwreg, mm2reg, mwmem;
    //wire [4:0] mdestReg;
    //wire [31:0] mr, mqb;
////////////////////wires added for lab 5:
    //wire wwreg;
    //wire wm2reg;
    //wire [4:0] wdestReg;
    //wire [31:0] wr, wdo;
    
    //wire [31:0] wbData;
    
////////Project Wires added here:////////////////////////////
//wire [4:0]  rs, rt;
// Forwarding controls
//wire [1:0] fwdA, fwdB;

// Forwarding MUX outputs (to EX inputs)
//wire [31:0] fwvA, fwvB;
wire [31:0] efwva, efwvb;
//////////////////////////////////////////////////////////////// 

//wire [31:0] id_eqa, id_eqb;
//wire [31:0] id_imm32; 
wire [31:0] wbData;
    
    PC2 program_counter (
        .Next_Pc(Next_Pc),
        .CLK(CLK),
        .PC(PC)
    );

    PC_Adder PCAdder (
        .PC(PC),
        .Next_Pc(Next_Pc)
    );

    Instruction_memory InstrMem (
        .CLK(CLK),
        .PC(PC),
        .InstOut(instruction)
    );

    IFID_Pipeline_Register IF_ID (
        .CLK(CLK),
        .InstOut(instruction),
        //.PC_Plus4(PCPlus4),
        .dinstOut(instOut) //(dinstOut)?
        //.doutPC(PC_Out)
    );

    // ID Stage Components
    Register_File reg_file (
        .rs(instOut[25:21]),
        .rt(instOut[20:16]),
        .qa(qa),
        .qb(qb),
        ////addeed for lab5: 
        .wdestReg(wdestReg),
        .wbData(wbData),
        .wwreg(wwreg),
        .CLK(~CLK)
    );

    Sign_Extender sign_ext (
        .imm(instOut[15:0]),
        .imm32(imm32)
    );

    Control_Unit control_unit (
        .op(instOut[31:26]),
        .func(instOut[5:0]),
        
        // forwarding inputs
        .rs(instOut[25:21]),
        .rt(instOut[20:16]),
        
        .ern        (edestReg),//(ex_mdest),
        .ewreg  (ewreg), // ex_mwreg --> mwreg --> ewreg
        .mrn        (mdestReg), // wdestReg --> wdestReg --> mdestReg
        .mwreg  (mwreg), // mem_wwreg --> wwreg --> mwreg
        
        .mm2reg(mm2reg),
        .em2reg(em2reg),
        
        .Register_rt(Register_rt),
        .Write_reg(Write_reg),
        .Mem2reg(Mem2reg),
        .Write_mem(Write_mem),
        .ALUC(ALUC),
        .ALUimm(ALUimm),
        
        // new forwarding selects
        .fwdA (fwdA),
        .fwdB (fwdB)
    );

    Reg_Mux reg_mux (
        .rt(instOut[20:16]),
        .rd(instOut[15:11]),
        .Register_rt(Register_rt),
        .destReg(destReg)
    );

    // ID/EX Pipeline Register
    IDEXE_Pipeline_Register ID_EX (
        .CLK(CLK),
        .Write_reg(Write_reg),
        .Mem2reg(Mem2reg),
        .Write_mem(Write_mem),
        .ALUC(ALUC), // changed from ALUC to eALUC
        .ALUimm(ALUimm), // was ALUimm
        .destReg(destReg),
        
        //.rs         (instOut[25:21]),   // NEW
        //.rt         (instOut[20:16]),   // NEW
        
        .qa(qa),
        .qb(qb),
        .imm32(imm32),
        
        .fwvA(fwvA),
        .fwvB(fwvB),
        
        .eWrite_reg(ewreg),
        .eMem2reg(em2reg),
        .eWrite_mem(ewmem),
        .eALUC(ealuc),
        .eALUimm(ealuimm),
        .edestReg(edestReg),
           
        .eqa(eqa), //according ot the project pdf these arent even outputs of IDEXE
        .eqb(eqb), // but they are in the waveform posted on canvas
        
        .eimm32(eimm32),
        
        .efwva(efwva),
        .efwvb(efwvb)
    );
////////////////////////////lab 4 instantiation starts here
   ALUmux myALUmux (
        .eqb(efwvb), //eqb --> fwvB --> efwvb
        .eimm32(eimm32),
        .ealuimm(ealuimm),
        .b(b)
);

    // 4-way forward mux for A
    ForwardMux4 fwdA_mux (
        
        .in0  (qa),    // eqa --> alu_out --> qa  
        .in1  (alu_out), //alu_out
        .in2  (mr),  // --> mqb?  
        .in3  (mdo),  // wdo --> rs --> mdo
        .sel  (fwdA),
        .out  (fwvA)
);

// 4-way forward mux for B
    ForwardMux4 fwdB_mux (
        
        .in0  (qb),  // eqb --> alu_out -> qb   
        .in1  (alu_out), // alu_out --> mr // alut out causes the sim to not work
        .in2  (mr),  // --> mqb?, mr -> wr 100% should be mr
        .in3  (mdo), // eimm32 --> rs --> mdo --> eimm32 100% mdo
        .sel  (fwdB),
        .out  (fwvB)
);

    ALU myALU(
        .eqa(efwva), // fwvA -->
        .b(b),
        .ealuc(ealuc),
        .result(alu_out)
);
    EXEMEM_Pipeline my_EXEMEM_Pipeline(
        .ewreg(ewreg),
        .em2reg(em2reg),
        .ewmem(ewmem),
        .edestReg(edestReg),
        .r(alu_out),
        .efwvb(efwvb), //was eqb
        .CLK(CLK),
        .mwreg(mwreg),
        .mm2reg(mm2reg),
        .mwmem(mwmem),
        .mdestReg(mdestReg),
        .mr(mr),
        .mqb(mqb)
        
);

    Data_Memory myDataMemory(
        .mr(mr),        // Address from EX/MEM (ALU result)
        .mqb(mqb),      // Data input for write (from EX/MEM)
        .mwmem(mwmem),  // Memory write enable from EX/MEM
        .CLK(CLK),
        .mdo(mdo)       // Memory data output
);

    MEMEWB_Pipeline_Register my_MEMEWB_Pipeline_Register(
        .mwreg(mwreg),     // from EX/MEM pipeline
        .mm2reg(mm2reg),   // from EX/MEM pipeline
        .mdestReg(mdestReg), // from EX/MEM pipeline
        .mr(mr),           // latched ALU result from EX/MEM
        .mdo(mdo),         // data output from Data Memory
        .CLK(CLK),
        .wwreg(wwreg),
        .wm2reg(wm2reg),
        .wdestReg(wdestReg),
        .wr(wr), 
        .wdo(wdo)
);

////////////////////////////lab  5instantiation starts here
    WbMux wb_mux(
        .wr(wr),
        .wdo(wdo),
        .wm2reg(wm2reg),
        .wbData(wbData)
    );
    
/////ends here
endmodule

module PC2(
    input wire [31:0] Next_Pc,
    input wire CLK,
    output reg [31:0] PC
    );
    initial begin
        PC = 32'd100;
    end
    always @(posedge CLK) begin
        PC <= Next_Pc;
    end
endmodule

module PC_Adder(
    input wire [31:0] PC,
    output reg [31:0] Next_Pc
    );
    always@(*) begin
        Next_Pc = PC + 32'd4;
    end
endmodule

module Instruction_memory(
    input wire CLK, // The slides and video didn't say this needs a clk as input?
    input [31:0] PC,
    output reg [31:0] InstOut
    );
    // 64-word (256-byte) instruction memory
    reg [31:0] memory [0:63];
    
    initial begin
        // commentted out for project memory[25] = {6'd35, 5'd1, 5'd2, 16'd0}; // lw $2, 0($1)
        // commentted out for project memory[26] = {6'd35, 5'd1, 5'd3, 16'd4}; // lw $3, 4($1)
        //the following was added for lab 4
        // commentted out for project memory[27] = {6'd35, 5'd1, 5'd4, 16'd8};   // lw $4, 8($1)
        // commentted out for project memory[28] = {6'd35, 5'd1, 5'd5, 16'd12}; // lw $5, 12($1)
        //added for lab 5:
        // commentted out for project memory[29] = {6'd0, 5'd2, 5'd10, 5'd6, 5'd0, 6'd32};
        
        //added for final project:
        memory[25] = {6'd0, 5'd1, 5'd2, 5'd3, 5'd0, 6'd32}; // add  $3, $1, $2
        memory[26] = {6'd0, 5'd9, 5'd3, 5'd4, 5'd0, 6'd34}; // sub  $4, $9, $3
        memory[27] = {6'd0, 5'd3, 5'd9, 5'd5, 5'd0, 6'd37}; // or   $5, $3, $9
        memory[28] = {6'd0, 5'd3, 5'd9, 5'd6, 5'd0, 6'd38}; // xor  $6, $3, $9
        memory[29] = {6'd0, 5'd3, 5'd9, 5'd7, 5'd0, 6'd36}; // and  $7, $3, $9

    end
    
    always@(*) begin
        InstOut = memory[PC[7:2]];
    end
endmodule

module IFID_Pipeline_Register(
    input wire [31:0] InstOut,
    input wire CLK,
    output reg [31:0] dinstOut
    );
    always@(posedge CLK) begin
        dinstOut <= InstOut;
    end 
endmodule

module Register_File(
    input [4:0] rs,
    input [4:0] rt,
    output reg [31:0] qa,
    output reg [31:0] qb,
    ////added for lab 5:
    input [4:0] wdestReg, // wn
    input [31:0] wbData, //d
    input wwreg,
    input CLK
    );
    
    reg [31:0] registers [0:31]; //originally: reg [31:0] registers [31:0]
    
    
    //integer i;
    initial begin
    ///added for project
        registers[0]  = 32'h00000000;
        registers[1]  = 32'hA00000AA;
        registers[2]  = 32'h10000011;
        registers[3]  = 32'h20000022;
        registers[4]  = 32'h30000033;
        registers[5]  = 32'h40000044;
        registers[6]  = 32'h50000055;
        registers[7]  = 32'h60000066;
        registers[8]  = 32'h70000077;
        registers[9]  = 32'h80000088;
        registers[10] = 32'h90000099;
        // zero the rest
        //for (i = 11; i < 32; i = i + 1)
            //registers[i] = 32'h00000000;
    end
    always@(*) begin //negedge CLK should this be async? - yes
        qa <= registers[rs];
        qb <= registers[rt];
    end
    ///added for lab 5:
    always@(posedge CLK) begin
        if (wwreg) begin
            registers[wdestReg] <= wbData;
        end
    end

endmodule

module Sign_Extender(
    input [15:0] imm,
    output reg [31:0] imm32
    );
    
    always@(*) begin
        if (imm[15] ==1'b1)
            imm32 = (imm | 32'hFFFF0000);
        else
            imm32 = imm; // = (imm | 32'h00000000)
    end
endmodule

module Control_Unit(
    input [5:0] op,            // Opcode field from instruction
    input [5:0] func,          // Function field for R-type instructions
    
    // forwarded-hazard inputs
    input  [4:0]  rs,          // 
    input  [4:0]  rt,          // 
    input  [4:0]  ern,         //  
    input         ewreg,   // 
    input  [4:0]  mrn,         // 
    input         mwreg,   //
    
    input mm2reg, // what to do with these
    input em2reg, // this too, theyre on the pdf as inputs but i never had a use for them here
    
    output reg Register_rt,    // Destination register selection (0 = rd, 1 = rt)
    output reg Write_reg,      // Register write enable signal
    output reg Mem2reg,        // Selects data source for register writeback (Memory or ALU)
    output reg Write_mem,      // Memory write enable signal
    output reg [3:0] ALUC,     // ALU control signal
    output reg ALUimm,
    
    // NEW forwarding selects for two 4-way muxes
    output reg [1:0]  fwdA,      // selects A-input
    output reg [1:0]  fwdB       // selects B-input
    
    );
    
    always @(*) begin
        //Register_rt = 0;
        //Write_reg = 0;
        //Mem2reg = 0;
        //Write_mem = 0;
        //ALUimm = 0;
        //ALUC = 4'd0; // removed so it would agree with the refrence wave form and stay at 2
        
        case(op)
            //-----------------------------------
            6'b000000: begin  // R-type
                Write_reg   = 1;
                Register_rt = 0;  // dest = rd
                Mem2reg     = 0;
                Write_mem   = 0;
                ALUimm      = 0;
                case(func)
                    6'b100000: ALUC = 4'd2; // ADD
                    6'b100010: ALUC = 4'd6; // SUB
                    6'b100101: ALUC = 4'd1; // OR changed from 3 to 1
                    6'b100110: ALUC = 4'd3; // XOR changed from 4 to 3
                    6'b100100: ALUC = 4'd0; // AND
                    default begin
                    end
                endcase
            end

            //-----------------------------------
            6'b100011: begin  // LW
                Write_reg   = 1;
                Register_rt = 1;  // dest = rt
                Mem2reg     = 1;
                Write_mem   = 0;
                ALUimm      = 1;
                ALUC        = 4'b0010; // ADD for address
            end

            //-----------------------------------
            6'b101011: begin  // SW
                Write_reg   = 0;
                Register_rt = 1;  // don't care
                Mem2reg     = 0;
                Write_mem   = 1;
                ALUimm      = 1;
                ALUC        = 4'b0010; // ADD for address
            end

            //-----------------------------------
            default: begin
            end
        endcase
        
        // ---- forwarding logic ---- works currently
        // ---- forwarding logic for A ----
        if (ewreg && ern != 5'd0 && ern == rs) begin
            fwdA = 2'b01;                // EX --> EX hazard
        end else if (mwreg && mrn != 5'd0 && mrn == rs) begin
            fwdA = 2'b10;                // MEM --> EX hazard
        end else begin
            fwdA = 2'b00;                // no forwarding
        end

        // ---- forwarding logic for B ----
        if  (ALUimm) begin
            fwdB = 2'b11;                // immediate path has top priority
        end else if (ewreg && ern != 5'd0 && ern == rt) begin
            fwdB = 2'b01;                // EX --> EX hazard
        end else if (mwreg && mrn != 5'd0 && mrn == rt) begin
            fwdB = 2'b10;                // MEM --> EX hazard
        end else begin
            fwdB = 2'b00;                // no forwarding
        end
        
    end
endmodule

module Reg_Mux(
    input [4:0] rt,
    input [4:0] rd,
    input Register_rt,
    output reg [4:0] destReg
    );
    
    always@(*) begin
        if (Register_rt)
            destReg = rt; //chose rt when Register_rt = 1
        else
            destReg = rd;
    end
endmodule

module IDEXE_Pipeline_Register(
    input Write_reg,
    input Mem2reg,
    input Write_mem,
    input [3:0] ALUC,
    input ALUimm,
    input [4:0] destReg,
    
    // raw instruction fields for forwarding
    //input       [4:0]  rs,        // NEW: source register #1
    //input       [4:0]  rt,        // NEW: source register #2
    
    input [31:0] qa,
    input [31:0] qb,
    input [31:0] imm32,
    input CLK,
    
    input [31:0] fwvA, fwvB,
    
    output reg eWrite_reg,
    output reg eMem2reg,
    output reg eWrite_mem,
    output reg [3:0] eALUC,
    output reg eALUimm,
    output reg [4:0] edestReg,
    
    //output reg [4:0]   ers,       // NEW: latched rs
    //output reg [4:0]   ert,       // NEW: latched rt
    
    output reg [31:0] eqa, // according ot the project pdf these arent even outputs of IDEXE but oh well
    output reg [31:0] eqb,
    output reg [31:0] eimm32,
    
    output reg [31:0] efwva,
    output reg [31:0] efwvb
    );
    
    always @(posedge CLK) begin
        eWrite_reg <= Write_reg;
        eMem2reg   <= Mem2reg;
        eWrite_mem <= Write_mem;
        eALUC      <= ALUC;
        eALUimm    <= ALUimm;
        edestReg   <= destReg;

        eqa       <= fwvA; // according to the project pdf these arent even outputs of IDEXE but oh well
        eqb       <= fwvB;
        eimm32    <= imm32;
        
        efwva <= fwvA;
        efwvb <= fwvB;
    end
endmodule

//lab 4 modules added here

module ALUmux(
    input [31:0] eqb, 
    input [31:0] eimm32,
    input ealuimm, 
    output reg [31:0] b
);
    always@(*) begin
        if (ealuimm)
            b = eimm32;
        else
            b = eqb;
    end

endmodule
    
module ALU(
    input  [31:0] eqa,
    input  [31:0] b,
    input  [3:0]  ealuc,
    output reg [31:0] result
);
    always @(*) begin
        case (ealuc)
            4'd0: result = eqa & b;         // AND
            4'd6: result = eqa - b;         // SUB
            4'd2: result = eqa + b;         // ADD
            4'd1: result = eqa | b;         // OR
            4'd3: result = eqa ^ b;         // XOR
            //4'b0101: result = ~(eqa | b);      // NOR
            //4'b0110: result = eqa << b[4:0];   // SLL (logical left shift)
            //4'b0111: result = eqa >> b[4:0];   // SRL (logical right shift)
            //4'b1000: result = $signed(eqa) >>> b[4:0]; // SRA (arithmetic right shift)
            //4'b1001: result = (eqa < b) ? 32'b1 : 32'b0; // SLT
        endcase
    end
endmodule

module EXEMEM_Pipeline(
    input ewreg,
    input em2reg,
    input ewmem,
    
    input [4:0] edestReg, // ern
    input [31:0] r, //out of alu
    input [31:0] efwvb, //efwvb
    input CLK,
    
    output reg mwreg,
    output reg mm2reg,
    output reg mwmem,
    output reg [4:0] mdestReg,
    output reg [31:0] mr,
    output reg [31:0] mqb
);

    always @(posedge CLK) begin
        mwreg <= ewreg;
        mm2reg <= em2reg;
        mwmem <= ewmem;
        mdestReg <= edestReg;
        mr <= r;
        mqb <= efwvb;
    end

endmodule

module Data_Memory(
    input [31:0] mr,
    input [31:0] mqb,
    input mwmem,
    input CLK,
    
    output reg [31:0] mdo
);
    reg [31:0] memory [0:63];
    initial begin
        memory[0] = 32'hA00000AA;
        memory[1] = 32'h10000011;
        memory[2] = 32'h20000022;
        memory[3] = 32'h30000033;
        memory[4] = 32'h40000044;
        memory[5] = 32'h50000055;
        memory[6] = 32'h60000066;
        memory[7] = 32'h70000077;
        memory[8] = 32'h80000088;
        memory[9] = 32'h90000099;
    end
    
    always @(*) begin
        mdo = memory[mr[7:2]];
    end
    
    always @(negedge CLK) begin
        if (mwmem) begin
            memory[mr[7:2]] <= mqb;
        end
    end
    
endmodule

module MEMEWB_Pipeline_Register(
    input mwreg, // register write enable
    input mm2reg, // memory to register control
    input [4:0] mdestReg, // distination register
    input [31:0] mr, 
    input [31:0] mdo, // data read out from the memory
    input CLK,
    
    output reg wwreg,
    output reg wm2reg,
    output reg [4:0] wdestReg,
    output reg [31:0] wr,
    output reg [31:0] wdo
);
    
    always @(posedge CLK) begin
        wwreg <= mwreg;
        wm2reg <= mm2reg;
        wdestReg <= mdestReg;
        wr <= mr;
        wdo <= mdo; 
    end
    
endmodule

//lab 5 modules added here
module WbMux(
    input [31:0] wr,
    input [31:0] wdo,
    input wm2reg,
    
    output reg [31:0] wbData 
);
    always@(*) begin
        if (wm2reg == 1'b1)
            wbData = wdo;
        else
            wbData = wr;
    end
endmodule
//////////////////////////////////////////////////////////////////////////////////////
//Project new modules added here//////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
module ForwardMux4(
 
    input  [31:0] in0, 
    input  [31:0] in1,
    input  [31:0] in2,    
    input  [31:0] in3,    
    input  [1:0]  sel,    
    output reg [31:0] out
);
    always @(*) begin
        case (sel)
            2'b00: out = in0;
            2'b01: out = in1;
            2'b10: out = in2;
            2'b11: out = in3;
        endcase
    end
endmodule

