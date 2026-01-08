`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Joseph Russo
// 
// Create Date: 05/04/2025
// Module Name: Project_3_TB
//////////////////////////////////////////////////////////////////////////////////

module TB_CPU();
    reg CLK;

    // IF/ID & PC
    wire [31:0] pc;
    wire [31:0] dinstOut;

    // ID/EX control
    wire        ewreg;
    wire        em2reg;
    wire        ewmem;

    // ID/EX ALU control
    wire [3:0]  ealuc;
    wire        ealuimm;
    wire [4:0]  edestReg;

    // ID/EX data
    wire [31:0] eqa;
    wire [31:0] eqb;
    wire [31:0] eimm32;

    // EX/MEM control
    wire        mwreg;
    wire        mm2reg;
    wire        mwmem;

    // EX/MEM data
    wire [4:0]  mdestReg;
    wire [31:0] mr;
    wire [31:0] mqb;

    // MEM/WB control
    wire        wwreg;
    wire        wm2reg;

    // MEM/WB data
    wire [4:0]  wdestReg;
    wire [31:0] wr;
    wire [31:0] wdo;

    // WB result
    //wire [31:0] wbData;

    // Forwarding selects
    wire [1:0] fwdA;
    wire [1:0] fwdB;

    // Raw regfile outputs
    wire [31:0] qa;
    wire [31:0] qb;

    // Forwarded inputs into ALU
    wire [31:0] fwvA;
    wire [31:0] fwvB;

    // Instantiate your updated datapath
    
    CPU_Project Datapath(
        .CLK        (CLK),
        .PC         (pc),
        .instOut    (dinstOut),

        .ewreg      (ewreg),
        .em2reg     (em2reg),
        .ewmem      (ewmem),

        .ealuc      (ealuc),
        .ealuimm    (ealuimm),
        .edestReg   (edestReg),

        .eqa        (eqa),
        .eqb        (eqb),
        .eimm32     (eimm32),

        .mwreg      (mwreg),
        .mm2reg     (mm2reg),
        .mwmem      (mwmem),

        .mdestReg   (mdestReg),
        .mr         (mr),
        .mqb        (mqb),

        .wwreg      (wwreg),
        .wm2reg     (wm2reg),

        .wdestReg   (wdestReg),
        .wr         (wr),
        .wdo        (wdo),

        //.wbData     (wbData),
        
        .qa(qa),
        .qb(qb),
        
        .fwdA(fwdA),
        .fwdB(fwdB),
        
        .fwvA(fwvA), 
        .fwvB(fwvB)
    );

    // Clock generator: 1 ns period
     initial begin
        CLK = 0;
        forever #5 CLK = ~CLK; // toggle clock every 1 ns
    end
endmodule