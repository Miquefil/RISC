//////////////////////////////////////////////////////////////////////
// Module Name: alu_tb
//              testbench for alu
//
// Author: Miqueas Filsinger
// Date: 16-06-2024
//
//  
//////////////////////////////////////////////////////////////////////

`timescale 1ns/1ns
`include "alu.v"
module alu_tb (
    // Ports here
);

    reg clk;
    localparam TCLK = 10;
    initial begin
        clk=1'b0;
        forever #(TCLK/2) clk = ~clk;
    end
    

    reg[2:0]        alu_ctrl;
    reg signed [33:0]       alu_regA;
    reg signed [33:0]       alu_regB;
    reg signed [33:0]       alu_regO;

    alu #(
        .NB_REGISTERS(34),
        .DATA_WIDTH(32)
    )
    u_alu (
        .i_alu_ctrl  (alu_ctrl), //input   wire [2:0]                  
        .i_data_A    (alu_regA), //input   wire [NB_REGISTERS-1:0]     
        .i_data_B    (alu_regB), //input   wire [NB_REGISTERS-1:0]     
        .o_data      (alu_regO)  //output  wire [NB_REGISTERS-1:0]     
    );

    //ALU operations
    parameter ADD   = 3'd0;
    parameter SUB   = 3'd1;
    parameter ADDC  = 3'd2;
    parameter SUBC  = 3'd3;
    parameter EXCEPTION = 3'd4;

    initial begin
        $dumpfile("dump.vcd"); 
        $dumpvars;  
        $display("-----------starting--------------");
            alu_ctrl = EXCEPTION;
            alu_regA = 34'd0;
            alu_regB = 34'd0;
            #20;

            alu_ctrl = ADD;
            alu_regA = 34'd-5;
            alu_regB = 34'd1;
            #20;

            


        $display("-----------finishing--------------");
        $finish;
    end

endmodule //