//////////////////////////////////////////////////////////////////////
// Module Name: tb_top_multicycle
//
// Author: Miqueas Filsinger
// Date: 17-07-2024
//
// Revision History:
//////////////////////////////////////////////////////////////////////

`timescale 1ns/1ns
`include "top_multicycle.v"
module tb_top_multicycle (
    
);

    ////////////////////// CLOCK GENERATION //////////////////////
    //////////////////////////////////////////////////////////////
    parameter TCLK = 10;
    reg clk;
    initial begin
        clk = 1'b0;
        forever #(TCLK/2) clk = ~clk;
    end
    
    /////////// TOP INSTANTIATION AND VARIABLES /////////////////
    /////////////////////////////////////////////////////////////

    reg                         rst_n  = 1'b0;
    
    
    wire [31:0]                     fetch_pc_instruction;
    wire [7:0]                      fetch_pc_out;
    wire [7:0]                      fetch_pc_input;
    wire [33:0]                     decode_regB;
    wire [33:0]                     decode_regC;
    wire [33:0]                     decode_write_regA;
    wire [31:0]                     decode_instruction;
    wire [31:0]                     execute_instruction;
    wire [2:0]                      execute_alu_ctrl;
    wire [33:0]                     execute_alu_regA;
    wire [33:0]                     execute_alu_regB;
    wire [33:0]                     execute_alu_out;
    wire [33:0]                     mem_address;
    wire [31:0]                     mem_instruction;
    wire [31:0]                     mem_output_data;
    wire [31:0]                     mem_input_data;
  
    top_multicycle #(
        .PC_FILE("C:/Users/mique/OneDrive/Universidad/10mo Cuatri/PPS - Allegro/Trabajo Practico/RISC/rtl/multicycle/pc_mem_target_multi.hex")
    )
    u_top_multi (
        .clk                     (clk),
        .rst_n                   (rst_n),

        //fetch
        .debug_pc_instruction   (fetch_pc_instruction),  //output  wire [31:0]
        .debug_pc_out           (fetch_pc_out),  //output  wire [7:0]
        .debug_pc_input         (fetch_pc_input),  //output  wire [7:0]
        //decode
        .debug_dec_instruction  (decode_instruction), //output  wire [31:0]
        .debug_dec_regB         (decode_regB),  //output  wire [33:0]
        .debug_dec_regC         (decode_regC),  //output  wire [33:0]
        .debug_dec_write_regA   (decode_write_regA),  //output  wire [33:0]
        //execute 
        .debug_exec_instruction (execute_instruction),  //output  wire [31:0]
        // .debug_exec_alu_ctrl    (execute_alu_ctrl),  //output  wire [2:0]
        .debug_exec_alu_regA    (execute_alu_regA),  //output  wire [33:0]
        .debug_exec_alu_regB    (execute_alu_regB),  //output  wire [33:0]
        .debug_exec_alu_out     (execute_alu_out),  //output  wire [33:0]
        //data mem
        .debug_mem_instruction  (mem_instruction), //output  wire [31:0]
        .debug_mem_address      (mem_address),  //output  wire [7:0]
        .debug_mem_output_data  (mem_output_data),  //output  wire [31:0]
        .debug_mem_input_data    (mem_input_data)  //output  wire [31:0]
    );

    initial begin
        #4 rst_n = 1'b1;

        //Filling the whole pipeline
        @(posedge clk);
        @(posedge clk);
        @(posedge clk);
        @(posedge clk);
        #300
        $finish;
    end

endmodule // 