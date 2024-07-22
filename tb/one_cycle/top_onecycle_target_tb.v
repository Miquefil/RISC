//////////////////////////////////////////////////////////////////////
// Module Name: top_onecycle_target_tb
//              -target testbench for top 
//
// Author: Miqueas Filsinger
// Date: 26-06-2024
//
// Notes:
// - register memory is pre-loaded with progressive natural numbers. That is,
//  reg[0] = 0, reg[1] = 1, reg[2] = 2, ....
// - reg[30] = reg[31] = FFFFFFFFF
// - the instruction memory is also preloaded with all operations in the following
// order: logic (T=10), arithmetic (T=11), movement (T=00), flow ctrl (T=01)
//////////////////////////////////////////////////////////////////////

`timescale 1ns/1ns
`include "top_onecycle.v"
module top_onecycle_target_tb (
    // Ports here
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

    reg                         rst_n  = 1'b1;
    wire [33:0]                 alu_out;
    wire [31:0]                 pc_instruction;
    wire [7:0]                  pc;
    wire [7:0]                  pc_in;
    wire [31:0]                 mem_output;
    wire [33:0]                 write_regA;
    wire [33:0]                 regB;
    wire [33:0]                 regC;
    wire [31:0]                 mem_data_output;
    wire [31:0]                 mem_data_input;
  
    top_onecycle u_top (
        .clk                     (clk),
        .rst_n                   (rst_n),
        //Debug outputs
        .debug_regB              (regB),
        .debug_regC              (regC),
        .debug_write_regA        (write_regA),
        .debug_alu_out           (alu_out),
        .debug_pc_instruction    (pc_instruction),
        .debug_pc_out            (pc),    //output  wire [7:0]
        .debug_pc_input          (pc_in),    //output  wire [7:0]
        .debug_mem_output_data   (mem_data_output),
        .debug_mem_input_data    (mem_data_input)
    );
  
  	wire [11:0] instruction_Imm;
  	wire [4:0]	instruction_RC_addr;
  	assign instruction_Imm = pc_instruction[26:15];
  	assign instruction_RC_addr = pc_instruction[14:10];
    
    //risc 
    wire [1:0]  risc_instruction_T;
    wire [2:0]  risc_instruction_OPC;
    assign risc_instruction_T       = pc_instruction[31:30];
    assign risc_instruction_OPC     = pc_instruction[29:27];


    reg [31:0]  imm;
    reg [31:0]  imm_rc;
    always @(*) begin
        imm         = {1'b0, {19{1'b0}}, {instruction_Imm}};
        if(instruction_Imm[11]) begin
            imm     = {1'b1, {19{1'b0}}, {instruction_Imm}};
        end

      	imm_rc  	= {1'b0, {14{1'b0}}, {instruction_Imm}, {instruction_RC_addr}};
      	if(instruction_Imm[11]) begin
            imm_rc  = {1'b1, {14{1'b0}}, {instruction_Imm}, {instruction_RC_addr}};
        end
    end
    

    //////////////////////     MEMORY DUPLICATE    //////////////////////
    /////////////////////////////////////////////////////////////////////
  	///	we copy the same memory used in RISC to use it for comparison in 
  	///	movement operations
    parameter MEM_INIT_FILE = "mem.hex";
  	reg [0:7]     tb_mem [0:1023];
    initial begin
        if (MEM_INIT_FILE != "") begin
          $readmemh(MEM_INIT_FILE, tb_mem, 0, 1023);
        end
    end 
  	reg  [7:0] 	tb_mem_pointer = 8'd0;
  	wire [31:0]	tb_mem_output;
  	assign tb_mem_output = {{tb_mem[tb_mem_pointer*4]}, {tb_mem[tb_mem_pointer*4+8'd1]}, {tb_mem[tb_mem_pointer*4+8'd2]}, {tb_mem[tb_mem_pointer*4+8'd3]}};

    ///////////     PROGRAM COUNTER MEMORY DUPLICATE    //////////////
    //////////////////////////////////////////////////////////////////
    parameter PC_MEM_FILE = "pc_mem.hex";
  	reg  [7:0]   tb_pc_mem [0:1023];
    reg  [7:0]   tb_pc_mem_counter = 8'd0; 
    

    initial begin
        if (PC_MEM_FILE != "") begin
          	$readmemh(PC_MEM_FILE, tb_pc_mem, 0, 1023);
        end
    end 

    //tb_instruction is the instruction, from a duplicate memory program counter, in the
    //same address as the RISC program counter memory
    wire [31:0]  tb_instruction;
    assign tb_instruction = {tb_pc_mem[pc], tb_pc_mem[pc+8'd1], tb_pc_mem[pc+8'd2], tb_pc_mem[pc+8'd3]};

    //tb_T and tb_OPC from testbench's PC memory 
    wire [1:0]  tb_T;
    wire [2:0]  tb_OPC;
    assign  tb_T       = tb_instruction[31:30];
    assign  tb_OPC     = tb_instruction[29:27];
    

    //////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////
    ///////////                 TESTBENCH                 ////////////
    /////////////////////////////////////////////////////////////////
    integer n_test = 0;
  
	  task esperar(
      	input  [33:0]    actual,
        input  [33:0]    expected
    );
      n_test = n_test + 1; 
      if(actual == expected)begin
        	$display("TEST N %d PASSED", n_test);
          end
          else begin
            $display("TEST N %d FAILED: Expected %h and obtained %h", n_test, expected, actual);
		end
    endtask

    /////////////////////// MAIN INITIAL TEST 	/////////////////////////////////
  	////////////////////////////////////////////////////////////////////////////
    integer j;

    initial begin
        $dumpfile("dump.vcd"); 
        $dumpvars;  
        $display("-----------starting--------------");
        
      
      	rst_n = 1'b1;
      					#3 esperar(write_regA, 34'h000000001);	//OR operation
      	@(posedge clk)  #3 esperar(write_regA, 34'h0fffffffd);	//INV
      	@(posedge clk)  #3 esperar(write_regA, 34'h000000004);	//AND
      	@(posedge clk)  #3 esperar(write_regA, 34'h100000004);	//ADDC
      	@(posedge clk)  #3 esperar(write_regA, 34'h10000001c);	//SUBC
      	@(posedge clk)  #3 esperar(write_regA, 34'h000000035);	//ADD
      	@(posedge clk)  #3 esperar(write_regA, 34'h000000001);	//SUB
      	@(posedge clk)  #3 esperar(write_regA, 34'h03d155142);	//LOAD
      	@(posedge clk)  #3 esperar(mem_data_input, 34'h00000014);//STORE
      	@(posedge clk)  #3 esperar(write_regA, 34'd63);			//LOADI
      	@(posedge clk)  #3 esperar(mem_data_input, 34'd127);	//LOADI
      	@(posedge clk)  #3 esperar(write_regA, 34'd15);			//MOV
      	
      	//JUMP instruction	
     	@(posedge clk)  #3 esperar(pc_instruction, 34'b01000000000000000000000001000000);			
      	esperar(pc, 34'd12);			//salta 2 lugares
      	esperar(pc_in, 34'd14);		
      
      	//BZ instruction
      	@(posedge clk)  #3 esperar(pc_instruction, 34'b01001000000000010000000000100000);
      	esperar(pc, 34'd14);			
      	esperar(pc_in, 34'd17);	//salta  3 lugares (12 posiciones de memoria)
      
      	//BNZ
      	@(posedge clk)  #3 esperar(pc_instruction, 34'b01010000000000010000010001000000);
      	esperar(pc, 34'd17);			
      	esperar(pc_in, 34'd21);	//salta  4 lugares (16 posiciones de memoria)
      
      	//BC
      	@(posedge clk)  #3 esperar(pc_instruction, 34'b01011000000000001111100000100000);
      	esperar(pc, 34'd21);			
      	esperar(pc_in, 34'd23);	//salta  2 lugares (8 posiciones de memoria)
      
      	//BV
      	@(posedge clk)  #3 esperar(pc_instruction, 34'b01100000000000000111100000100000);
      	esperar(pc, 34'd23);			
      	esperar(pc_in, 34'd24);	//salta  1 lugar (4 posiciones de memoria)
      
      	//JAL
      	@(posedge clk)  #3 esperar(pc_instruction, 34'b01101000000000000000110000011111);
      	esperar(pc, 34'd24);			
      	esperar(pc_in, 34'd27);	//salta  3 lugares (12 posiciones de memoria)
      
      	//JRAL
      @(posedge clk)  #3 esperar(pc_instruction, 34'b01110000000000000000100000011111);
      	esperar(pc, 34'd27);			
      	esperar(pc_in, 34'd29);	//salta  2 lugares (8 posiciones de memoria)
      
      	//RET
      	@(posedge clk)  #3 esperar(pc, 34'd29);
      	esperar(pc_in, 34'd18);
      
      
      
        #200;  
      
      	$display("-----------finishing--------------");
        $finish;
    end
endmodule //