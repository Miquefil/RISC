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
    
    wire [33:0]                     decode_regB;
    wire [33:0]                     decode_regC;
    wire [33:0]                     decode_write_regA;
    wire [31:0]                     fetch_pc_instruction;
    wire [7:0]                      fetch_pc_out;
    wire [7:0]                      fetch_pc_input;
    wire [7:0]                      mem_address;
    wire [31:0]                     mem_output_data;
    wire [31:0]                     mem_input_data;
  
    top_multicycle u_top_multi (
        .clk                     (clk),
        .rst_n                   (rst_n),
        //fetch
        .debug_pc_instruction   (fetch_pc_instruction),     //output  wire [31:0]
        .debug_pc_out           (fetch_pc_out),     //output  wire [7:0]
        .debug_pc_input         (fetch_pc_input),     //output  wire [7:0]
        //Debug outputs
      .debug_dec_instruction  (),
        .debug_dec_regB             (decode_regB), //output  wire [33:0]
        .debug_dec_regC             (decode_regC), //output  wire [33:0]
        .debug_dec_write_regA       (decode_write_regA), //output  wire [33:0]
        //exec
        .debug_exec_instruction (),
        .debug_exec_alu_regA    (),
        .debug_exec_alu_regB    (),
        .debug_exec_alu_out     (),
        //data mem
        .debug_mem_address      (mem_address),     //output  wire [33:0]
        .debug_mem_output_data  (mem_output_data),     //output  wire [31:0]
        .debug_mem_input_data   (mem_input_data),     //output  wire [31:0]
        .debug_mem_instruction  ()
    );

    //////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////
    ///////////                 TESTBENCH                 ////////////
    /////////////////////////////////////////////////////////////////
    integer n_test = 0;
  
    task esperar_fetch(
      	input  [31:0]    expected_instruction,
        input  [7:0]     expected_pc,
        input  [7:0]     expected_pc_in
    );
      if(   
        (expected_instruction == fetch_pc_instruction) &&
        (expected_pc == fetch_pc_out) &&
        (expected_pc_in == fetch_pc_input)
        )
      begin
        	$display("TEST N %d PASSED", n_test);
          end
          else begin
            $display("TEST N %d FAILED FETCH: Expected instruction %h and obtained instruction %h", 
            n_test, expected_instruction, fetch_pc_instruction);
            $display("ALSO Expected PC %h and obtained PC %h", 
            expected_pc, fetch_pc_out);
            $display("ALSO Expected PC Bias %h and obtained %h",
            expected_pc_in, fetch_pc_input);
		end
    endtask

    task esperar_decode(
      	input  [33:0]    expected_regB,
        input  [33:0]    expected_regC,
        input  [33:0]    expected_regA
    );
      if(   
        (expected_regB == decode_regB) &&
        (expected_regC  == decode_regC) &&
        (expected_regA  == decode_write_regA)
        )
      begin
        	$display("TEST N %d PASSED", n_test);
          end
          else begin
            $display("TEST N %d FAILED REG FILE: Expected regB %h and obtained  %h", 
            n_test, expected_regB, decode_regB);
            $display("ALSO Expected regC %h and obtained %h", 
            expected_regC, decode_regC);
            $display("ALSO Expected regA %h and obtained %h",
            expected_regA, decode_write_regA);
		end
    endtask

    task esperar_mem(
      	input  [7:0]    expected_address,
      	input  [31:0]    expected_in,
      	input  [31:0]    expected_out
    );
      if(   
        (expected_address == mem_address) &&
        (expected_out == mem_output_data) &&
        (expected_in == mem_input_data)
        )
      begin
        	$display("TEST N %d PASSED", n_test);
          end
          else begin
            $display("TEST N %d FAILED MEM: Expected ADDR %h and obtained  %h", 
            n_test, expected_address, mem_address);
            $display("ALSO Expected OUT %h and obtained %h", 
            expected_out, mem_output_data);
            $display("ALSO Expected IN %h and obtained %h",
            expected_in, mem_input_data);
            end
    endtask

  
    initial begin
        $dumpfile("dump.vcd"); 
        $dumpvars;  
        $display("-----------starting--------------");
        #4 rst_n = 1'b1;

        //Filling the whole pipeline
      	n_test=0;
                        //Fetch OR
        @(posedge clk); //Fetch INV
        @(posedge clk); //Fetch AND
        @(posedge clk); //Fetch ADDC
        @(posedge clk); //Fetch SUBC

        //Instrucio or, inv, and, addc, subc
        #1; 
      	esperar_fetch(32'hd800779f, 8'd4, 8'd6);
      	esperar_decode(34'h3ffffffff, 34'h3ffffffff, 34'h000000001);	//ADDC: 31, 31, 30, regA: OR 
      	esperar_mem(8'hfd, 32'hffffffff, 32'h0d484321);
      	
        ////////////////  
      	n_test=1;
      	@(posedge clk); //Fetch ADD
      	@(posedge clk); //Fetch SUB
      	@(posedge clk); //Fetch LOAD
      	@(posedge clk); //Fetch STORE
		//MEM: ADD
      	//EXEC: SUB
      	//DEC: LOAD
      	//FETCH: STORE
        #1; 
      	//pc: instruction, pc, pc_bias
      	esperar_fetch(32'h1009d2a0, 8'd8, 8'd7);
      	//reg file: regB, regC, in_regA
      	esperar_decode(34'd23, 34'd22, 34'h30000001c);
      	//mem: addr, mem_in, mem_out
      	esperar_mem(8'd53, 32'd22, 32'h0d484321);
      
      	///////////////////////
      	n_test=2;
      	@(posedge clk); //Fetch 
      	@(posedge clk); //Fetch ADD
      	@(posedge clk); //Fetch SUB
      	@(posedge clk); //Fetch LOAD
      	@(posedge clk); //Fetch STORE
		
        #1; 
      	//pc: instruction, pc, pc_bias
      	esperar_fetch(32'hc0000bfe, 8'd13, 8'd26);
      	//reg file: regB, regC, in_regA
      	esperar_decode(34'd1, 34'd2, 34'h00000003f);
      	//mem: addr, mem_in, mem_out
      esperar_mem(8'h42, 32'h0000007f, 32'hdbbdaeea);
      
      	///////////////////////
      	n_test=3;
      	@(posedge clk); //Fetch 
      	@(posedge clk); //Fetch 
      	@(posedge clk); //Fetch 
      	@(posedge clk); //Fetch 
		
        #1; 
      	//pc: instruction, pc, pc_bias
      	esperar_fetch(32'h8000041f, 8'd16, 8'h2c);
      	//reg file: regB, regC, in_regA
      	esperar_decode(34'd3, 34'd0, 34'd0);
      	//mem: addr, mem_in, mem_out
      	esperar_mem(8'h2c, 32'd0, 32'h0025f92f);
      
      	///////////////////////
      	n_test=4;
      	@(posedge clk); //Fetch 
      	@(posedge clk); //Fetch 
     
		
        #1; 
      	//pc: instruction, pc, pc_bias
     	  esperar_fetch(32'h48010020, 8'd18, 8'd0);
      	//reg file: regB, regC, in_regA
      	esperar_decode(34'd0, 34'd0, 34'd30); //NOP!!
      	//mem: addr, mem_in, mem_out
      	esperar_mem(8'd3, 32'd0, 32'h4d10102a);
      
      	///////////////////////
      	n_test=5;
      	@(posedge clk); //Fetch 
      	@(posedge clk); //Fetch 
      	@(posedge clk); //Fetch 
     
		
        #1; 
      	//pc: instruction, pc, pc_bias
      	esperar_fetch(32'h50010440, 8'd21, 8'd0);
      	//reg file: regB, regC, in_regA
      	esperar_decode(34'd0, 34'd0, 34'd0);  //NOP!!
      	//mem: addr, mem_in, mem_out
      	esperar_mem(8'd3, 32'd0, 32'h4d10102a);
      
      	///////////////////////
      	n_test=6;
      	@(posedge clk); //Fetch 
      	@(posedge clk); //Fetch 
      	@(posedge clk); //Fetch 
     
		
        #1; 
      	//pc: instruction, pc, pc_bias
      	esperar_fetch(32'h5800f820, 8'd25, 8'd0);
      	//reg file: regB, regC, in_regA
      	esperar_decode(34'd0, 34'd0, 34'd0);  //NOP!!
      	//mem: addr, mem_in, mem_out
      	esperar_mem(8'd4, 32'd0, 32'he4f7febc);

        ///////////////////////
      	n_test=7;
      	@(posedge clk); //Fetch 
      	@(posedge clk); //Fetch 
      	@(posedge clk); //Fetch 
     
		
        #1; 
      	//pc: instruction, pc, pc_bias
      	esperar_fetch(32'h60007820, 8'd27, 8'd0);
      	//reg file: regB, regC, in_regA
      	esperar_decode(34'd0, 34'd0, 34'd0);  //NOP!!
      	//mem: addr, mem_in, mem_out
      	esperar_mem(8'd2, 32'd0, 32'h245c3221);

        ///////////////////////
      	n_test=8;
      	@(posedge clk); //Fetch 
      	@(posedge clk); //Fetch 
      	@(posedge clk); //Fetch 
     
	
        #1; 
      	//pc: instruction, pc, pc_bias

        //Note that pc went pc+1, thats because jal instruction (now)
        //is one pc after bv instruction
      	esperar_fetch(32'h68000c1f, 8'd28, 8'd0); //jal!!
      	//reg file: regB, regC, in_regA
      	esperar_decode(34'd0, 34'd0, 34'd0);  //NOP!!
      	//mem: addr, mem_in, mem_out
      	esperar_mem(8'd1, 32'd00, 32'h3d155142);
      

        ///////////////////////
      	n_test=9;
      	@(posedge clk); //Fetch 
      	@(posedge clk); //Fetch 
      	@(posedge clk); //Fetch 
        #1; 
      	//pc: instruction, pc, pc_bias
		esperar_fetch(32'h7000081f, 8'd31, 8'd0); //jral!!!
      	//reg file: regB, regC, in_regA
      	esperar_decode(34'd0, 34'd0, 34'd0);  //NOP!!
      	//mem: addr, mem_in, mem_out
      	esperar_mem(8'd3, 32'd0, 32'h4d10102a);
      
      ///////////////////////
      	n_test=10;
      	@(posedge clk); //Fetch 
      	@(posedge clk); //Fetch 
      	@(posedge clk); //Fetch 
        #1; 
      	//pc: instruction, pc, pc_bias
      	esperar_fetch(32'h78000240, 8'd33, 8'd0); //ret!!!
      	//reg file: regB, regC, in_regA
      	esperar_decode(34'd0, 34'd0, 34'd0);  //NOP!!
      	//mem: addr, mem_in, mem_out
      	esperar_mem(8'd2, 32'd0, 32'h245c3221);
      
      	///////////////////////
      	n_test=11;
      	@(posedge clk); //Fetch 
      	@(posedge clk); //Fetch 
      	@(posedge clk); //Fetch 
        #1; 
      	//pc: instruction, pc, pc_bias
      	esperar_fetch(32'h48010020, 8'd18, 8'd0); //ret jumped to bz!!!
      	//reg file: regB, regC, in_regA
      	esperar_decode(34'd0, 34'd0, 34'd0);  //NOP!!
      	//mem: addr, mem_in, mem_out
      	esperar_mem(8'h12, 32'd0, 32'h40255a28);
      
      
      
      
      
      
      
      
      
        #300;
        $display("-----------finishing--------------");
        $finish;
      
    end

endmodule // 