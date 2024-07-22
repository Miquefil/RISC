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
  	reg [7:0]     tb_mem [0:1023];
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
    parameter PC_MEM_FILE = "pc_mem_RANDOM.hex";
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
    integer n_test  = 0;
    integer n_logic = 0;
    integer n_arith = 0;
    integer n_move  = 0;
    integer n_flow  = 0;
  
	////////////////////////////////////////////////////////////////
  	////////		TASK EXPECT FOR LOGIC OPERATIONS 	///////////
  	//////////////////////////////////////////////////////////////
  	// Section of code to implement predictor for arithmetic operations
  	reg [32:0] computation = 33'd0;
  
  	localparam L_OR  = 3'b000;
	  localparam L_INV = 3'b001;
  	localparam L_AND = 3'b010;
  
    task esperar_logic(
      	input  [2:0]    operation
    );
        n_test = n_test + 1;     
        n_logic = n_logic + 1; 
      	case(operation)
          L_OR  : computation = {2'b00, {(regB[31:0]|regC[31:0])}};
          L_AND : computation = {2'b00, {(regB[31:0]&regC[31:0])}};
          L_INV : computation = {2'b00, {(~regB[31:0])}};
        endcase
      
      
      	if(write_regA ==  computation)begin
          	$display("TEST N %d PASSED", n_test);
          end
          else begin
            $display("TEST N %d FAILED: Expected %h and obtained %h", n_test, computation, write_regA);
            $display("FAILED ON OP: %b in LOGIC operations", operation);
		end
      
    endtask
  	////////////////////////////////////////////////////////////////
  	////////		TASK EXPECT FOR ARITHMETIC OPERATIONS 	///////
  	//////////////////////////////////////////////////////////////
  	// Section of code to implement predictor for arithmetic operations  
  	localparam A_ADD  = 3'b000;
	  localparam A_SUB  = 3'b001;
  	localparam A_ADDC = 3'b010;
  	localparam A_SUBC = 3'b011;
  
  	reg [33:0] calculus_comparation = 34'd0;
  
  	task esperar_arithmetic(
      	input  [2:0]    operation
    );
      n_test = n_test + 1; 
      n_arith = n_arith + 1;
      computation = 34'd0;
      case(operation)
        A_ADD  : begin 
          computation = (regB[31:0] + regC[31:0]);
          calculus_comparation[32:0] = computation;
          if (regB[31]&& regC[31] && (!computation[31])) begin
            calculus_comparation[33] = 1'b1;
          end
        end
        A_SUB  : begin
            computation = (regB[31:0] - regC[31:0]);
            calculus_comparation[32:0] = computation;
            if (regB[31]&& (!regC[31]) && (!computation[31])) begin
              calculus_comparation[33] = 1'b1;
            end
        end
          
        A_ADDC : begin
            computation = (regB[31:0] + regC[31:0] + {31'd0, regC[32]} );
            calculus_comparation[32:0] = computation;
            if (regB[31]&& regC[31] && (!computation[31])) begin
              calculus_comparation[33] = 1'b1;
            end
        end
        A_SUBC : begin
            computation = (regB[31:0] - regC[31:0] - {31'd0, regC[32]} );
            calculus_comparation[32:0] = computation;
            if (regB[31]&& (!regC[31]) && (!computation[31])) begin
              calculus_comparation[33] = 1'b1;
            end
        end
      endcase
      
      
      if(write_regA ==  calculus_comparation)begin
        	$display("TEST N %d PASSED", n_test);
          end
          else begin
            $display("TEST N %d FAILED: Expected %h and obtained %h", n_test, computation, write_regA);
            $display("FAILED ON OP: %b in ARITHMETIC operations", operation);
		end
      
      
    endtask
  
  
  	
  	////////////////////////////////////////////////////////////////
  	////////		TASK EXPECT FOR MOVEMENT OPERATIONS 	///////
  	//////////////////////////////////////////////////////////////
  	// Section of code to implement predictor for moving operations
  	// ---IMPORTANT! It is important to clarify that WE WONT test 
  	// memories and their operations, that means we cannot assure
  	//that writing, reading, and internal operations of memories are
  	// correct, however we gonna assure that input writing channel, 
  	// addressing, and outside treatment of memory is correctly done.
  
  	localparam M_LOAD  	= 3'b000;
  	localparam M_STORE 	= 3'b010;
  	localparam M_LOADI 	= 3'b001;
  	localparam M_STOREI = 3'b011;
  	localparam M_MOV 	= 3'b100;
  
  	reg[31:0]	comparador_expect;
  	reg[31:0]	comparador_actual;
    reg[33:0] pointer;
  
  	task esperar_movement(
      	input  [2:0]    operation
    );
        n_test = n_test + 1; 
        n_move = n_move + 1;
      
      	case(operation)
          //REMEMBER! Memory has a not clocked output, that means as soon as i_address is asserted, output is asserted. Nevertheless, Memory IS clocked at its writing channel, that means once i_address is asserted, memory will be written in next clock.
        M_LOAD  : begin
            	//In load we compare if the actual RISC output of mem is correct
            	tb_mem_pointer 	  = regB[31:0] + imm_rc;
            	comparador_expect = {{tb_mem[tb_mem_pointer[7:0]*4]}, {tb_mem[tb_mem_pointer[7:0]*4+8'd1]}, {tb_mem[tb_mem_pointer[7:0]*4+8'd2]}, {tb_mem[tb_mem_pointer[7:0]*4+8'd3]}};
            	comparador_actual = mem_data_output;
          end
  			M_STORE : begin
              	//In store we compare that the actual RISC mem input is regC. BE CAREFUL!! It doesnt test
              	//that actually the information is gonna be written.
            	  tb_mem_pointer      = regB[31:0] + imm_rc;
              	comparador_expect   = regC;
              	comparador_actual   = mem_data_input;
                if(comparador_expect == comparador_actual) begin
                    pointer = regB[31:0] + imm;
                    tb_mem[4*pointer[7:0]+8'd3]    = regC[31:24];
                    tb_mem[4*pointer[7:0]+8'd2]    = regC[23:16];
                    tb_mem[4*pointer[7:0]+8'd1]    = regC[15:8];
                    tb_mem[4*pointer[7:0]]         = regC[7:0];
                    $display("%h",regC);
                    $display("%h",tb_mem[pointer[7:0]]);   
                end
          end
  			M_LOADI : begin
              	//In LoadI, we compare that write input channel of RegA is Imm*
              	comparador_expect 	= imm_rc;
              	comparador_actual	= write_regA;
          end
  			M_STOREI: begin
              	comparador_expect 	= imm_rc;
              	comparador_actual	= mem_data_input;
                if(comparador_expect == comparador_actual) begin
                    pointer = regB[31:0];
                    tb_mem[4*pointer[7:0]+8'd3]   = imm_rc[31:24];
                    tb_mem[4*pointer[7:0]+8'd2]   = imm_rc[23:16];
                    tb_mem[4*pointer[7:0]+8'd1]   = imm_rc[15:8];
                    tb_mem[4*pointer[7:0]]        = imm_rc[7:0];
                    $display("%h",imm_rc);
                    $display("%h",tb_mem[regB[7:0]]);
                end
          end
  			M_MOV 	: begin
              	comparador_expect	= regB;
              	comparador_actual	= write_regA;
          end
		endcase
      	
      if(comparador_expect ==  comparador_actual)begin
        	$display("TEST N %d PASSED", n_test);
          end
          else begin
            $display("TEST N %d FAILED: Expected %h and obtained %h", n_test, comparador_expect, comparador_actual);
            $display("FAILED ON OP: %b in MOVEMENT operations", operation);
	  end
    endtask
  
  	////////////////////////////////////////////////////////////////
  	////////		TASK EXPECT FOR FLOW CTRL OPERATIONS 	///////
  	//////////////////////////////////////////////////////////////
  	// Section of code to implement predictor for flow ctrl operations
    // IMPORTANT!!!! -- we wont verify internal program counter, instead
    // we're going to verify that actual program counter and the following one is correct
  
    localparam  F_JUMP  	  = 3'b000;
  	localparam  F_BZ 	      = 3'b001;
  	localparam  F_BNZ 	    = 3'b010;
  	localparam  F_BC        = 3'b011;
  	localparam  F_BV 	      = 3'b100;
    localparam  F_JAL 	    = 3'b101;
    localparam  F_JRAL 	    = 3'b110;
    localparam  F_RET 	    = 3'b111;
  
    reg         side_check;         //SIDE CHECK: jal, jral need two comparisons. 
                                    //Auxiliary reg to check both operations
    reg [34:0] sum;
  	task esperar_flow(
      	input  [2:0]    operation
    );
        n_test = n_test + 1; 
        n_flow = n_flow + 1;
        side_check = 1'b1;
      	case(operation)
            F_JUMP: begin
                comparador_actual   = pc_in;
                comparador_expect   = pc + regB[31:0];
            end
            F_BZ:   begin
                comparador_actual   = 32'd1;
                comparador_expect = 32'd1;
                if(regC[31:0] == 32'd0) begin
                    comparador_actual = pc_in;
                    comparador_expect = pc + regB[31:0] + imm;
                    comparador_expect = comparador_expect[7:0];
                end
            end
            F_BNZ:  begin
                comparador_actual   = 32'd1;
                comparador_expect = 32'd1;
                if(regC[31:0] != 32'd0) begin
                    comparador_actual = pc_in;
                    comparador_expect = pc + regB[31:0] + imm;
                    comparador_expect = comparador_expect[7:0];
                end
            end
            F_BC:   begin
                comparador_actual   = 32'd1;
                comparador_expect = 32'd1;
                if(regC[32] == 1'b1) begin
                    comparador_actual = pc_in;
                    comparador_expect = pc + regB[31:0] + imm;
                    comparador_expect = comparador_expect[7:0];
                end
            end
            F_BV:   begin
                comparador_actual   = 32'd1;
                comparador_expect = 32'd1;
                if(regC[33] == 1'b1) begin
                    comparador_actual = pc_in;
                    comparador_expect = pc + regB[31:0] + imm;
                    comparador_expect = comparador_expect[7:0];
                end
            end
            F_JAL:  begin
                comparador_actual   = write_regA;
                comparador_expect   = pc + 8'd4;
                sum = pc + imm_rc;

              	if(pc_in !=  sum[7:0])begin
                        side_check = 1'b0;
                        $display("Failed side check!");
                end
            end
            F_JRAL: begin
                comparador_actual   = write_regA;
                comparador_expect   = pc + 8'd4;
                sum = pc + regB[31:0] + imm_rc;
              if(pc_in !=  sum[7:0])begin
                        side_check = 1'b0;
                        $display("Failed side check!");
                end
            end
            F_RET:  begin
                comparador_actual = pc_in;
                comparador_expect = regB[7:0];
            end
		endcase
      	
      	if((comparador_expect ==  comparador_actual) && (side_check == 1'b1)) begin
        	$display("TEST N %d PASSED", n_test);
          end
          else begin
            $display("TEST N %d FAILED: Expected %h and obtained %h", n_test, comparador_expect, comparador_actual);
            $display("FAILED ON OP: %b in FLOW CTRL operations", operation);
	  end
    endtask


    ///////////////////////     GENERAL TASK    /////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////
    //The purpose of this task is: reading instruction from tb PC memory, we call
    //the appropiate "esperar" task in order to execute the test
    localparam MOVEMENT     = 2'b00;
    localparam LOGIC        = 2'b10;
    localparam ARITHMETIC   = 2'b11;
    localparam FLW_CTRL     = 2'b01;

    task general_task (
        input [1:0]     T,
        input [2:0]     OPC
    );
      	if((T == risc_instruction_T) && (OPC == risc_instruction_OPC)) begin
            case (T)
                MOVEMENT:  begin
                    case (OPC)
                        M_LOAD:     esperar_movement(M_LOAD);
                        M_STORE:    esperar_movement(M_STORE);
                        M_LOADI:    esperar_movement(M_LOADI);
                        M_STOREI:   esperar_movement(M_STOREI);
                        M_MOV:      esperar_movement(M_MOV);
                    endcase
                end
                LOGIC: begin
                    case (OPC)
                        L_OR:       esperar_logic(L_OR);
                        L_INV:      esperar_logic(L_INV);
                        L_AND:      esperar_logic(L_AND);
                    endcase
                end
                ARITHMETIC: begin
                    case (OPC)
                        A_ADD:      esperar_arithmetic(A_ADD);
                        A_SUB:      esperar_arithmetic(A_SUB);
                        A_ADDC:     esperar_arithmetic(A_ADDC);
                        A_SUBC:     esperar_arithmetic(A_SUBC);
                    endcase
                end
                FLW_CTRL: begin
                    case (OPC)
                        F_JUMP:     esperar_flow(F_JUMP);
                        F_BZ:       esperar_flow(F_BZ);
                        F_BNZ:      esperar_flow(F_BNZ);
                        F_BC:       esperar_flow(F_BC);
                        F_BV:       esperar_flow(F_BV);
                        F_JAL:      esperar_flow(F_JAL);
                        F_JRAL:     esperar_flow(F_JRAL);
                        F_RET:      esperar_flow(F_RET);
                    endcase
                end
            endcase
        end
        else begin
            $display("TEST N %d: RISC instruction, and TESTBENCH instruction are not the same.", n_test);
        end
        
    endtask

integer j;
  
    /////////////////////// MAIN INITIAL TEST 	/////////////////////////////////
  
////////////////////////////////////////////////////////////////////////////
    initial begin
        $dumpfile("dump.vcd"); 
        $dumpvars;  
        $display("-----------starting--------------");
        rst_n = 1'b1;
      	
        
			#3 general_task(tb_T, tb_OPC);
      for (j = 0; j < 200 ; j=j+1) begin
          @(posedge clk); #1 general_task(tb_T, tb_OPC);
      end

      $display("------> Logic Instructions Executed: %d", n_logic);
      $display("------> Arithmetic Instructions Executed: %d", n_arith);
      $display("------> Movement Instructions Executed: %d", n_move);
      $display("------> FlowCtrl Instructions Executed: %d", n_flow);

      #200	
      $display("-----------finishing--------------");
      $finish;
    end

endmodule //