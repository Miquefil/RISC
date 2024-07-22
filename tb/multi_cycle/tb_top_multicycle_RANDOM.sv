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
`include "top_multicycle.v"
`include "multicycle_model.sv"
module top_onecycle_target_tb (
    // Ports here
);
    //Implemented Assembly Bibliotheque ///////////////////////////////
    parameter MOVEMENT      = 2'b00 ;
                parameter load          = 3'b000;
                parameter store         = 3'b010;
                parameter loadi         = 3'b001;
                parameter storei        = 3'b011;
                parameter mov           = 3'b100;
    parameter LOGIC         = 2'b10 ;
                parameter or_l          = 3'b000;
                parameter inv_l         = 3'b001;
                parameter and_l         = 3'b010;
    parameter ARITHMETIC    = 2'b11 ;
                parameter add_a         = 3'b000;
                parameter sub_a         = 3'b001;
                parameter addc_a        = 3'b010;
                parameter subc_a        = 3'b011;
    parameter FLOWCTRL      = 2'b01 ;
                parameter jump          = 3'b000;
                parameter bz            = 3'b001;
                parameter bnz           = 3'b010;
                parameter bc            = 3'b011;
                parameter bv            = 3'b100;
                parameter jal           = 3'b101;
                parameter jral          = 3'b110;
                parameter ret           = 3'b111;

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
    reg                             rst_n  = 1'b0;
    
    wire [31:0]                     fetch_pc_instruction;
    wire [7:0]                      fetch_pc_out;
    wire [7:0]                      fetch_pc_input;
    wire [31:0]                     decode_instruction;
    wire [33:0]                     decode_regB;
    wire [33:0]                     decode_regC;
    wire [33:0]                     decode_write_regA;
    wire [31:0]                     execute_instruction;
    wire [2:0]                      execute_alu_ctrl;
    wire [33:0]                     execute_alu_regA;
    wire [33:0]                     execute_alu_regB;
    wire [33:0]                     execute_alu_out;
    wire [7:0]                      mem_address;
    wire [31:0]                     mem_instruction;
    wire [31:0]                     mem_output_data;
    wire [31:0]                     mem_input_data;

    wire [1:0]      top_exec_T;
    wire [2:0]      top_exec_OPC;
    assign  top_exec_T               =  execute_instruction[31:30];
    assign  top_exec_OPC             =  execute_instruction[29:27];

    wire [1:0]      top_mem_T;
    wire [2:0]      top_mem_OPC;
    assign  top_mem_T               =  mem_instruction[31:30];
    assign  top_mem_OPC             =  mem_instruction[29:27];

  
    top_multicycle  
    #(
        .MEMORY_FILE            ("mem.hex"),
//         .PC_FILE                ("pc_mem_target_multi.hex"),
      	.PC_FILE                ("pc_mem_RANDOM.hex"),
        .REG_FILE               ("register_mem.hex")
    )
    u_top_multi
    (
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
        .debug_mem_input_data   (mem_input_data)  //output  wire [31:0]
    );

    wire [31:0]                     mod_fetch_pc_instruction;
    wire [7:0]                      mod_fetch_pc_out;
    wire [7:0]                      mod_fetch_pc_input;
    wire [31:0]                     mod_decode_instruction;
    wire [33:0]                     mod_decode_regB;
    wire [33:0]                     mod_decode_regC;
    wire [33:0]                     mod_decode_write_regA;
    wire [33:0]                     mod_execute_alu_out;
    wire [31:0]                     mod_execute_instruction;
    wire [33:0]                     mod_execute_alu_regA;
    wire [33:0]                     mod_execute_alu_regB;
    wire [31:0]                     mod_mem_instruction;
    wire [7:0]                      mod_mem_address;
    wire [31:0]                     mod_mem_output_data;
    wire [31:0]                     mod_mem_input_data;

    wire mod_debug_mem_to_reg;
    wire mod_hz_A;
    wire mod_hz_B;
    wire mod_stall;
    wire rst_decode;
    wire rst_execute;
    wire writingA;
    wire [33:0]                     debug_wb_a;
    wire [31:0]         model_wb_instruction;
    wire [33:0] model_mem_alu;
    multicycle_model #(
        .MEMORY_FILE        ("mem.hex"),
//         .PC_FILE            ("pc_mem_target_multi.hex"),
      	.PC_FILE                ("pc_mem_RANDOM.hex"),
        .REG_FILE           ("register_mem.hex")
    )
    u_model
    (
    .clk                    (clk),
    .rst_n                  (rst_n),
    //fetch
    .model_pc_instruction   (mod_fetch_pc_instruction), //output  wire [31:0]
    .model_pc_out           (mod_fetch_pc_out), //output  wire [7:0]
    .model_pc_input         (mod_fetch_pc_input), //output  wire [7:0]
    //decode
    .model_dec_instruction  (mod_decode_instruction), //output  wire [31:0]
    .model_dec_regB         (mod_decode_regB), //output  wire [33:0]
    .model_dec_regC         (mod_decode_regC), //output  wire [33:0]
    .model_dec_write_regA   (mod_decode_write_regA), //output  wire [33:0]
    //execute 
    .model_exec_alu_out     (mod_execute_alu_out), //output  wire [33:0]
    .model_exec_instruction (mod_execute_instruction), //output  wire [31:0]
    .model_exec_alu_regA    (mod_execute_alu_regA), //output  wire [33:0]
    .model_exec_alu_regB    (mod_execute_alu_regB), //output  wire [33:0]
    //data mem
    .model_mem_instruction  (mod_mem_instruction), //output  wire [31:0]
    .model_mem_address      (mod_mem_address), //output  wire [7:0]
    .model_mem_output_data  (mod_mem_output_data), //output  wire [31:0]
    .model_mem_input_data   (mod_mem_input_data), //output  wire [31:0]
    .debug_mem_to_reg       (mod_debug_mem_to_reg),
    .hazard_A               (mod_hz_A),
    .hazard_B               (mod_hz_B),
    .debug_stall            (mod_stall),
    .debug_rst_decode       (rst_decode),
    .debug_rst_execute      (rst_execute),
    .debug_wb_wen           (writingA),
    .debug_wb_a             (debug_wb_a),
    .model_wb_instruction   (model_wb_instruction),
    .debug_mem_alu          (model_mem_alu)
    );



/////////////////////// MAIN INITIAL TEST 	/////////////////////////////////
integer n_test = 0;   
bit check_alu   	= 1'b1;    
bit check_mem_in    = 1'b1;
bit check_mem_out   = 1'b1;
bit check_addr = 1'b1;
task esperar_decode(
    );
        n_test = n_test + 1;
      if(   
        (fetch_pc_instruction  == mod_fetch_pc_instruction) &&
        (fetch_pc_out          == mod_fetch_pc_out) &&
        (decode_instruction    == mod_decode_instruction)
        )
      begin
            check_mem_out   = 1'b1;    
            check_mem_in    = 1'b1;
            check_alu       = 1'b1;
        	check_addr      = 1'b1;
        if( (top_exec_T == (ARITHMETIC||LOGIC)))
        begin
          	//if computations are needed
        	check_alu = mod_execute_alu_out == execute_alu_out;
        end
        if ((top_exec_T==MOVEMENT)&&(top_exec_OPC inside {load,store,storei})) begin
          	//if movement operations with memory are requires
          	check_addr = mod_mem_address == mem_address;
        end
        if(top_mem_T == MOVEMENT) begin
            //comparing memory out/in when a movement instruction detected in memory stage
            case (top_mem_OPC)
                load:   check_mem_out = mem_output_data == mod_mem_output_data;
                store:  check_mem_in  = mod_mem_input_data == mem_input_data;
                storei: check_mem_in  = mod_mem_input_data == mem_input_data;
            endcase
        end

            
           if(check_alu && check_mem_in && check_mem_out && check_addr) begin
        	$display("TEST N %d PASSED", n_test);
        end 
        else begin
          $display("TEST N %d FAILED, alu comparation is %d, addr is %d, mem out is %d, mem in is %d", n_test, check_alu, check_addr, check_mem_out, check_mem_in);
          $display("pipeline exec instruction: %b", execute_instruction);
          $display("model exec instruction: %b", mod_execute_instruction);
          $display("pipeline mem instruction: %b", mem_instruction);
          $display("model mem instruction: %b", mod_mem_instruction);
          $display("alu model: %h, alu real: %h", mod_execute_alu_out, execute_alu_out);
          $display("mem model: %h, mem real: %h", mod_mem_output_data, mem_output_data);
          $display("model address: %h, real address: %h, at %d", mod_mem_address, mem_address, $time);
        end
      end
          else begin
            $display("TEST N %d FAILED", n_test);
        end
    endtask


integer j;  
////////////////////////////////////////////////////////////////////////////
    initial begin
        $dumpfile("dump.vcd"); 
        $dumpvars;  
        $display("-----------starting--------------");
        rst_n = 1'b1;
      	
        
			// #3 general_task(tb_T, tb_OPC);
      	for (j = 0; j < 200 ; j=j+1) begin
            @(posedge clk); 
            #1 esperar_decode();
        end

    //   $display("------> Logic Instructions Executed: %d",       n_logic);
    //   $display("------> Arithmetic Instructions Executed: %d",  n_arith);
    //   $display("------> Movement Instructions Executed: %d",    n_move);
    //   $display("------> FlowCtrl Instructions Executed: %d",    n_flow);

      #200	
      $display("-----------finishing--------------");
      $finish;
    end

    // function bit alu_check();
    //     alu_check = mod_execute_alu_out == execute_alu_out;
    // endfunction

    // function bit mem_in_check();
    //     mem_in_check = mod_mem_input_data == mem_input_data;
    // endfunction

    // function bit mem_out_check();
    //     mem_out_check = mem_output_data == mod_mem_output_data;
    // endfunction

endmodule //