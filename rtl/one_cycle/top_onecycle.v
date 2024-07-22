//////////////////////////////////////////////////////////////////////
// Module Name: top_onecycle
//              - top module for implementing a One-Cycle RISC Processor
//
// Author: Miqueas Filsinger
// Date: 14-06-2024
//
// Revision History:
// - Date: 
//
//////////////////////////////////////////////////////////////////////

`include "defines.v"
`include "program_counter.v"
`include "register_file.v"
`include "alu.v"
`include "memory.v"

module top_onecycle (
    input   wire                            clk,
    input   wire                            rst_n,


    //Debug outputs
    output  wire [33:0]                     debug_regB,
    output  wire [33:0]                     debug_regC,
    output  wire [33:0]                     debug_write_regA,
    output  wire [33:0]                     debug_alu_out,
    output  wire [31:0]                     debug_pc_instruction,
    output  wire [7:0]                      debug_pc_out,
    output  wire [7:0]                      debug_pc_input,
    output  wire [31:0]                     debug_mem_output_data,
    output  wire [31:0]                     debug_mem_input_data
);  
    ////////////////////////////////////////////////////////////////
    //          PROGRAM COUNTER                     ////////////////
    ////////////////////////////////////////////////////////////////
    wire    [31:0]                          pc_instruction;
    reg                                    	pc_bias_en;
    reg                                     pc_load_en;
	reg     [7:0]                           pc_bias;
    wire    [7:0]                           program_counter;
    program_counter   u_pc(
        .clk                    (clk),
        .bias_en                (pc_bias_en),
        .bias                   (pc_bias),
        .instruction_output     (pc_instruction),
        .program_counter        (program_counter)
    );
    ////////////////////////////////////////////////////////////////
    //          REGISTER FILE                       ////////////////
    ////////////////////////////////////////////////////////////////
    reg     [`REGISTERS_AD_WIDTH-1:0]       regA_ad;
    reg     [`REGISTERS_AD_WIDTH-1:0]       regB_ad;
    reg     [`REGISTERS_AD_WIDTH-1:0]       regC_ad;
    reg     [`REGISTERS_WIDTH-1:0]          regA_in_data;
    wire    [`REGISTERS_WIDTH-1:0]          regB_out_data;
    wire    [`REGISTERS_WIDTH-1:0]          regC_out_data;
    reg                                     regA_wen;

    wire                                     reg_C_carry;
  	assign  reg_C_carry = regC_out_data[`REGISTERS_WIDTH-2];

    wire                                     reg_C_overflow;
  	assign  reg_C_overflow = regC_out_data[`REGISTERS_WIDTH-1];
    register_file #(
        .REG_WIDTH      (34),
        .ADDR_WIDTH     (5)
    ) 
    u_register(
        .clk                        (clk),                  //input   wire
        .rst_n                      (rst_n),
        .i_address_reg_a            (regA_ad),              //input   wire  <- Writing register
        .i_wenable_reg_a            (regA_wen),         //input   wire
        .i_writedata_reg_a          (regA_in_data),             //input   wire
        .i_address_reg_b            (regB_ad),              //input   wire
        .i_address_reg_c            (regC_ad),              //input   wire
        .o_data_regb                (regB_out_data),        //output  wire
        .o_data_regc                (regC_out_data)         //output  wire
    );
    ////////////////////////////////////////////////////////////////
    //          MEMORY                              ////////////////
    ////////////////////////////////////////////////////////////////
    wire                                    mem_rst_n;
    reg                                     mem_wen;
    wire [7:0]                              mem_addr;
    reg  [31:0]                             mem_input_data;
    wire [31:0]                             mem_output_data;
    memory  #(
        .MEM_WIDTH      (8),
        .DATA_WIDTH     (32),
      	.ADDR_WIDTH     (8)
    )
    u_mem(
            .clk            (clk),                  //input   wire
            .rst_n          (mem_rst_n),            //input   wire
            .i_wenable      (mem_wen),              //input   wire
            .i_address      (mem_addr),             //input   wire
            .i_data         (mem_input_data),       //input   wire
            .o_data         (mem_output_data)       //output  wire
    );
    ////////////////////////////////////////////////////////////////
    /////          ALU                              ////////////////
    ////////////////////////////////////////////////////////////////
    reg     [2:0]                       alu_ctrl;
    reg     [`NB_REGISTERS-1:0]         alu_regA;
    reg     [`NB_REGISTERS-1:0]         alu_regB;
    wire    [`NB_REGISTERS-1:0]         alu_out;
    alu #(
        .NB_REGISTERS   (`NB_REGISTERS),       //34 bits normally
        .DATA_WIDTH     (`DATA_WIDTH)          //32 bits
    ) 
    u_alu (
        .i_alu_ctrl         (alu_ctrl),     //input   wire [0:2]              
        .i_data_A           (alu_regA),     //input   wire [0:NB_REGISTERS-1] 
        .i_data_B           (alu_regB),     //input   wire [0:NB_REGISTERS-1] 
        .o_data             (alu_out)       //output  wire [0:NB_REGISTERS-1] 
    );

    ////////////////////////////////////////////////////////////////
    //          DEBUG SIGNALS                       ////////////////
    ////////////////////////////////////////////////////////////////
    assign regB                         = regB_out_data;
    assign regC                         = regC_out_data;
    assign debug_alu_out                = alu_out;
    assign debug_pc_instruction         = pc_instruction;
    assign pc                           = program_counter;
    assign debug_mem_output_data        = mem_output_data;
    assign write_regA                   = regA_in_data;

    ///////////////////////////////////////////////////////////////////////
    //////////      ONE CYCLE CONTROLLER                        ///////////
    ///////////////////////////////////////////////////////////////////////
    reg  [1:0]                              instruction_T;
    reg  [2:0]                              instruction_OPC;
    reg  [11:0]                             instruction_Imm;
    reg  [4:0]                              instruction_RA_addr;
    reg  [4:0]                              instruction_RB_addr;
    reg  [4:0]                              instruction_RC_addr;

    //Imm_extension contains a 32 bits extension of Imm field conserving sign bit
    //Example: Imm              = 1010_0000_0001
    //         imm_extension    = 1000_0000_0000_0000_1010_0000_0001
    //                           |<----Extension---->|<---Imm------>
    //Imm_rc_extension contains a 32 bits extension of the concatenation of Imm and Rc,
    //conserving the Imm sign bit. Example:
    //         imm = 1010_0000_0001,     rc = 01110
    //         imm_extension    = 1000_0000_0001_0100_0000_0010_1110
    //                              28   24   16   12    8    4    0
    //                           |<---Ext---->|<------Imm--->|<RC->|
    wire [31:0]          imm_extension;
    assign imm_extension = (instruction_Imm[11])?{1'b1,{19{1'b0}}, {instruction_Imm}}
                                                :{1'b0,{19{1'b0}}, {instruction_Imm}};
    wire [31:0]          imm_rc_extension;
    assign imm_rc_extension = (instruction_Imm[11])?{1'b1, {14{1'b0}}, {instruction_Imm}, {instruction_RC_addr}}
                                                :{1'b0, {14{1'b0}}, {instruction_Imm}, {instruction_RC_addr}};

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

    //ALU operations:
    parameter ADD       = 3'd0;
    parameter SUB       = 3'd1;
    parameter ADDC      = 3'd2;
    parameter SUBC      = 3'd3;
    parameter EXCEPTION = 3'd4;



    //Logic & Control------------------------------------------------
    //mem addr: 
    reg     mem_addr_selector;
    assign  mem_addr = ~mem_addr_selector ? alu_out[7:0]:
                        regB_out_data[7:0];



    reg [34:0]                 sum;
    always @(*) begin
        //////////// Program Counter Initialitation ///////////
        instruction_T       = pc_instruction[31:30];
        instruction_OPC     = pc_instruction[29:27];
        instruction_Imm     = pc_instruction[26:15];
        instruction_RC_addr = pc_instruction[14:10];
        instruction_RB_addr = pc_instruction[9:5];
        instruction_RA_addr = pc_instruction[4:0];
      
      	regA_ad = instruction_RA_addr;
        regB_ad = instruction_RB_addr;
        regC_ad = instruction_RC_addr;
        ////////////  Default case declared first  ////////////
        ///////////////////////////////////////////////////////
        alu_ctrl            = EXCEPTION;    //Output zero
        alu_regA            = 34'd0;
        alu_regB            = 34'd0;
        
        regA_in_data        = 34'd0;
        regA_wen            = 1'b0;

        mem_input_data      = 32'd0;
        mem_addr_selector   = 1'b0;
        mem_wen             = 1'b0;

        pc_bias_en          = 1'b0;
        pc_load_en          = 1'b0;
        pc_bias             = 8'd0;

        sum                 = 35'd0;

        case (instruction_T)
            MOVEMENT: begin
                case (instruction_OPC)
                    load: begin
                        alu_ctrl            = ADD;
                        alu_regA            = regB_out_data;
                        alu_regB            = imm_rc_extension;
                        mem_addr_selector   = 1'b0;
                        regA_in_data        = {2'b00, mem_output_data}; 
                        regA_wen            = 1'b1;
                    end
                    store: begin
                        alu_ctrl            = ADD;
                        alu_regA            = regB_out_data;
                        alu_regB            = imm_extension;
                        mem_addr_selector   = 1'b0;
                        mem_wen             = 1'b1;
                        mem_input_data      = regC_out_data;
                    end
                    loadi: begin
                        regA_in_data        = {2'b00, imm_rc_extension}; 
                        regA_wen            = 1'b1;
                    end
                    storei: begin
                        mem_wen             = 1'b1;
                        mem_addr_selector   = 1'b1;
                        mem_input_data      = imm_rc_extension;
                    end
                    mov:    begin
                        regA_wen            = 1'b1;
                        regA_in_data        = regB_out_data; 
                    end
                endcase
            end
            LOGIC: begin
                case (instruction_OPC)
                    or_l: begin
                      	regA_in_data        = {2'b00, regB_out_data[31:0] | regC_out_data[31:0]}; 
                        regA_wen            = 1'b1;
                    end
                    inv_l: begin
                      	regA_in_data        = {2'b00, ~regB_out_data[31:0]};
                        regA_wen            = 1'b1;
                    end 
                    and_l: begin
                      	regA_in_data        = {2'b00, regB_out_data[31:0] & regC_out_data[31:0]};
                        regA_wen            = 1'b1;
                    end 
                endcase 
            end
            ARITHMETIC: begin
                case (instruction_OPC)
                    add_a: begin
                        alu_ctrl            = ADD;
                        alu_regA            = regB_out_data;
                        alu_regB            = regC_out_data;
                        regA_in_data        = alu_out;
                        regA_wen            = 1'b1;
                    end
                    sub_a: begin
                        alu_ctrl            = SUB;
                        alu_regA            = regB_out_data;
                        alu_regB            = regC_out_data;
                        regA_in_data        = alu_out;
                        regA_wen            = 1'b1;
                    end
                    addc_a: begin
                        alu_ctrl            = ADDC;
                        alu_regA            = regB_out_data;
                        alu_regB            = regC_out_data;
                        regA_in_data        = alu_out;
                        regA_wen            = 1'b1;
                    end
                    subc_a: begin
                        alu_ctrl            = SUBC;
                        alu_regA            = regB_out_data;
                        alu_regB            = regC_out_data;
                        regA_in_data        = alu_out;
                        regA_wen            = 1'b1;
                    end
                endcase
            end
            FLOWCTRL: begin
                case (instruction_OPC)
                    jump : begin
                        //TODO: Is this possible?
                        alu_ctrl            = ADD;
                        alu_regA            = regB_out_data;                     
                        alu_regB            = {26'd0, program_counter};
                        pc_bias_en          = 1'b1;
                        pc_bias             = alu_out[7:0];
                    end 
                    bz: begin
                        if(regC_out_data == 34'd0) begin
                            alu_ctrl            = ADD;
                            sum                 = regB_out_data + imm_extension;
                            alu_regA            = sum[33:0];                     
                            alu_regB            = {26'd0, program_counter};  //pc_instruction has 8 bits + 26 bits = 34 bits (data width)
                            pc_bias_en          = 1'b1;
                            pc_bias             = alu_out[7:0];
                        end
                    end
                    bnz: begin
                        if(regC_out_data != 34'd0) begin
                            alu_ctrl            = ADD;
                            sum                 = regB_out_data + imm_extension;
                            alu_regA            = sum[33:0];                     
                            alu_regB            = {26'd0, program_counter};  //pc_instruction has 8 bits + 26 bits = 34 bits (data width)
                            pc_bias_en          = 1'b1;
                            pc_bias             = alu_out[7:0];
                        end
                    end
                    bc: begin
                        if(reg_C_carry == 1'b1) begin
                            alu_ctrl            = ADD;
                            sum                 = regB_out_data + imm_extension;
                            alu_regA            = sum[33:0];                     
                            alu_regB            = {26'd0, program_counter};  //pc_instruction has 8 bits + 26 bits = 34 bits (data width)
                            pc_bias_en          = 1'b1;
                            pc_bias             = alu_out[7:0];
                        end
                    end
                    bv: begin
                        if(reg_C_overflow == 1'b1) begin
                            alu_ctrl            = ADD;
                            sum                 = regB_out_data + imm_extension;
                            alu_regA            = sum[33:0];                     
                            alu_regB            = {26'd0, program_counter};  //pc_instruction has 8 bits + 26 bits = 34 bits (data width)
                            pc_bias_en          = 1'b1;
                            pc_bias             = alu_out[7:0];
                        end
                    end
                    jal: begin
                        alu_ctrl            = ADD;
                        alu_regA            = 34'd4;                     
                        alu_regB            = {26'd0, program_counter};  //pc_instruction has 8 bits + 26 bits = 34 bits (data width)
                        regA_in_data        = {2'b00, alu_out[31:0]};
                        regA_wen            = 1'b1;

                        sum                 = imm_rc_extension + program_counter;
                        pc_bias             = sum[7:0];
                        pc_bias_en          = 1'b1;
                    end
                    jral: begin
                        alu_ctrl            = ADD;
                        alu_regA            = 34'd4;                     
                        alu_regB            = {26'd0, program_counter};  //pc_instruction has 8 bits + 26 bits = 34 bits (data width)
                        regA_in_data        = {2'b00, alu_out[31:0]};
                        regA_wen            = 1'b1;

                        sum                 = imm_rc_extension + regB_out_data + program_counter;
                        pc_bias             = sum[7:0];
                        pc_bias_en          = 1'b1;
                    end
                    ret: begin
                        pc_bias_en          = 1'b1;
                        pc_bias             = regB_out_data[7:0];
                    end
                endcase
            end
        endcase
    end
 

    ////////////////////////////////////////////////////////////////
    /////          DEBUG OUTPUTS ASSIGN                /////////////
    ////////////////////////////////////////////////////////////////
    assign debug_regB               =  regB_out_data;              //output  wire [33:0]
    assign debug_regC               =  regC_out_data;              //output  wire [33:0]
    assign debug_write_regA         =  regA_in_data;        //output  wire [33:0]
    assign debug_alu_out            =  alu_out;           //output  wire [33:0]
    assign debug_pc_instruction     =  pc_instruction;    //output  wire [31:0]
    assign debug_pc_out             =  program_counter;                //output  wire [7:0]
    assign debug_pc_input           =  pc_bias;                //output  wire [7:0]
    assign debug_mem_output_data    =  mem_output_data;   //output  wire [31:0]
    assign debug_mem_input_data     =  mem_input_data;     //output  wire [31:0]

endmodule //
