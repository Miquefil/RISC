//////////////////////////////////////////////////////////////////////
// Module Name: top_multicycle
//              pipeline for RISC
//
// Author: Miqueas Filsinger
// Date: 15-07-2024
//
// Revision History:
// - Date: Description of changes made
// - Date: Description of changes made
//
//////////////////////////////////////////////////////////////////////

`include "defines.v"
`include "alu_multicycle.v"
`include "memory.v"
`include "register_file_multi.v"
`include "program_counter_multi.v"

module top_multicycle #(
    parameter MEMORY_FILE   = "C:/Users/mique/OneDrive/Universidad/10mo Cuatri/PPS - Allegro/Trabajo Practico/RISC/rtl/multicycle/mem.hex",
    parameter PC_FILE       = "C:/Users/mique/OneDrive/Universidad/10mo Cuatri/PPS - Allegro/Trabajo Practico/RISC/rtl/multicycle/pc_mem_target_multi.hex",
    parameter REG_FILE      = "C:/Users/mique/OneDrive/Universidad/10mo Cuatri/PPS - Allegro/Trabajo Practico/RISC/rtl/multicycle/register_mem.hex"
    )
    (
    input   wire                            clk,
    input   wire                            rst_n,

    //Debug outputs
    //fetch
    output  wire [31:0]                     debug_pc_instruction,
    output  wire [7:0]                      debug_pc_out,
    output  wire [7:0]                      debug_pc_input,
    //decode
    output  wire [31:0]                     debug_dec_instruction,
    output  wire [33:0]                     debug_dec_regB,
    output  wire [33:0]                     debug_dec_regC,
    output  wire [33:0]                     debug_dec_write_regA,
    //execute 
    output  wire [31:0]                     debug_exec_instruction,
    output  wire [33:0]                     debug_exec_alu_regA,
    output  wire [33:0]                     debug_exec_alu_regB,
    output  wire [33:0]                     debug_exec_alu_out,
    //data mem
    output  wire [31:0]                     debug_mem_instruction,
    output  wire [7:0]                      debug_mem_address,
    output  wire [31:0]                     debug_mem_output_data,
    output  wire [31:0]                     debug_mem_input_data
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

    //NOP instruction
    parameter NOP = {2'b00, 3'b111, 27'd0};

    //ALU operations:
    parameter ADD       = 3'd0;
    parameter SUB       = 3'd1;
    parameter ADDC      = 3'd2;
    parameter SUBC      = 3'd3;
    parameter ALU_OR    = 3'd4;
    parameter ALU_AND   = 3'd5;
    parameter ALU_INV   = 3'd6;
    parameter EXCEPTION = 3'd7;
    

    ////////////////////////////////////////////////////////////////
    ////                 FETCH STAGE                     ///////////
    ////////////////////////////////////////////////////////////////
    wire    [31:0]                          pc_instruction;
    //Fetch Register - Flush and Enable     

    reg                                    	pc_en;
    reg                                    	pc_bias_en;
	reg     [7:0]                           pc_bias;
    wire    [7:0]                           program_counter;
    program_counter_multi #(
        .MEM_INIT_FILE  (PC_FILE)
    )
    u_pc (
        .clk                    (clk),
        .pc_en                  (pc_en),
        // .pc_en                  (1'b1),
        .bias_en                (pc_bias_en),
        .bias                   (pc_bias),
        .instruction_output     (pc_instruction),
        .program_counter        (program_counter)
    );


    ////////////////////////////////////////////////////////////////
    ////////                DECODE STAGE                  //////////
    ////////////////////////////////////////////////////////////////
    reg     [`REGISTERS_AD_WIDTH-1:0]       regA_ad;
    reg                                     regA_wen;
    reg     [`REGISTERS_AD_WIDTH-1:0]       regB_ad;
    reg     [`REGISTERS_AD_WIDTH-1:0]       regC_ad;
    reg     [`REGISTERS_WIDTH-1:0]          regA_in_data;
    wire    [`REGISTERS_WIDTH-1:0]          regB_out_data;
    wire    [`REGISTERS_WIDTH-1:0]          regC_out_data;  
    
    wire                                     reg_C_carry;
  	assign  reg_C_carry = regC_out_data[`REGISTERS_WIDTH-2];

    wire                                     reg_C_overflow;
  	assign  reg_C_overflow = regC_out_data[`REGISTERS_WIDTH-1];

    //Negedge write register file
    register_file_multi #(
        .REG_WIDTH      (34),
        .ADDR_WIDTH     (5),
        .MEM_INIT_FILE  (REG_FILE)
    ) 
    u_register(
        .clk                        (clk),                  //input   wire
        .rst_n                      (rst_n),
        .i_address_reg_a            (regA_ad),              //input[5]    wire  <- Writing register
        .i_wenable_reg_a            (regA_wen),             //input[1]    wire
        .i_writedata_reg_a          (regA_in_data),         //input[34]   wire
        .i_address_reg_b            (regB_ad),              //input[5]    wire
        .i_address_reg_c            (regC_ad),              //input[5]    wire
        .o_data_regb                (regB_out_data),        //output[34]  wire
        .o_data_regc                (regC_out_data)         //output[34]  wire
    );

    //>>>----------------- Execute Register ----------------------<<<
    reg   [7:0]                     dec_pc;
    reg   [31:0]                    dec_instruction; 
    reg                             rst_decode;
    wire  [1:0]             dec_ins_T;
    wire  [2:0]             dec_ins_OPC;
    wire  [11:0]            dec_ins_Imm;
    wire  [4:0]             dec_ins_RA_addr;
    wire  [4:0]             dec_ins_RB_addr;
    wire  [4:0]             dec_ins_RC_addr;
    assign  dec_ins_T               = dec_instruction[31:30];
    assign  dec_ins_OPC             = dec_instruction[29:27];
    assign  dec_ins_Imm             = dec_instruction[26:15];
    assign  dec_ins_RC_addr         = dec_instruction[14:10];
    assign  dec_ins_RB_addr         = dec_instruction[9:5];
    assign  dec_ins_RA_addr         = dec_instruction[4:0];
    
    ///Instruction Immediate comes from registered pc instruction

    
    reg                                     en_decode; 

    wire[11:0]          dec_instruction_Imm;
    assign              dec_instruction_Imm = dec_instruction[26:15];
    
    //-----------------instruction in Decode stage
    //sign extension of imm register
    wire [31:0]          dec_imm_extension;
    assign dec_imm_extension = (dec_instruction_Imm[11])?{1'b1,{19{1'b0}}, {dec_instruction_Imm}}
                                                        :{1'b0,{19{1'b0}}, {dec_instruction_Imm}};

    //sign extension of the concatenation of imm and rc register                                                
    wire [31:0]          dec_imm_rc_extension;
    assign dec_imm_rc_extension = (dec_instruction_Imm[11])?{1'b1, {14{1'b0}}, {dec_ins_Imm}, {dec_ins_RC_addr}}
                                                           :{1'b0, {14{1'b0}}, {dec_ins_Imm}, {dec_ins_RC_addr}};

    
    

    ////////////////////////////////////////////////////////////////
    ////////            EXECUTION STAGE                  ///////////
    ////////////////////////////////////////////////////////////////
    reg     [2:0]                       alu_ctrl;
    reg     [`NB_REGISTERS-1:0]         alu_regA;
    reg     [`NB_REGISTERS-1:0]         alu_regB;
    wire    [`NB_REGISTERS-1:0]         alu_out;

    alu_multicycle #(
        .NB_REGISTERS   (`NB_REGISTERS),       //34 bits normally
        .DATA_WIDTH     (`DATA_WIDTH)          //32 bits
    ) 
    u_alu (
        .i_alu_ctrl         (alu_ctrl),     //input   wire [0:2]              
        .i_data_A           (alu_regA),     //input   wire [0:NB_REGISTERS-1] 
        .i_data_B           (alu_regB),     //input   wire [0:NB_REGISTERS-1] 
        .o_data             (alu_out)       //output  wire [0:NB_REGISTERS-1] 
    );

    //-------- Memory Register
    //
    reg                                     rst_execute;
    //---two flags to know if we need to sum imm or imm_rc in exec stage
    reg                                     exec_sum_imm;
    reg                                     exec_sum_imm_rc;
    //-------- Instruction in execute stage
    reg  [31:0]                             exec_imm_extension;
    reg  [31:0]                             exec_imm_rc_extension;
    reg  [33:0]                             exec_regB;   
    reg  [33:0]                             exec_regC; 
    reg  [31:0]                             exec_instruction;  
    reg [7:0]                               exec_pc;  

    wire  [1:0]             exec_ins_T;
    wire  [2:0]             exec_ins_OPC;
    wire  [11:0]            exec_ins_Imm;
    wire  [4:0]             exec_ins_RA_addr;
    wire  [4:0]             exec_ins_RB_addr;
    wire  [4:0]             exec_ins_RC_addr;
    assign  exec_ins_T               = exec_instruction[31:30];
    assign  exec_ins_OPC             = exec_instruction[29:27];
    assign  exec_ins_Imm             = exec_instruction[26:15];
    assign  exec_ins_RC_addr         = exec_instruction[14:10];
    assign  exec_ins_RB_addr         = exec_instruction[9:5];
    assign  exec_ins_RA_addr         = exec_instruction[4:0];


    ////////////////////////////////////////////////////////////////
    ////////                MEMORY STAGE                  //////////
    ////////////////////////////////////////////////////////////////
    wire                                    mem_rst_n;
    reg                                     mem_wen;
    reg  [7:0]                              mem_addr;
    reg  [31:0]                             mem_input_data;
    wire [31:0]                             mem_output_data;
    memory  #(
        .MEM_WIDTH      (8),
        .DATA_WIDTH     (32),
      	.ADDR_WIDTH     (8),
        .MEM_INIT_FILE  (MEMORY_FILE)
    )
    u_mem(
            .clk            (clk),                  //input   wire
            .rst_n          (1'b0),            //input   wire
            .i_wenable      (mem_wen),              //input   wire
            .i_address      (mem_addr),             //input   wire
            .i_data         (mem_input_data),       //input   wire
            .o_data         (mem_output_data)       //output  wire
    );

    //------ Memory Register
    //mem register also keeps the alu out
    reg [33:0]                              mem_alu;
    reg [33:0]                              mem_regC;
    reg [31:0]                              mem_dec_immrc;
    reg                                     mem_write_to_reg;
    reg [7:0]                               mem_pc;  

    //-------- Instruction in mem stage
    reg   [31:0]            mem_instruction;

    wire  [1:0]             mem_ins_T;
    wire  [2:0]             mem_ins_OPC;
    wire  [11:0]            mem_ins_Imm;
    wire  [4:0]             mem_ins_RA_addr;
    wire  [4:0]             mem_ins_RB_addr;
    wire  [4:0]             mem_ins_RC_addr;
    assign  mem_ins_T               = mem_instruction[31:30];
    assign  mem_ins_OPC             = mem_instruction[29:27];
    assign  mem_ins_Imm             = mem_instruction[26:15];
    assign  mem_ins_RC_addr         = mem_instruction[14:10];
    assign  mem_ins_RB_addr         = mem_instruction[9:5];
    assign  mem_ins_RA_addr         = mem_instruction[4:0];




    ////////////////////////////////////////////////////////////////
    ////////            WRITE BACK STAGE                  //////////
    ////////////////////////////////////////////////////////////////
    reg [33:0]                  wb_alu;
    reg [31:0]                  wb_mem_in;
    reg [33:0]                  wb_output;
    reg [7:0]                   wb_pc;

    //------> THREE FLAGS:
    // wb_from_mem: assert high if instruction is meant to write from memory data
    // wb_to_reg:   assert high if regA write intended
    reg                     wb_from_mem;
    reg                     wb_to_reg;
    reg                     wb_storei;

    
    //-------- Instruction in WB stage
    //sign extension of the concatenation of imm and rc register                                                
    reg   [31:0]            wb_instruction;

    wire  [1:0]                 wb_ins_T;
    wire  [2:0]                 wb_ins_OPC;
    wire  [11:0]                wb_ins_Imm;
    wire  [4:0]                 wb_ins_RA_addr;
    wire  [4:0]                 wb_ins_RB_addr;
    wire  [4:0]                 wb_ins_RC_addr;
    assign  wb_ins_T               = wb_instruction[31:30];
    assign  wb_ins_OPC             = wb_instruction[29:27];
    assign  wb_ins_Imm             = wb_instruction[26:15];
    assign  wb_ins_RC_addr         = wb_instruction[14:10];
    assign  wb_ins_RB_addr         = wb_instruction[9:5];
    assign  wb_ins_RA_addr         = wb_instruction[4:0];

    wire [31:0]          wb_imm_rc_extension;
    assign wb_imm_rc_extension = (wb_ins_Imm[11])?{1'b1, {14{1'b0}}, {wb_ins_Imm}, {wb_ins_RC_addr}}
                                                :{1'b0, {14{1'b0}}, {wb_ins_Imm}, {wb_ins_RC_addr}};


    ////////////////////////////////////////////////////////
    ///////         REGISTERS FOR EVERY STAGE       ////////
    
    always @(posedge clk) begin
        //-------> Starting DigTop
        if(!rst_n) begin
            //
        end
        
        //-------> Decode Register ------------
        if (rst_decode) begin
            dec_instruction     <= NOP;
            dec_pc              <= 8'd0;
        end
        else if(en_decode) begin
            //enable when not STALL HAZARD
            dec_instruction     <= pc_instruction;  
            dec_pc              <= program_counter;  
        end
        
        //-------> Execution Register ---------
        if (rst_execute) begin
            exec_regB              <= 34'd0;
            exec_regC              <= 34'd0;
            exec_imm_extension     <= 32'd0;
            exec_imm_rc_extension  <= 32'd0;    
            //NOP instruction added
            exec_instruction       <= NOP;
            exec_pc                <= 8'd0;
        end
        else begin
            exec_regB              <= regB_out_data;
            exec_regC              <= regC_out_data;
            exec_imm_extension     <= dec_imm_extension;
            exec_imm_rc_extension  <= dec_imm_rc_extension;
            exec_instruction    <= dec_instruction;
            exec_pc             <= dec_pc;
        end
        
        //-------> Memory Stage -------------
        mem_regC        <= exec_regC;
        mem_dec_immrc   <= exec_imm_rc_extension;
        mem_alu         <= alu_out;
        mem_instruction <= exec_instruction;
        mem_pc          <= exec_pc;
        
        //-------> WB Stage -----------------
        wb_instruction    <= mem_instruction;
        wb_alu            <= mem_alu;
        wb_mem_in         <= mem_output_data;
        wb_pc             <= mem_pc;

    end
    ///////                                     ///////
    ///////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////
    ////////                HAZARD UNIT                   //////////
    ////////////////////////////////////////////////////////////////
    reg             hazard_stall;
    reg             hazard_stall_regB;
    reg             hazard_stall_regC;
    reg             hazard_forward_mem_regB;   
    reg             hazard_forward_mem_regC;   
    reg             hazard_forward_wb_regB;   
    reg             hazard_forward_wb_regC;   
   
    //TODO:  CHECK FLUSH --- if flush clears the whole ExecReg then maybe 00..00 may perform
    //an undisired action
    
    reg             branch_control_detected;
    always @(*) begin
        ////////////////////-----RESET STAGE-----/////////////////////
        //very important stage, as any flag has values, then structural states
        //also doesnt have a state.
        if(!rst_n) begin
            pc_en       = 1'b1;     // NO HAZARDS --> Program Counter Enabled
            en_decode   = 1'b1;     // NO HAZARDS --> Fetch Register Enabled
            rst_execute = 1'b0;
            mem_write_to_reg = 1'b0;
            hazard_forward_mem_regB         = 1'b0;
            hazard_forward_mem_regC         = 1'b0;
            hazard_forward_wb_regB          = 1'b0;
            hazard_forward_wb_regC          = 1'b0;
            hazard_stall                    = 1'b0;
            hazard_stall_regB               = 1'b0;
            hazard_stall_regC               = 1'b0;
        end

        ////////////////////---END RESET STAGE--/////////////////////


        pc_en       = 1'b1;     // NO HAZARDS --> Program Counter Enabled
        en_decode   = 1'b1;     // NO HAZARDS --> Fetch Register Enabled
        rst_execute = 1'b0;
        rst_decode  = 1'b0;

        ///////////////     FORWARDING      ///////////////////////////
        //
        //FORWARDING: When one source register of the execute stage
        //has the same address that a writing register of Memory or Writeback
        //stage
        //
        //IMPORTANT FOR HAZARD: mem_write_to_reg asserts high when instruction
        //in mem stage intends to write to a reg- IMPORTANT TO NOT INCLUDE 'LOAD' OPTION
        mem_write_to_reg = 1'b0;
        case (mem_ins_T)
            MOVEMENT: begin
                case (mem_ins_OPC) 
                    mov:       mem_write_to_reg    = 1'b1;
                endcase
            end
            LOGIC:             mem_write_to_reg    = 1'b1;

            ARITHMETIC:        mem_write_to_reg    = 1'b1;

            FLOWCTRL: begin
                case (mem_ins_OPC)
                    jal:       mem_write_to_reg    = 1'b1;
                    jral:      mem_write_to_reg    = 1'b1;                
                endcase
            end
        endcase

        //----- IMPORTANT NOTE!: Forwarding is done when either in Mem or WB stage
        //destination write reg is the same as source reg in Exec stage, 
        //BUT IT CANNOT SOLVE 'LOAD FROM MEMORY' IN MEM STAGE. ALTHOUGH IT CAN
        //SOLVE 'LOAD FROM MEMORY' IN WB STAGE.
        // ---- IMPORTANT: Precedence of Memory FORWARD HAZARD over
        //              WB Stage FOWRWARD Hazard
        hazard_forward_mem_regB         = 1'b0;
        hazard_forward_mem_regC         = 1'b0;
        hazard_forward_wb_regB          = 1'b0;
        hazard_forward_wb_regC          = 1'b0;
        if (mem_write_to_reg) begin  //Memory Stage WRITE INTENDED
            //mem_write_to_reg ignores LDR instruction!!!
            
            if (exec_ins_RB_addr == mem_ins_RA_addr) begin
                //if execute reg B should be the written one    
                //wb_ins_RA_addr : address of intended write
                    hazard_forward_mem_regB = 1'b1;
                end
            if (exec_ins_RC_addr == mem_ins_RA_addr) begin 
                //if execute reg C should be the written one 
                //wb_ins_RA_addr : address of intended write   
                    hazard_forward_mem_regC = 1'b1;
                end
            end
        else begin
            if(wb_to_reg) begin //WriteBack FORWARDING HAZARD 
                if (exec_ins_RB_addr == wb_ins_RA_addr) begin
                    //if exec_reg B should be the written one 
                    //wb_ins_RA_addr : address of intended write   
                    hazard_forward_wb_regB = 1'b1;
                end
                if (exec_ins_RC_addr == wb_ins_RA_addr) begin 
                    //if exec_reg C should be the written one   
                    //wb_ins_RA_addr : address of intended write 
                    hazard_forward_wb_regC = 1'b1;
                end
            end
        end
        ///////////////     END OF FORWARDING      ///////////////////////////

        ////////////////////////     STALL      //////////////////////////////
        //STALL: if we need to load from memory to register (LDR) and the write address
        //matches the source address of either one of the registers in decode stage,
        //the processor should STALL
        hazard_stall                    = 1'b0;
        hazard_stall_regB               = 1'b0;
        hazard_stall_regC               = 1'b0;
        if(
            (exec_ins_T == 2'b00) && (exec_ins_OPC == 3'b000) //LOAD FROM MEMORY INSTRUCTION
        )
        begin
            if (exec_ins_RA_addr == dec_ins_RB_addr) begin
                hazard_stall_regB = 1'b1;
            end
            if (exec_ins_RA_addr == dec_ins_RC_addr) begin
                hazard_stall_regC = 1'b1;
            end
        end

        if(hazard_stall_regB||hazard_stall_regC) begin
            hazard_stall        = 1'b1;
            //Program Counter Stall
            pc_en               = 1'b0;
            en_decode           = 1'b0;
            rst_execute         = 1'b1;
        end 
        /////////////////////     END OF STALL      //////////////////////////
        //
        //
        //
        /////////////////////     BRANCH CONTROl      ///////////////////////
        //branch_control_detected flag asserts high when branch control instructions
        //are detected ON EXECUTE STAGE
        branch_control_detected     = 1'b0;
        if(exec_ins_T == FLOWCTRL) begin
            branch_control_detected = 1'b1;
        end

        if(branch_control_detected && pc_bias_en)
        //if control instruction detected & the control is valid 
        //(pc_bian_en asserted high if the instruction is correcto, ie Rc.C == 0)
        begin
            hazard_stall    = 1'b1;
            rst_decode      = 1'b1;     //flush decode
            rst_execute     = 1'b1;     //flush exec
        end

        //////////////////    END OF BRANCH CONTROl      ////////////////////
    end


    ////////////////////////////////////////////////////////////////
    ////////                CONTROL UNIT                   /////////
    ////////////////////////////////////////////////////////////////
    
    
        //////////// Fetch Stage ///////////
        /////////////////////////////////////
    always @(*) begin

        //-----> Flow Control 
        if ((exec_ins_T == FLOWCTRL) && (exec_ins_OPC==ret)) begin
            pc_bias     = alu_out;  //For Branch Control
        end
        else begin
            pc_bias     = alu_out + exec_pc;  //For Branch Control
        end
        pc_bias_en = 1'b0;

        if((exec_ins_T == FLOWCTRL)) begin
            case (exec_ins_OPC)
                jump :  pc_bias_en = 1'b1;
                bz: begin
                    if(exec_regC == 34'd0) begin    //regC.data == 0
                        pc_bias_en = 1'b1;
                    end
                end
                bnz: begin
                    if(exec_regC != 34'd0) begin    //regC.data != 0
                        pc_bias_en = 1'b1;
                    end
                end
                bc: begin
                    if(exec_regC[32] == 1'b0) begin //regC carry == 0
                        pc_bias_en = 1'b1;
                    end
                end
                bv: begin
                    if(exec_regC[33] == 1'b0) begin //reg ov == 0
                        pc_bias_en = 1'b1;
                    end
                end
                jal:    pc_bias_en = 1'b1;
                jral:   pc_bias_en = 1'b1;
                ret:    pc_bias_en = 1'b1;
            endcase
        end
    end
        

        //////////// Decode Stage ///////////
        /////////////////////////////////////
    always @(*) begin
        //regB and regC come from decode instruction
        regB_ad         = dec_ins_RB_addr;
        regC_ad         = dec_ins_RC_addr;

        //write regA comes from Write Back stage
        regA_in_data    = wb_output;        //WriteBack data channel
        regA_ad         = wb_ins_RA_addr;   //Write Direction comes from WB stage
        regA_wen        = wb_to_reg;        //Reg Write enable
    end



        //////////// Execute Stage //////////
        /////////////////////////////////////
    always @(*) begin
        //----> CONTROL! 
        exec_sum_imm        = 1'b0;
        exec_sum_imm_rc     = 1'b0;

        //imm control
        case (exec_ins_T)
            MOVEMENT: begin
                if(exec_ins_OPC == store) begin
                    exec_sum_imm = 1'b1;
                end
            end
            FLOWCTRL: begin
                if( (exec_ins_OPC == bz)||(exec_ins_OPC == bnz)||(exec_ins_OPC == bc)||(exec_ins_OPC == bv) ) begin
                    exec_sum_imm = 1'b1;
                end
            end            
        endcase

        //imm_rc control
        case (exec_ins_T)
            MOVEMENT: begin
                if((exec_ins_OPC == load)||(exec_ins_OPC == loadi))  begin
                    exec_sum_imm_rc = 1'b1;
                end
            end
            FLOWCTRL: begin
                if( (exec_ins_OPC == jal) || (exec_ins_OPC == jral) ) begin
                    exec_sum_imm_rc = 1'b1;
                end
            end            
        endcase

        ///////////////////
        //....ALU RegA MUX!
        alu_regA = exec_regB; //REGA of ALU from Reg File reg B

        if( ((exec_ins_T == MOVEMENT)&&(exec_ins_OPC == loadi)) 
            || ((exec_ins_T == FLOWCTRL)&&(exec_ins_OPC == jal))            
        ) begin 
                //if loadi or storei instructions, reg goes to zero
                alu_regA = 34'd0;
        end
        begin
            case ({hazard_forward_wb_regB, hazard_forward_mem_regB})        
                2'b01:  alu_regA = mem_alu;
                2'b10:  alu_regA = wb_output;
            endcase    
        end
    
        //////////////////
        //...ALU RegB MUX!
        alu_regB = exec_regC; //REGB zero --- instructions dont need sum

        if( ( (exec_ins_T == MOVEMENT) && (exec_ins_OPC == mov) ) ||
            ( (exec_ins_T == FLOWCTRL) && (exec_ins_OPC == ret) ) ||
            ( (exec_ins_T == MOVEMENT) && (exec_ins_OPC == storei) )) begin 
                //if ret or mov instructions, reg goes to zero
                alu_regB = 34'd0;
        end
        else if (exec_sum_imm) begin  //If immediate register needed
            //THIS PRECEDENCE IS IMPORTANT!
            //Hazards may arise when trying to use Imm*, as Hazard Unit
            //doesnt know if we're trying to load Rc or Imm*.
            alu_regB = exec_imm_extension;
        end
        else if(exec_sum_imm_rc) begin
            //THIS PRECEDENCE IS IMPORTANT!
            //Hazards may arise when trying to use Imm*, as Hazard Unit
            //doesnt know if we're trying to load Rc or Imm*.
            alu_regB = exec_imm_rc_extension;
        end
        else begin
            case ({hazard_forward_wb_regC, hazard_forward_mem_regC})        
                2'b01:  alu_regB = mem_alu;
                2'b10:  alu_regB = wb_output;
            endcase
        end

        //////////////////
        //...ALU CONTROL!
        alu_ctrl = EXCEPTION;
        case (exec_ins_T)
            MOVEMENT: alu_ctrl = ADD;
            
            LOGIC:  case (exec_ins_OPC)
                or_l:   alu_ctrl = ALU_OR;
                inv_l:  alu_ctrl = ALU_INV;
                and_l:  alu_ctrl = ALU_AND;
            endcase

            ARITHMETIC: case (exec_ins_OPC)
                add_a:  alu_ctrl = ADD;
                sub_a:  alu_ctrl = SUB;
                addc_a: alu_ctrl = ADDC;
                subc_a: alu_ctrl = SUBC;
            endcase

            FLOWCTRL:   alu_ctrl = ADD;
        endcase

    end



        /////////   Memory Stage   ///////////
        /////////////////////////////////////
    always @(*) begin
        mem_addr        = mem_alu[7:0];
        mem_wen = 1'b0;

        //In all movement instructions where memory is going to be written
        //there are 3 different cases:
        //1. by default the input should be regC (OPC = store)
        //2. if wb stage is going to write in the same address as regC
        //      then check it. If not, input can be come directly from reg file
        //3. in storei operation, mem input should be Imm*
        //Thus, the following is implemented:
        mem_input_data = regC_out_data[31:0];

        if(mem_ins_T == MOVEMENT) begin
            mem_wen         = 1'b1;
            if(mem_ins_OPC==store)    begin
                if(wb_to_reg) begin
                    //IF wb stage wants to write a reg, check if its the
                    //one we need
                    if(wb_ins_RA_addr == mem_ins_RC_addr) begin
                        mem_input_data  = wb_output[31:0];
                    end
                    else begin
                        mem_input_data  = mem_regC[31:0];
                    end
                end
            end
            else if(mem_ins_OPC==storei) begin
                mem_input_data  = mem_dec_immrc;
                mem_wen         = 1'b1;
            end
        end
        
    end
    
    
        ///////// WriteBack Stage ///////////
        /////////////////////////////////////
    reg                 wb_pc_to_reg;
    always @(*) begin
        //------> CONTROL OF THE THREE FLAGS
        wb_from_mem     = 1'b0;
        wb_to_reg       = 1'b0;
        wb_storei       = 1'b0;
        wb_pc_to_reg    = 1'b0;

        case (wb_ins_T)
            MOVEMENT: begin
                case (wb_ins_OPC)
                    load: begin
                                wb_to_reg   = 1'b1;  
                                wb_from_mem = 1'b1;
                    end
                    loadi:      wb_to_reg   = 1'b1;
                    storei:    wb_storei   = 1'b1;
                    mov:       wb_to_reg   = 1'b1;
                endcase
            end
            
            LOGIC:             wb_to_reg = 1'b1;

            ARITHMETIC:        wb_to_reg = 1'b1;

            FLOWCTRL: begin
                case (wb_ins_OPC)
                    jal:       begin
                        wb_pc_to_reg = 1'b1;
                        wb_to_reg    = 1'b1; //wen enable in regfile
                    end
                    jral:   begin
                        wb_pc_to_reg = 1'b1;                
                        wb_to_reg    = 1'b1; //wen enable in regfile
                    end
                endcase
            end
            
        endcase

        //------>CONTROL OF THE OUTPUT
        wb_output = wb_alu;
        if (wb_from_mem) begin
            wb_output = {2'b00, wb_mem_in};
        end
        else if (wb_storei) begin  //JUST IN CASE OF EXECUTING STOREI instruction!!
            wb_output = wb_imm_rc_extension;
        end
        else if (wb_pc_to_reg) begin
            wb_output = wb_pc + 8'd1;   //PC <--- PC + 4
        end
    end
 

    ////////////////////////////////////////////////////////////////
    /////          DEBUG OUTPUTS ASSIGN                /////////////
    ////////////////////////////////////////////////////////////////
    //fetch
    assign  debug_pc_instruction    = pc_instruction;           //output  wire [31:0]
    assign  debug_pc_out            = program_counter;          //output  wire [7:0] 
    assign  debug_pc_input          = pc_bias;                  //output  wire [7:0] 
    //decode
    assign  debug_dec_instruction   = dec_instruction;          //output  wire [31:0]
    assign  debug_dec_regB          = regB_out_data;            //output  wire [33:0]
    assign  debug_dec_regC          = regC_out_data;            //output  wire [33:0]
    assign  debug_dec_write_regA    = regA_in_data;             //output  wire [33:0]
    //execute 
    assign  debug_exec_instruction  = exec_instruction;         //output  wire [31:0]
    assign  debug_exec_alu_regA     = alu_regA;                 //output  wire [33:0]
    assign  debug_exec_alu_regB     = alu_regB;                 //output  wire [33:0]
    assign  debug_exec_alu_out      = alu_out;                  //output  wire [33:0]
    //data mem
    assign  debug_mem_instruction   = mem_instruction;          //output  wire [31:0]
    assign  debug_mem_address       = mem_addr;                 //output  wire [33:0]                     
    assign  debug_mem_output_data   = mem_output_data;          //output  wire [31:0]                     
    assign  debug_mem_input_data    = mem_input_data;           //output  wire [31:0]   
endmodule //