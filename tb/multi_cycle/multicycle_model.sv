//////////////////////////////////////////////////////////////////////
// Module Name: multicycle_model
//              - model to predict the behaviour of Pipeline RISC
//
// Author: Miqueas Filsinger
// Date: 20-07-2024
//
//
//////////////////////////////////////////////////////////////////////

module multicycle_model #(
    parameter   MEMORY_FILE = "mem.hex",
    parameter   PC_FILE     = "pc_mem_RANDOM.hex",
    parameter   REG_FILE    = "register_mem.hex"
)
(
    input   wire                            clk,
    input   wire                            rst_n,
    //fetch
    output  wire [31:0]                     model_pc_instruction,
    output  wire [7:0]                      model_pc_out,
    output  wire [7:0]                      model_pc_input,
    //decode
    output  wire [31:0]                     model_dec_instruction,
    output  wire [33:0]                     model_dec_regB,
    output  wire [33:0]                     model_dec_regC,
    output  wire [33:0]                     model_dec_write_regA,
    //execute 
    output  wire [31:0]                     model_exec_instruction,
    output  wire [33:0]                     model_exec_alu_regA,
    output  wire [33:0]                     model_exec_alu_regB,
    output  wire [33:0]                     model_exec_alu_out,
    //data mem
    output  wire [31:0]                     model_mem_instruction,
    output  wire [7:0]                      model_mem_address,
    output  wire [31:0]                     model_mem_output_data,
    output  wire [31:0]                     model_mem_input_data,

    //debug
    output  wire                            debug_mem_to_reg,
    output  wire                            hazard_A,
    output  wire                            hazard_B,
    output  wire                            debug_stall,
    output  wire                            debug_rst_decode,
    output  wire                            debug_rst_execute,
    output  wire                            debug_wb_wen,
    output  wire [33:0]                     debug_wb_a,
    output  wire [31:0]                     model_wb_instruction,
    output  wire [33:0]                     debug_mem_alu

);

    reg [31:0] NOP = {2'b00, 3'b111, 27'd0};

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

    /////////////////////////////////////////////////////////////////////
    //////////////////////     FETCH STAGE    ///////////////////////
    /////////////////////////////////////////////////////////////////////

    ///////////     PROGRAM COUNTER MEMORY DUPLICATE    //////////////
  	reg  [7:0]  tb_pc_mem [0:1023];

    initial begin
        if (PC_FILE != "") begin
          	$readmemh(PC_FILE, tb_pc_mem, 0, 1023);
        end
    end 

    //tb_instruction is the instruction, from a duplicate memory program counter
    wire [31:0]     tb_instruction;
    reg  [7:0]      pc = 8'd0;
    assign tb_instruction = {tb_pc_mem[4*pc], tb_pc_mem[4*pc+8'd1], tb_pc_mem[4*pc+8'd2], tb_pc_mem[4*pc+8'd3]};
    ///////////--------------MEM------------------//////////////

    reg                     branch_control_detected = 1'b0;
    reg fetch_enable        = 1'b1;
    reg [7:0]               pc_bias;
    reg                     pc_bias_en = 1'b0;
    
    
    always @(posedge clk) begin
        if(pc_bias_en) begin
            pc <= pc_bias;
        end
        else if (fetch_enable) begin
            pc <= pc + 8'd1;
        end
    end



	//////////////////////////////////////////////////////////////////
    ///////////             DECODE STAGE                   ////////////
    /////////////////////////////////////////////////////////////////
    reg  [33:0]  reg_mem [0:1023];

    initial begin
        if (REG_FILE != "") begin
          	$readmemh(REG_FILE, reg_mem, 0, 31);
        end
    end 

    reg [31:0]      dec_instruction;
    reg [33:0]      dec_regB;
    reg [33:0]      dec_regC;
    reg [7:0]       dec_PC;
    reg             dec_en = 1'b1;
    reg             flush_decode;

    reg [33:0]          dec_write_data;
    reg [4:0]           dec_write_address;
    reg                 regA_wen = 1'b0;

    wire  [1:0]                         dec_ins_T;
    wire  [2:0]                         dec_ins_OPC;
    wire  [11:0]                        dec_ins_Imm;
    wire  [4:0]                         dec_ins_RA_addr;
    wire  [4:0]                         dec_ins_RB_addr;
    wire  [4:0]                         dec_ins_RC_addr;
    wire  [31:0]                        dec_imm_extension;
    wire  [31:0]                        dec_imm_rc_extension;
    assign  dec_ins_T               =  dec_instruction[31:30];
    assign  dec_ins_OPC             =  dec_instruction[29:27];
    assign  dec_ins_Imm             =  dec_instruction[26:15];
    assign  dec_ins_RC_addr         =  dec_instruction[14:10];
    assign  dec_ins_RB_addr         =  dec_instruction[9:5];
    assign  dec_ins_RA_addr         =  dec_instruction[4:0];

    assign dec_imm_extension = (dec_ins_Imm[11])?{1'b1,{19{1'b0}}, {dec_ins_Imm}}
                                                        :{1'b0,{19{1'b0}}, {dec_ins_Imm}};
    assign dec_imm_rc_extension = (dec_ins_Imm[11])?{1'b1, {14{1'b0}}, {dec_ins_Imm}, {dec_ins_RC_addr}}
                                                   :{1'b0, {14{1'b0}}, {dec_ins_Imm}, {dec_ins_RC_addr}};

    //OUTPUT REGISTERS
    always @(*) begin
        dec_regB = reg_mem[dec_ins_RB_addr];
        dec_regC = reg_mem[dec_ins_RC_addr];
    end

    //REGISTER FILE WRITING
    always @(negedge clk) begin
        if(regA_wen) begin
            reg_mem[dec_write_address] <= dec_write_data;
        end
    end

    always @(posedge clk) begin
        //DECODE REGISTER
        if(flush_decode) begin
            dec_instruction <= NOP;
            dec_PC          <= 8'd0;
        end 
        else if(dec_en) begin
            dec_instruction <= tb_instruction;
            dec_PC          <= pc;      //copying fetch pc
        end
    end

    //////////////////////////////////////////////////////////////////
    ///////////                 EXECUTION                 ////////////
    /////////////////////////////////////////////////////////////////
    reg     [7:0]           exec_PC;
    reg     [33:0]          forward_A;
    reg     [33:0]          forward_B;
    reg     [33:0]          exec_regB;
    reg     [33:0]          exec_regC;
    reg     [33:0]          exec_alu_out;
    reg     [33:0]          exec_alu_A;
    reg     [33:0]          exec_alu_B;
    reg                     exec_flush;
    reg     [31:0]          exec_imm_extension;
    reg     [31:0]          exec_imm_rc_extension;
    reg                     flush_execute;

    reg     [31:0]          exec_instruction;
    wire  [1:0]                         exec_ins_T;
    wire  [2:0]                         exec_ins_OPC;
    wire  [11:0]                        exec_ins_Imm;
    wire  [4:0]                         exec_ins_RA_addr;
    wire  [4:0]                         exec_ins_RB_addr;
    wire  [4:0]                         exec_ins_RC_addr;
    assign  exec_ins_T               =  exec_instruction[31:30];
    assign  exec_ins_OPC             =  exec_instruction[29:27];
    assign  exec_ins_Imm             =  exec_instruction[26:15];
    assign  exec_ins_RC_addr         =  exec_instruction[14:10];
    assign  exec_ins_RB_addr         =  exec_instruction[9:5];
    assign  exec_ins_RA_addr         =  exec_instruction[4:0];
    
    always @(posedge clk) begin
        if (flush_execute) begin
            exec_instruction        <= NOP;
            exec_PC                 <= 8'd0;
            exec_imm_extension      <= 31'd0;
            exec_imm_rc_extension   <= 31'd0;
            exec_regB               <= 34'd0;
            exec_regC               <= 34'd0;
        end
        else begin
            exec_instruction        <= dec_instruction;    
            exec_imm_extension      <= dec_imm_extension;
            exec_imm_rc_extension   <= dec_imm_rc_extension;
            exec_PC                 <= dec_PC; 
            exec_regB               <= dec_regB;
            exec_regC               <= dec_regC; 
        end
    end

    //////////////////////////////////////////////////////////////////
    ///////////             MEM STAGE                     ////////////
    /////////////////////////////////////////////////////////////////

    ///	we copy the same memory used in RISC to use it for comparison in 
  	///	movement operations
    // parameter MEM_INIT_FILE = MEMORY_FILE;

  	reg [7:0]     model_mem [0:1023];
    initial begin
        if (MEMORY_FILE != "") begin
          $readmemh(MEMORY_FILE, model_mem, 0, 1023);
        end
    end 
    reg             mem_wen;
    reg             mem_to_reg;
    reg [7:0]       mem_pc;
    reg [31:0]      mem_instruction;
    reg [31:0]      mem_alu;
    reg [31:0]      mem_dec_regC;

    wire [31:0]     mem_output;
    reg  [31:0]     mem_input;
    wire [7:0]      mem_addr;
    assign mem_addr = mem_alu[7:0];

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

    reg [31:0]          mem_immrc;
    assign mem_immrc =         (mem_ins_Imm[11])?{1'b1, {14{1'b0}}, {mem_ins_Imm}, {mem_ins_RC_addr}}
                                                :{1'b0, {14{1'b0}}, {mem_ins_Imm}, {mem_ins_RC_addr}};


    //---- output addressing
    reg [7:0]       addr_out;
    assign mem_output = {
                        {model_mem[addr_out*4]}, 
                        {model_mem[addr_out*4 + 8'd1]}, 
                        {model_mem[addr_out*4 + 8'd2]}, 
                        {model_mem[addr_out*4 + 8'd3]}
                        };

    //MEM STAGE register
    always @(posedge clk) begin
        mem_dec_regC    <= exec_regC[31:0];
        mem_alu         <= exec_alu_out;
        mem_instruction <= exec_instruction;
        mem_pc          <= exec_PC;

        //writing of memory
        if(mem_wen) begin
            model_mem[mem_addr*4]        <= mem_input[31:24];
            model_mem[mem_addr*4 + 8'd1] <= mem_input[23:16];
            model_mem[mem_addr*4 + 8'd2] <= mem_input[15:8];
            model_mem[mem_addr*4 + 8'd3] <= mem_input[7:0];
        end
    end

    //////////////////////////////////////////////////////////////////
    ///////////             WRITEBACK STAGE               ////////////
    /////////////////////////////////////////////////////////////////
    reg [33:0]                  wb_alu;
    reg [31:0]                  wb_mem_output;
    reg [33:0]                  wb_output;
    reg [7:0]                   wb_pc;
    reg [31:0]                  wb_instruction;
    reg                         wb_to_reg = 1'b0;
    
    always @(posedge clk ) begin
        wb_mem_output   <= mem_output;
        wb_pc           <= mem_pc;
        wb_instruction  <= mem_instruction;
        wb_alu          <= mem_alu;
    end

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


    //////////////////////////////////////////////////////////////////
    ///////////          HAZARD CONTROL                   ////////////
    /////////////////////////////////////////////////////////////////
    reg stall       = 1'b0;
    reg hazard_fw_A = 1'b0;
    reg hazard_fw_B = 1'b0;
    always @(*) begin
       //Forwarding Hazard
        hazard_fw_A = 1'b0;
        hazard_fw_B = 1'b0;

        forward_A = mem_alu;
        forward_B = mem_alu;
        if(mem_to_reg) 
        begin
            if(mem_ins_RA_addr == exec_ins_RB_addr) begin
                hazard_fw_A = 1'b1;
            end
            if(mem_ins_RA_addr == exec_ins_RC_addr) begin
                hazard_fw_B = 1'b1;
            end
        end
        else if(wb_to_reg) begin
           if(wb_ins_RA_addr == exec_ins_RB_addr) begin
                hazard_fw_A = 1'b1;
                forward_A   = wb_output;
            end
            if(wb_ins_RA_addr == exec_ins_RC_addr) begin
                hazard_fw_B = 1'b1;
                forward_B   = wb_output;
            end 
        end


        ////LOAD Hazard
        fetch_enable    = 1'b1;
        flush_execute   = 1'b0;
        dec_en          = 1'b1;
        stall           = 1'b0;
        if(
            (exec_ins_T == MOVEMENT) && (exec_ins_OPC == load) //LOAD FROM MEMORY INSTRUCTION
        )
        begin
            if((exec_ins_RA_addr == dec_ins_RB_addr) || (exec_ins_RA_addr == dec_ins_RC_addr))
            begin
                //if load is detected in execute stage and intends to write to a reg which will be used
                //in the following instruction
                stall           = 1'b1;
                fetch_enable    = 1'b0;
                dec_en          = 1'b0;
                flush_execute   = 1'b1; 
            end
        end

        branch_control_detected = 1'b0;
        flush_decode            = 1'b0;
        
        //// CONTROL HAZARD
        if(exec_ins_T == FLOWCTRL) begin
            branch_control_detected  = 1'b1;
            if(pc_bias_en) begin
                stall                   = 1'b1;
                flush_decode            = 1'b1;
                flush_execute           = 1'b1;
            end
            
        end

    end

    //////////////////////////////////////////////////////////////////
    ///////////          CONTROL UNIT                     ////////////
    /////////////////////////////////////////////////////////////////
    //----------> FETCH Control
    always @(*) begin
        pc_bias = 8'd0;
        if( (exec_ins_T == FLOWCTRL) && (exec_ins_OPC == ret) ) begin
            pc_bias = exec_alu_out;
        end 
        else if((exec_ins_T == FLOWCTRL))begin
            pc_bias = exec_alu_out + exec_PC;
        end

        pc_bias_en = 1'b0;
        if((exec_ins_T == FLOWCTRL)) begin
            case (exec_ins_OPC)
                jump :  pc_bias_en = 1'b1;
                bz: begin
                    if(exec_regC[31:0] == 32'd0) begin    //regC.data == 0
                        pc_bias_en = 1'b1;
                    end
                end
                bnz: begin
                    if(exec_regC[31:0] != 32'd0) begin    //regC.data != 0
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
    //----------> Decode Control
    always @(*) begin
        //write regA comes from Write Back stage
        dec_write_data      = wb_output;        //WriteBack data channel
        dec_write_address   = wb_ins_RA_addr;   //Write Direction comes from WB stage
        regA_wen            = wb_to_reg;        //Reg Write enable
    end

    //----------> Memory Stage
    always @(*) begin
        mem_to_reg = 1'b0;
        // if (  ((mem_ins_T == MOVEMENT) && (mem_ins_OPC == mov))
        //     || (mem_ins_T == LOGIC)    //LOGIC operations
        //     || (mem_ins_T == ARITHMETIC)     //arithmetic operations
        //     || ((mem_ins_T == FLOWCTRL)&&((mem_ins_OPC == jal)||(mem_ins_OPC == jral)))    //jal or jral
        // ) begin
        //     mem_to_reg = 1'b1;
        // end

        case (mem_ins_T)
            MOVEMENT: begin
                case (mem_ins_OPC) 
                    mov:       mem_to_reg    = 1'b1;
                endcase
            end
            LOGIC:             mem_to_reg    = 1'b1;

            ARITHMETIC:        mem_to_reg    = 1'b1;

            FLOWCTRL: begin
                case (mem_ins_OPC)
                    jal:       mem_to_reg    = 1'b1;
                    jral:      mem_to_reg    = 1'b1;                
                endcase
            end
        endcase
        // mem_wen         = 1'b0;
        // mem_input       = mem_dec_regC[31:0];
        // if(mem_ins_T == MOVEMENT) begin
        //     if(mem_ins_OPC==store)    begin
        //         mem_input       = mem_dec_regC[31:0];
        //         mem_wen         = 1'b1;
        //     end
        //     else if(mem_ins_OPC==storei) begin
        //         mem_input       = mem_immrc;
        //         mem_wen         = 1'b1;
        //     end
        // end

        //INPUT ADDRESSING------------------
        mem_wen         = 1'b0;
        mem_input       = mem_immrc;
        if(mem_ins_T == MOVEMENT) begin
            if(mem_ins_OPC==store)    begin
                mem_wen         = 1'b1;
                mem_input       = reg_mem[mem_ins_RC_addr];   //register C
                if ((wb_to_reg) && (wb_ins_RA_addr == mem_ins_RC_addr)) begin
                    mem_input       = wb_output;   //register C
                end
            end
            else if(mem_ins_OPC==storei) begin
                mem_input       = mem_immrc;
                mem_wen         = 1'b1;
            end
        end
        //INPUT ADDRESSING------------------

        //OUTPUT ADDRESSING------------------
        addr_out = 8'd0;
        if(mem_ins_RB_addr != wb_ins_RA_addr) begin
            //Checking not possible forwarding
            addr_out = add(reg_mem[mem_ins_RB_addr], mem_immrc);    
        end
        else begin
            addr_out = add(wb_output, mem_immrc); 
        end
        //OUTPUT ADDRESSING------------------
    end

    //----------> writback  Stage
    always @(*) begin
        wb_to_reg = 1'b0;           ///WB will write regA!
        if(
            (wb_ins_T == ARITHMETIC) || (wb_ins_T == LOGIC) 
            || ((wb_ins_T == MOVEMENT) && ((wb_ins_OPC == load)||(wb_ins_OPC == loadi)||(wb_ins_OPC == mov)))
            || ((wb_ins_T == FLOWCTRL) && ((wb_ins_OPC == 3'b101)||(wb_ins_OPC == 3'b110)))
        ) 
        begin
            wb_to_reg = 1'b1;
        end

        wb_output = wb_alu;

        if ((wb_ins_T == MOVEMENT)&&(wb_ins_OPC == load))begin
            wb_output = {2'b00, wb_mem_output};
        end
        else if(wb_ins_T == MOVEMENT && wb_ins_OPC == storei) begin
            wb_output = wb_imm_rc_extension;
        end
        else if ((wb_ins_T == FLOWCTRL)&&((wb_ins_OPC==jal)||(wb_ins_OPC==jral))) begin
            wb_output = wb_pc + 8'd1;
        end
    end

    //----------> EXECUTE  Stage
    reg[32:0] suma;
    always @(*) begin
        exec_alu_out = 34'd0;
        exec_alu_A = exec_regB;
        exec_alu_B = exec_regC;
        case (exec_ins_T)
            MOVEMENT: begin
                case (exec_ins_OPC)
                    load: begin
                        if(hazard_fw_A) begin
                            exec_alu_A = forward_A;
                        end 
                        exec_alu_B = exec_imm_rc_extension;
                        exec_alu_out = add(exec_alu_A, exec_alu_B);
                    end
                    store:  begin
                        if(hazard_fw_A) begin
                            exec_alu_A = forward_A;
                        end 
                        exec_alu_B = exec_imm_extension;
                        exec_alu_out = add(exec_alu_A, exec_alu_B);
                    end
                    loadi: begin
                        exec_alu_A = 34'd0;
                        exec_alu_B = exec_imm_rc_extension;
                        exec_alu_out = add(exec_alu_A, exec_alu_B);
                    end
                    storei: begin
                        if(hazard_fw_A) begin
                            exec_alu_A = forward_A;
                        end 
                        exec_alu_B = 34'd0;
                        exec_alu_out = add(exec_alu_A, exec_alu_B);
                    end
                    mov: begin
                        if(hazard_fw_A) begin
                            exec_alu_A = forward_A;
                        end 
                        exec_alu_B = 34'd0;
                        exec_alu_out = add(exec_alu_A, exec_alu_B);
                    end

                    3'b111: begin //NOP!
                        exec_alu_A      = 34'd0;
                        exec_alu_B      = 34'd0;
                        exec_alu_out    = 34'd0;
                    end
                endcase
            end

            LOGIC: begin
                if(hazard_fw_B) begin
                    exec_alu_B = forward_B;
                end 
                if(hazard_fw_A) begin
                    exec_alu_A = forward_A;
                end 
                case (exec_ins_OPC)
                    or_l:   exec_alu_out = {2'b00, (exec_alu_A[31:0] | exec_alu_B[31:0])}; 
                    inv_l:  exec_alu_out = {2'b00, (~exec_alu_A[31:0])}; 
                    and_l:  exec_alu_out = {2'b00, (exec_alu_A[31:0] & exec_alu_B[31:0])}; 
                endcase 
            end

            ARITHMETIC: begin
                if(hazard_fw_A) begin
                    exec_alu_A = forward_A;
                end 
                if(hazard_fw_B) begin
                    exec_alu_B = forward_B;
                end 
                case (exec_ins_OPC)
                    add_a:      exec_alu_out = add(exec_alu_A, exec_alu_B);
                    sub_a:      exec_alu_out = sub(exec_alu_A, exec_alu_B);
                    addc_a:     exec_alu_out = addc(exec_alu_A, exec_alu_B);
                    subc_a:     exec_alu_out = subc(exec_alu_A, exec_alu_B);
                endcase
            end 

            FLOWCTRL: begin
                if(hazard_fw_A && exec_ins_OPC != jral) begin
                    exec_alu_A = forward_A;
                end 
                case (exec_ins_OPC)
                    jump:   exec_alu_B = 34'd0;
                    bz:     exec_alu_B = exec_imm_extension;
                    bnz:    exec_alu_B = exec_imm_extension;
                    bc:     exec_alu_B = exec_imm_extension;
                    bv:     exec_alu_B = exec_imm_extension;
                    jal:    exec_alu_B = exec_imm_rc_extension;
                    jral:   begin
                        exec_alu_A = 34'd0;
                        exec_alu_B = exec_imm_rc_extension;
                    end
                    ret:    exec_alu_B = 34'd0;
                endcase

                exec_alu_out = add(exec_alu_A, exec_alu_B);
            end
        endcase
    end


    function bit[33:0] add(bit[33:0] x, bit[33:0] y);
        bit [32:0] suma;
        bit overflow;
        suma        = x[31:0] + y[31:0];
        overflow    = (x[31] == y[31]) && (suma[31] != x[31]);
        add         = {overflow, suma};
    endfunction

    function bit[33:0] sub(bit[33:0] x, bit[33:0] y);
        bit [32:0] suma;
        bit overflow;
        suma        = x[31:0] - y[31:0];
        overflow    = (x[31] != y[31]) && (suma[31] == x[31]);
        sub         = {overflow, suma};
    endfunction

    function bit[33:0] addc(bit[33:0] x, bit[33:0] y);
        bit [32:0] suma;
        bit overflow;
        suma        = x[31:0] + y[31:0] + y[32];
        overflow    = (x[31] == y[31]) && (suma[31] != x[31]);
        addc         = {overflow, suma};
    endfunction

    function bit[33:0] subc(bit[33:0] x, bit[33:0] y);
        bit [32:0] suma;
        bit overflow;
        suma        = x[31:0] - y[31:0] - y[32];
        overflow    = (x[31] != y[31]) && (suma[31] == x[31]);
        subc        = {overflow, suma};
    endfunction
    

    /////////////////////////////////////////////////////////////////////
    //////////////////////     DEBUG SIGNALS      ///////////////////////
    /////////////////////////////////////////////////////////////////////
    
    assign  model_pc_instruction    = tb_instruction; //output  wire [31:0]
    assign  model_pc_out            = pc; //output  wire [7:0] 
    assign  model_pc_input          = pc_bias; //output  wire [7:0] 
    //decode
    assign  model_dec_instruction   = dec_instruction; //output  wire [31:0]                     
    assign  model_dec_regB          = dec_regB; //output  wire [33:0]                     
    assign  model_dec_regC          = dec_regC; //output  wire [33:0]                     
    assign  model_dec_write_regA    = dec_write_data; //output  wire [33:0]                     
    //execute 
    assign  model_exec_instruction  = exec_instruction; //output  wire [31:0]
    assign  model_exec_alu_regA     = exec_alu_A; //output  wire [33:0]
    assign  model_exec_alu_regB     = exec_alu_B; //output  wire [33:0]
    assign  model_exec_alu_out      = exec_alu_out; //output  wire [33:0]
    //data mem
    assign  model_mem_instruction   = mem_instruction; //output  wire [31:0]
    assign  model_mem_address       = mem_addr; //output  wire [7:0]
    assign  model_mem_output_data   = mem_output; //output  wire [31:0]
    assign  model_mem_input_data    = mem_input; //output  wire [31:0]


    assign debug_mem_to_reg = mem_to_reg;
    assign hazard_A = hazard_fw_A;
    assign hazard_B = hazard_fw_B;
    assign debug_stall = stall;
    assign debug_rst_decode = flush_decode;
    assign debug_rst_execute = flush_execute;
    assign debug_wb_wen = regA_wen;
    assign debug_wb_a = dec_write_data;
    assign model_wb_instruction   = wb_instruction; //output  wire [31:0]
    assign debug_mem_alu = mem_alu;
endmodule //
