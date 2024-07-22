//////////////////////////////////////////////////////////////////////
//@ Module Name: ALU
//@              - Arithmetic and Logic Processing module
//@              - ALU ctrl parameters:
//@                     ADD  = 3'd0
//@                     ADDC = 3'd1
//@                     SUB  = 3'd2
//@                     SUBC = 3'd3
//@                     EXCEPTION = 3'd4 --> will output zero
// Author: Miqueas Filsinger
// Date: 13-06-2024
//
// Interesting Notes:
// @ https://teaching.idallen.com/dat2343/10f/notes/040_overflow.txt
// =====================================================
// The CARRY flag and OVERFLOW flag in binary arithmetic
// =====================================================
// - Ian! D. Allen - idallen@idallen.ca - www.idallen.com
// 
// Do not confuse the "carry" flag with the "overflow" flag in integer
// arithmetic.  Each flag can occur on its own, or both together.  The CPU's
// ALU doesn't care or know whether you are doing signed or unsigned
// mathematics; the ALU always sets both flags appropriately when doing any
// integer math.  The ALU doesn't know about signed/unsigned; the ALU just
// does the binary math and sets the flags appropriately.  It's up to you,
// the programmer, to know which flag to check after the math is done.
// 
// If your program treats the bits in a word as unsigned numbers, you
// must watch to see if your arithmetic sets the carry flag on, indicating
// the result is wrong.  You don't care about the overflow flag when doing
// unsigned math.  (The overflow flag is only relevant to signed numbers, not
// unsigned.)
// 
// If your program treats the bits in a word as two's complement signed
// values, you must watch to see if your arithmetic sets the overflow flag
// on, indicating the result is wrong.  You don't care about the carry
// flag when doing signed, two's complement math.  (The carry flag is only
// relevant to unsigned numbers, not signed.)
// 
// In unsigned arithmetic, watch the carry flag to detect errors.
// In unsigned arithmetic, the overflow flag tells you nothing interesting.
// 
// In signed arithmetic, watch the overflow flag to detect errors.
// In signed arithmetic, the carry flag tells you nothing interesting.
//
//////////////////////////////////////////////////////////////////////

module alu 
    #(
        parameter NB_REGISTERS  = 34,
        parameter DATA_WIDTH    = 32
    )
    (
        input   wire [2:0]                   i_alu_ctrl,
        input   wire [NB_REGISTERS-1:0]      i_data_A,
        input   wire [NB_REGISTERS-1:0]      i_data_B,

        output  wire [NB_REGISTERS-1:0]      o_data
    );

    //Handling variables  
    wire    regA_v;
    assign  regA_v = i_data_A[NB_REGISTERS-1];                      
    wire    regA_c;
    assign  regA_c = i_data_A[NB_REGISTERS-2];                      
    wire [DATA_WIDTH-1:0] regA_data;
    assign  regA_data = i_data_A[NB_REGISTERS-3:0];  

    wire    regB_v;
    assign  regB_v = i_data_B[NB_REGISTERS-1];                      
    wire    regB_c;
    assign  regB_c = i_data_B[NB_REGISTERS-2];
    wire [DATA_WIDTH-1:0] regB_data;
    assign  regB_data = i_data_B[NB_REGISTERS-3:0];

    reg [DATA_WIDTH-1:0]            r_out_data;
    reg                             r_out_c;
    reg                             r_out_v;
    assign o_data = {r_out_v, r_out_c, r_out_data};

    reg [DATA_WIDTH:0] r_sum;   //Intermidiate register with +1bit to compute sum with carry out


    //ALU operations
    parameter ADD   = 3'd0;
    parameter SUB   = 3'd1;
    parameter ADDC  = 3'd2;
    parameter SUBC  = 3'd3;
    parameter EXCEPTION = 3'd4;
    
    always @(*) begin
        r_sum       = {DATA_WIDTH+1{1'b0}};
        r_out_data  = {NB_REGISTERS{1'b0}};
        r_out_c     = 1'b0;
        r_out_v     = 1'b0;
        case (i_alu_ctrl)
            ADD : begin
                    r_sum       = regA_data + regB_data;
                    r_out_data  = r_sum[DATA_WIDTH-1:0];
                    r_out_c     = r_sum[DATA_WIDTH]; 
                    if ((regA_data[DATA_WIDTH-1]==regB_data[DATA_WIDTH-1])&&(regA_data[DATA_WIDTH-1] != r_out_data[DATA_WIDTH-1])) begin
                        //if operands have the same msb but output doesnt, then overflow!
                        r_out_v = 1'b1;
                    end
            end

            SUB : begin
                    r_sum       = regA_data - regB_data;
                    r_out_data  = r_sum[DATA_WIDTH-1:0];
                    r_out_c     = r_sum[DATA_WIDTH];
                    if ((regA_data[DATA_WIDTH-1]!=regB_data[DATA_WIDTH-1]) && (regA_data[DATA_WIDTH-1]==r_out_data[DATA_WIDTH-1])) begin
                        //if operands have the same msb but output doesnt, then overflow!
                        r_out_v = 1'b1;
                    end
            end

            ADDC : begin
                    r_sum       = regA_data + regB_data + {{DATA_WIDTH-1{1'b0}}, regB_c};
                    r_out_data  = r_sum[DATA_WIDTH-1:0];
                    r_out_c     = r_sum[DATA_WIDTH];   
                    if ((regA_data[DATA_WIDTH-1]==regB_data[DATA_WIDTH-1])&&(regA_data[DATA_WIDTH-1] != r_out_data[DATA_WIDTH-1])) begin
                        //if operands have the same sign but output doesnt, then overflow!
                        r_out_v = 1'b1;
                    end  
            end

            SUBC : begin
                    r_sum       = regA_data - regB_data - {{DATA_WIDTH-1{1'b0}}, regB_c};
                    r_out_data  = r_sum[DATA_WIDTH-1:0];
                    r_out_c     = r_sum[DATA_WIDTH];
                    if ((regA_data[DATA_WIDTH-1]!=regB_data[DATA_WIDTH-1]) && (regA_data[DATA_WIDTH-1]==r_out_data[DATA_WIDTH-1])) begin
                        //if operands have the same msb but output doesnt, then overflow!
                        r_out_v = 1'b1;
                    end
            end
            
            EXCEPTION: begin
                    r_sum       = {DATA_WIDTH+1{1'b0}};
                    r_out_data  = {NB_REGISTERS{1'b0}};
                    r_out_c     = 1'b0;
                    r_out_v     = 1'b0;
            end
        endcase

        // if (i_alu_ctrl != EXCEPTION) begin
        //     if(regA_data[DATA_WIDTH-1] == regB_data[DATA_WIDTH-1]) begin
        //         //if both inputs have equal sign bits
        //         //and result sign is different -> OVERFLOW!
        //         if(r_out_data[DATA_WIDTH-1] != regA_data[DATA_WIDTH-1]) begin
        //             r_out_v = 1'b1;
        //         end
        //     end
        // end
    end

endmodule //


