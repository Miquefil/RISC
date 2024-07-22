//////////////////////////////////////////////////////////////////////
// Module Name: memory
//              -   32 bits wide memory, byte adressable
//              -   2^(ADDR_WIDTH) memory directions
//              -   Less significant byte is addressed first in memory
//          
//
// Author: Miqueas Filsinger
// Date: 05-06-2024
//
// Revision History:
//
//////////////////////////////////////////////////////////////////////

 module memory 
    #(
        parameter MEM_WIDTH  = 8,
        parameter DATA_WIDTH = 32,
        parameter ADDR_WIDTH = 8,
        parameter MEM_INIT_FILE = "C:/Users/mique/OneDrive/Universidad/10mo Cuatri/PPS - Allegro/Trabajo Practico/RISC/rtl/multicycle/mem.hex"
    )
    (
    input   wire                            clk,
    input   wire                            rst_n,
    //Input writing data
    input   wire                            i_wenable,
    input   wire    [0:ADDR_WIDTH-1]        i_address,
    input   wire    [0:DATA_WIDTH-1]        i_data,
    //Outputs
    output  wire    [0:DATA_WIDTH-1]        o_data
    );

    //////////  MEMORY DECLARATION    ///////////
    // parameter   MEM_INIT_FILE   = "C:/Users/mique/OneDrive/Universidad/10mo Cuatri/PPS - Allegro/Trabajo Practico/RISC/rtl/multicycle/mem.hex";
    parameter               N_ROWS = (1 << ADDR_WIDTH);      //Memory depth
    reg[0:MEM_WIDTH-1]      memory [0:4*N_ROWS-1];                //Memory Declaration
    
    initial begin
        if (MEM_INIT_FILE != "") begin
          	$readmemh(MEM_INIT_FILE, memory, 0, 1023);
        end
    end 

    integer i;
    always @(posedge clk) begin
        if(!rst_n) begin
            // for (i = 0; i < N_ROWS ; i = i+1) begin
            //   memory[i]   <= 8'd0;
            // end
        end
        else begin
            if(i_wenable) begin
                memory[i_address*4]                               <= i_data[24:31];
                memory[i_address*4 + {{ADDR_WIDTH-1{1'b0}},1'd1}] <= i_data[16:23];
                memory[i_address*4 + {{ADDR_WIDTH-2{1'b0}},2'd2}] <= i_data[8:15];
                memory[i_address*4 + {{ADDR_WIDTH-2{1'b0}},2'd3}] <= i_data[0:7];
            end
        end
    end

    //OUTPUT
    //TODO: Raise a flag if input address is not divisible by 4 
//     assign o_data[24:31]            = memory[i_address];
//     assign o_data[16:23]            = memory[i_address + {{ADDR_WIDTH-1{1'b0}},1'd1}];
//     assign o_data[8:15]             = memory[i_address + {{ADDR_WIDTH-2{1'b0}},2'd2}];
//     assign o_data[0:7]              = memory[i_address + {{ADDR_WIDTH-2{1'b0}},2'd3}];
	
   	assign o_data = {{memory[i_address*4]}, 
                     {memory[i_address*4 + {{ADDR_WIDTH-1{1'b0}},1'd1}]}, 
                     {memory[i_address*4 + {{ADDR_WIDTH-2{1'b0}},2'd2}]}, 
                     {memory[i_address*4 + {{ADDR_WIDTH-2{1'b0}},2'd3}]}};


 endmodule
 //