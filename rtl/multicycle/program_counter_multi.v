//////////////////////////////////////////////////////////////////////
// Module Name: program counter
//              -bias port sums to the program counter
//              -load_en port enables the program counter to take its
//                  value directly from bias
//              -bias has precedence over load
//              -if any of this options is used, in each clock prog_cnt
//                  updates with the prog_cnt + 4
//
// Author: Miqueas Filsinger
// Date: 14-06-2024
//
// Revision History:
// - Date: Description of changes made
// - Date: Description of changes made
//
//////////////////////////////////////////////////////////////////////

module program_counter_multi #(
        parameter MEM_INIT_FILE = "C:/Users/mique/OneDrive/Universidad/10mo Cuatri/PPS - Allegro/Trabajo Practico/RISC/rtl/multicycle/pc_mem_target_multi.hex"
    )
    (
    input   wire                clk,
    input   wire                pc_en,
    input   wire                bias_en,
    input   wire [7:0]          bias,
    output  wire [31:0]         instruction_output,
    output  wire [7:0]          program_counter
);

    // parameter   MEM_INIT_FILE   = "C:/Users/mique/OneDrive/Universidad/10mo Cuatri/PPS - Allegro/Trabajo Practico/RISC/rtl/multicycle/pc_mem_target_multi.hex";   
  	reg [7:0]   mem [0:1023];
    reg [7:0]   mem_counter = 8'h00; 

    initial begin
        if (MEM_INIT_FILE != "") begin
          	$readmemh(MEM_INIT_FILE, mem, 0, 1023);
        end
    end

    always @(posedge clk) begin
        if(bias_en) begin
            mem_counter <= bias;
        end 
        else begin
            if(pc_en) begin
                mem_counter <= mem_counter + 8'd1;    
            end 
        end
    end    
    assign      instruction_output[31:0] = {mem[mem_counter*4], mem[mem_counter*4+8'd1], mem[mem_counter*4+8'd2], mem[mem_counter*4+8'd3]};
    assign      program_counter          = mem_counter;

endmodule //