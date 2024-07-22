//////////////////////////////////////////////////////////////////////
// Module Name: Register File
//              - 32 general purpose registers for ALU fast access.
//              - First register (Register A) is the only one available to do a write process
//              - Reg B and Reg C have ALU purposes
// Author: Miqueas Filsinger
// Date: 13-06-2024
//
// Revision History:
//////////////////////////////////////////////////////////////////////

module register_file 
    #(
        parameter REG_WIDTH     = 34,
        parameter ADDR_WIDTH    = 5
    )
    (
        input   wire                        clk,
        input   wire                        rst_n,

        //Inputs:
        //REGISTER A
        input   wire [ADDR_WIDTH-1:0]       i_address_reg_a,
        input   wire                        i_wenable_reg_a,
        input   wire [REG_WIDTH-1:0]        i_writedata_reg_a,
        //REGISTER B AND C
      	input   wire [ADDR_WIDTH-1:0]       i_address_reg_b,
      	input   wire [ADDR_WIDTH-1:0]       i_address_reg_c,

        //OUTPUTS
        output  wire [REG_WIDTH-1:0]        o_data_regb,
        output  wire [REG_WIDTH-1:0]        o_data_regc
    );

    //Memory creation
    parameter N_REGISTERS   = (1 << ADDR_WIDTH);
    parameter MEM_INIT_FILE = "C:/Users/mique/OneDrive/Universidad/10mo Cuatri/PPS - Allegro/Trabajo Practico/RISC/rtl/register_mem.hex";
    reg [REG_WIDTH-1:0]     registers [0:N_REGISTERS-1];
    initial begin
        if (MEM_INIT_FILE != "") begin
          	$readmemh(MEM_INIT_FILE, registers, 0, 31);
        end
    end 

    //Usable registers assignation
    wire    [REG_WIDTH-1:0] reg_B;                       //RegB declaration
    assign reg_B = registers[i_address_reg_b];      //RegB definition
    assign o_data_regb = reg_B;                     //Output

    wire    [REG_WIDTH-1:0] reg_C;
    assign reg_C = registers[i_address_reg_c];
    assign o_data_regc = reg_C;    

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            for (i = 0; i<N_REGISTERS ; i=i+1) begin
                registers[i]    <= {REG_WIDTH{1'b0}};
            end
        end else begin
           if(i_wenable_reg_a) begin
                registers[i_address_reg_a] <= i_writedata_reg_a;
            end 
        end        
    end


endmodule //