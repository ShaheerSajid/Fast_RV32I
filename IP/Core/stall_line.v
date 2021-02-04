module stall_line
(clk, inst,lu_ie, lu_imem, wb_ie, wb_imem, rd_ie, rd_imem, rs1_id, rs2_id, br_stall);

input clk;
input wb_ie, wb_imem, lu_ie, lu_imem;
input inst;
input [4:0] rd_ie, rd_imem, rs1_id, rs2_id;
output reg br_stall;


wire stall_condition_ie, stall_condition_imem;

assign stall_condition_ie = wb_ie && rd_ie != 0 && ((rd_ie == rs1_id) || (rd_ie == rs2_id));
assign stall_condition_imem = wb_imem && rd_imem != 0 && ((rd_imem == rs1_id) || (rd_imem == rs2_id));


always@(*)
begin
		br_stall = ( ((inst | lu_ie) & stall_condition_ie) | ((inst & lu_imem) & stall_condition_imem) ) ;
end
endmodule