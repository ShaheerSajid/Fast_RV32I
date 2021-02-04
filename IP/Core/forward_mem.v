module forward_mem
(rs1id,rs2id, rdmem, wbmem, fa, fb);

input [4:0] rs1id,rs2id, rdmem;
input wbmem;
output reg fa,fb;

always@(*)
begin

	fa = wbmem && rdmem != 0 && rdmem == rs1id;
	fb = wbmem && rdmem != 0 && rdmem == rs2id;
end
endmodule