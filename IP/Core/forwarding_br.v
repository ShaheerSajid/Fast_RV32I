module forwarding_br
(rs1id,rs2id, rdmem, rdwb, wbmem, wbwb, fa, fb);

input [4:0] rs1id,rs2id,rdmem, rdwb;
input wbmem, wbwb;
output reg[1:0] fa,fb;

always@(*)
begin
	
	if(wbmem && rdmem != 0 && rdmem == rs1id)
		fa = 1;
	else if (wbwb && rdwb != 0 && rdwb == rs1id)
		fa = 2;
	else
		fa = 0;
		
	if(wbmem && rdmem != 0 && rdmem == rs2id)
		fb = 1;
	else if (wbwb && rdwb != 0 && rdwb == rs2id)
		fb = 2;
	else
		fb = 0;
end
endmodule