module Branch_comp(A,B,BrUn, BrLT, BrEq);



input [31:0]A,B;
input BrUn;
output reg BrLT, BrEq;

always@(*)
begin
	
	BrLT = BrUn?A<B:$signed(A)<$signed(B);
	BrEq = BrUn?A==B:$signed(A)==$signed(B);

end
endmodule

