module ALU
(A,B,ALUsel,result);

input [31:0]A,B;
input[4:0]ALUsel;
output [31:0] result;

reg[31:0]ALUout;
assign result = ALUout;

always@(*)
begin
	case (ALUsel)
		5'b00000:ALUout = A+B;//add
		5'b00001:ALUout = A<<B[4:0];//sll
		5'b00010:ALUout = ($signed(A)<$signed(B));//slt
		5'b00011:ALUout = (A<B);//sltu
		5'b00100:ALUout = A^B;//xor
		5'b00101:ALUout = A>>B[4:0];//srl
		5'b00110:ALUout = A|B;//or
		5'b00111:ALUout = A&B;//and
		5'b01000:ALUout = A-B;//sub
		5'b01101:ALUout = $signed(A)>>>B[4:0];//srai
		5'b01111:ALUout = B;//B
		default: ALUout = 0;
	endcase	
end

endmodule
