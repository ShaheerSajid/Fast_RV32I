module Core
(
	clk,
	reset,
	
	//instruction master
	inst_addr, 
	inst_data, 
	inst_stall, 
	
	//avalon master
	avl_address, 
	avl_byteenable, 
	avl_read, 
	avl_readdata, 
	avl_waitrequest, 
	avl_write, 
	avl_writedata
	
);

input clk;
input reset;

input [31:0] inst_data;
output [31:0] inst_addr;
output inst_stall; 


input [31:0] avl_readdata;
input avl_waitrequest;
output [31:0] avl_address;
output [31:0] avl_writedata;
output [3:0] avl_byteenable;
output avl_read;
output avl_write;


//Signals
wire [31:0] pc_in;
wire [31:0]pc_out;
wire pc_sel;
wire [31:0]instruction;
wire RegWEn;
wire [31:0] rd;
wire [31:0]rs1;
wire [31:0]rs2;
reg [31:0]rs1_br,rs2_br, fa_out,fb_out;
wire [1:0]fa_br,fb_br;
wire [1:0]fa_mem,fb_mem;
wire [31:0]A,B,imm,A_out;
wire [2:0]immsel;
wire Asel,Asel_br,Bsel_br,Bsel;
wire [4:0]ALUsel;
wire [31:0]aluout, pcAlu;
wire [1:0]wbSel;
wire [1:0]memWrite;
wire [31:0]dmout;
wire BrUn, BrEq, BrLT, flushid;		
wire c_sel,id_stall,pc_stall;
wire [31:0]instruction_pipe_id;
wire [31:0]pc_out_pipe_id;
wire [2:0]wbsel_pipe_iwb;
wire [31:0]dmout_pipe_iwb,  aluout_pipe_iwb, pc_out_pipe_iwb;
wire [4:0] rdadd_pipe_iwb;
wire [1:0]memWrite_pipe_ie;
wire [2:0]wbsel_pipe_ie;
wire [6:0] aluop_pipe_ie;
wire [31:0] pc_out_pipe_ie, rs1_pipe_ie, rs2_pipe_ie, imm_pipe_ie;
wire [4:0] rdadd_pipe_ie,rs1add_pipe_ie,rs2add_pipe_ie;
wire [1:0]memWrite_pipe_imem;
wire [2:0]wbsel_pipe_imem;
wire [31:0]rs2_pipe_imem,  aluout_pipe_imem,pc_out_pipe_imem;
wire [4:0] rdadd_pipe_imem;
wire [31:0]adm;
wire [2:0]mode_ie, mode_imem;
wire[31:0] pc_out_sync;
wire [31:0]inst_addr_mid;
wire reset_id;
wire clk1;
wire stall_core;
wire reset_ie;
wire br_true;
wire br_stall;

assign clk1 = clk;

//Program Counter Start
mux2to1 mux2to1_inst
(
	.din_0(pc_out+4) ,	// input [31:0] din_0_sig
	.din_1(pcAlu +4) ,	// input [31:0] din_1_sig
	.sel(pc_sel) ,	// input  sel_sig
	.mux_out(pc_in) 	// output  mux_out_sig
);
Program_Counter pc(
	.stall(br_stall),
	.pc_in(pc_in) ,	// input [31:0] pc_in_sig
	.pc_reset(reset) ,	// input  pc_reset_sig
	.clk(clk1) ,	// input  clk_sig
	.pc_out(pc_out) 	// output [31:0] pc_out_sig
);


/*register_stall #(.width(32)) reg_pc_out_sync
(
	.stall(br_stall),
	.in((pc_sel)?pcAlu: pc_out) ,	// input [width-1:0] in_sig
	.out(pc_out_sync) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset) 	// input  reset_sig
);*/
assign pc_out_sync = (pc_sel)?pcAlu: pc_out;
//Program Counter End
assign inst_addr = (pc_sel)?pcAlu >>2: pc_out >>2;
assign instruction = inst_data;


//####################IF/ID########################
assign reset_id = (~br_stall & flushid) | reset;
assign inst_stall = br_stall;


register_stall #(.width(32)) regID_pc
(
	.stall(br_stall),
	.in(pc_out_sync) ,	// input [width-1:0] in_sig
	.out(pc_out_pipe_id) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset) 	// input  reset_sig
);
/*register_stall #(.width(32)) regID_im
(
	.stall(br_stall),
	.in(instruction) ,	// input [width-1:0] in_sig
	.out(instruction_pipe_id) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset_id) 	// input  reset_sig
);*/
assign instruction_pipe_id = instruction;
//#################################################

//Reg File Start

REG_FILE regfile
(
	.clk(clk1) ,	// input  clk_sig
	.reset(reset) ,	// input  reset_sig
	.write(wbsel_pipe_iwb[0]) ,	// input  write_sig
	.wrAddr(rdadd_pipe_iwb) ,	// input [4:0] wrAddr_sig
	.wrData(rd) ,	// input [31:0] wrData_sig
	.rdAddrA(instruction_pipe_id[19:15]) ,	// input [4:0] rdAddrA_sig
	.rdDataA(rs1) ,	// output [31:0] rdDataA_sig
	.rdAddrB(instruction_pipe_id[24:20]) ,	// input [4:0] rdAddrB_sig
	.rdDataB(rs2) 	// output [31:0] rdDataB_sig
);

//Reg File End



IMM_gen IMM_gen_inst
(
	.instruction(instruction_pipe_id) ,	// input [31:0] instruction_sig
	.immsel(immsel) ,	// input [1:0] immsel_sig
	.imm(imm) 	// output [31:0] imm_sig
);




//######################ID/IE#####################
assign reset_ie = br_stall | reset;

register #(.width(3)) regIE_wb
(
	.in({wbSel,RegWEn}) ,	// input [width-1:0] in_sig
	.out(wbsel_pipe_ie) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset_ie) 	// input  reset_sig
);
register #(.width(2)) regIE_m
(
	.in(memWrite) ,	// input [width-1:0] in_sig
	.out(memWrite_pipe_ie) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset_ie) 	// input  reset_sig
);
register #(.width(7)) regIE_ex
(
	.in({Asel,Bsel,ALUsel}) ,	// input [width-1:0] in_sig
	.out(aluop_pipe_ie) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset_ie) 	// input  reset_sig
);
register #(.width(32)) regIE_pc
(
	.in(pc_out_pipe_id) ,	// input [width-1:0] in_sig
	.out(pc_out_pipe_ie) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset) 	// input  reset_sig
);
register #(.width(32)) regIE_rs1
(
	.in(rs1_br) ,	// input [width-1:0] in_sig
	.out(rs1_pipe_ie) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset) 	// input  reset_sig
);
register #(.width(32)) regIE_rs2
(
	.in(rs2_br) ,	// input [width-1:0] in_sig
	.out(rs2_pipe_ie) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset) 	// input  reset_sig
);
register #(.width(32)) regIE_imm
(
	.in(imm) ,	// input [width-1:0] in_sig
	.out(imm_pipe_ie) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset) 	// input  reset_sig
);

register #(.width(5)) regIE_rd
(
	.in(instruction_pipe_id[11:7]) ,	// input [width-1:0] in_sig
	.out(rdadd_pipe_ie) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset) 	// input  reset_sig
);
register #(.width(5)) regIE_rs1add
(
	.in(instruction_pipe_id[19:15]) ,	// input [width-1:0] in_sig
	.out(rs1add_pipe_ie) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset) 	// input  reset_sig
);
register #(.width(5)) regIE_rs2add
(
	.in(instruction_pipe_id[24:20]) ,	// input [width-1:0] in_sig
	.out(rs2add_pipe_ie) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset) 	// input  reset_sig
);
register #(.width(3)) regIE_mode
(
	.in(instruction_pipe_id[14:12]) ,	// input [width-1:0] in_sig
	.out(mode_ie) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset) 	// input  reset_sig
);
//################################################


//Hazards Compensation
//Forawrding Unit for Branch Comparator
forwarding_br forwarding_br_inst
(
	.rs1id(instruction_pipe_id[19:15]) ,	// input [4:0] rs1id_sig
	.rs2id(instruction_pipe_id[24:20]) ,	// input [4:0] rs2id_sig
	.rdmem(rdadd_pipe_imem),
	.rdwb(rdadd_pipe_iwb) ,	// input [4:0] rdwb_sig
	.wbmem(wbsel_pipe_imem[0]),
	.wbwb(wbsel_pipe_iwb[0]) ,	// input  wbwb_sig
	.fa(fa_br) ,	// output [1:0] fa_sig
	.fb(fb_br) 	// output [1:0] fb_sig
);

always@(*)
begin
	case(fa_br)
		0:rs1_br = rs1;
		1:rs1_br = (!wbsel_pipe_imem[2] & wbsel_pipe_imem[1])?pc_out_pipe_imem: aluout_pipe_imem;
		2:rs1_br = rd;
		default:rs1_br =  0;
	endcase
	
	case(fb_br)
		0:rs2_br = rs2;
		1:rs2_br = (!wbsel_pipe_imem[2] & wbsel_pipe_imem[1])?pc_out_pipe_imem: aluout_pipe_imem;
		2:rs2_br = rd;
		default:rs2_br =  0;
	endcase
end

forwarding_br forwarding_br_inst_1
(
	.rs1id(rs1add_pipe_ie) ,	// input [4:0] rs1id_sig
	.rs2id(rs2add_pipe_ie) ,	// input [4:0] rs2id_sig
	.rdmem(rdadd_pipe_imem),
	.rdwb(rdadd_pipe_iwb) ,	// input [4:0] rdwb_sig
	.wbmem(wbsel_pipe_imem[0]),
	.wbwb(wbsel_pipe_iwb[0]) ,	// input  wbwb_sig
	.fa(fa_mem) ,	// output [1:0] fa_sig
	.fb(fb_mem) 	// output [1:0] fb_sig
);
always@(*)
begin
	case(fa_mem)
		0:fa_out = rs1_pipe_ie;
		1:fa_out = (!wbsel_pipe_imem[2] & wbsel_pipe_imem[1])?pc_out_pipe_imem: aluout_pipe_imem;
		2:fa_out = rd;
		default:fa_out =  0;
	endcase
	
	case(fb_mem)
		0:fb_out = rs2_pipe_ie;
		1:fb_out = (!wbsel_pipe_imem[2] & wbsel_pipe_imem[1])?pc_out_pipe_imem: aluout_pipe_imem;
		2:fb_out = rd;
		default:fb_out =  0;
	endcase
end

//2. Stall Condition
stall_line stall_line_inst
(
	.clk(clk1),
	.inst(br_true),
	.lu_ie(memWrite_pipe_ie[1]),
	.lu_imem(memWrite_pipe_imem[1]),
	.wb_ie(wbsel_pipe_ie[0]) ,	// input  mem_ie_sig
	.wb_imem(wbsel_pipe_imem[0]) ,	// input  mem_ie_sig
	.rd_ie(rdadd_pipe_ie) ,	// input [4:0] rd_ie_sig
	.rd_imem(rdadd_pipe_imem) ,	// input [4:0] rd_ie_sig
	.rs1_id(instruction_pipe_id[19:15]) ,	// input [4:0] rs1_id_sig
	.rs2_id(instruction_pipe_id[24:20]) ,	// input [4:0] rs2_id_sig
	.br_stall(br_stall) 	// output  pc_stall_sig
);

//###################################################




//ALU Start

mux2to1 mux2to1_inst1
(
	.din_0(pc_out_pipe_id) ,	// input [31:0] din_0_sig
	.din_1(rs1_br) ,	// input [31:0] din_1_sig
	.sel(Asel_br) ,	// input  sel_sig
	.mux_out(A_out) 	// output  mux_out_sig
);
pcALU pcALU_inst
(
	.A(A_out) ,	// input [31:0] A_sig
	.B(imm) ,	// input [31:0] B_sig
	.pcalu(pcAlu), 	// output [31:0] pcalu_sig
	.Bsel_br(Bsel_br)
);
mux2to1 mux2to1_inst8
(
	.din_0(fa_out) ,	// input [31:0] din_0_sig
	.din_1(pc_out_pipe_ie) ,	// input [31:0] din_1_sig
	.sel(aluop_pipe_ie[6]) ,	// input  sel_sig
	.mux_out(A) 	// output  mux_out_sig
);
mux2to1 mux2to1_inst2
(
	.din_0(fb_out) ,	// input [31:0] din_0_sig
	.din_1(imm_pipe_ie) ,	// input [31:0] din_1_sig
	.sel(aluop_pipe_ie[5]) ,	// input  sel_sig
	.mux_out(B) 	// output  mux_out_sig
);
ALU alu
(
	.A(A) ,	// input [31:0] A_sig
	.B(B) ,	// input [31:0] B_sig
	.ALUsel(aluop_pipe_ie[4:0]) ,	// input [3:0] ALUsel_sig
	.result(aluout)	// output [31:0] ALUout_sig
);
//ALU End



//################IEX/IMEM######################
register #(.width(32)) regIMEM_rs2
(
	.in(fb_out) ,	// input [width-1:0] in_sig
	.out(rs2_pipe_imem) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset) 	// input  reset_sig
);
register #(.width(32)) regIMEM_aluout
(
	.in(aluout) ,	// input [width-1:0] in_sig
	.out(aluout_pipe_imem) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset) 	// input  reset_sig
);
register #(.width(3)) regIMEM_wb
(
	.in(wbsel_pipe_ie) ,	// input [width-1:0] in_sig
	.out(wbsel_pipe_imem) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset) 	// input  reset_sig
);
register #(.width(5)) regIMEM_rd
(
	.in(rdadd_pipe_ie) ,	// input [width-1:0] in_sig
	.out(rdadd_pipe_imem) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset) 	// input  reset_sig
);
register #(.width(2)) regIMEM_m
(
	.in(memWrite_pipe_ie) ,	// input [width-1:0] in_sig
	.out(memWrite_pipe_imem) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset) 	// input  reset_sig
);
register #(.width(32)) regIMEM_pc
(
	.in(pc_out_pipe_ie + 4) ,	// input [width-1:0] in_sig
	.out(pc_out_pipe_imem) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset) 	// input  reset_sig
);
register #(.width(3)) regIMEM_mode
(
	.in(mode_ie) ,	// input [width-1:0] in_sig
	.out(mode_imem) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset) 	// input  reset_sig
);




//Data Master
core2avl core2avl_inst
(
	.clk(clk1),
	.reset(reset),
	.mode(mode_imem) ,	// input [2:0] mode_sig
	.addr(aluout_pipe_imem) ,	// input [ADDR_WIDTH-1:0] addr_sig
	.data2write(rs2_pipe_imem) ,	// input [DATA_WIDTH-1:0] data2write_sig
	.data2read(dmout) ,	// output [DATA_WIDTH-1:0] data2read_sig
	.rw(memWrite_pipe_imem) ,	// input [1:0] rw_sig
	.stall(stall_core) ,	// output  stall_sig
	
	
	.readdata(avl_readdata) ,	// input [DATA_WIDTH-1:0] readdata_sig
	.waitrequest(avl_waitrequest) ,	// input  waitrequest_sig
	.address(avl_address) ,	// output [ADDR_WIDTH-1:0] address_sig
	.writedata(avl_writedata) ,	// output [DATA_WIDTH-1:0] writedata_sig
	.byteenable(avl_byteenable) ,	// output [3:0] bytenable_sig
	.read(avl_read) ,	// output  read_sig
	.write(avl_write) 	// output  write_sig
);
defparam core2avl_inst.DATA_WIDTH = 32;
defparam core2avl_inst.ADDR_WIDTH = 32;

	
//############IWB#####################
register #(.width(3)) regIWB_wb
(
	.in(wbsel_pipe_imem) ,	// input [width-1:0] in_sig
	.out(wbsel_pipe_iwb) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset) 	// input  reset_sig
);
register #(.width(32)) regIWB_aluout
(
	.in(aluout_pipe_imem) ,	// input [width-1:0] in_sig
	.out(aluout_pipe_iwb) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset) 	// input  reset_sig
);
register #(.width(5)) regIWB_rd
(
	.in(rdadd_pipe_imem) ,	// input [width-1:0] in_sig
	.out(rdadd_pipe_iwb) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset) 	// input  reset_sig
);
register #(.width(32)) regIWB_pc
(
	.in(pc_out_pipe_imem) ,	// input [width-1:0] in_sig
	.out(pc_out_pipe_iwb) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset) 	// input  reset_sig
);
/*register #(.width(32)) regIWB_dmout
(
	.in(dmout) ,	// input [width-1:0] in_sig
	.out(dmout_pipe_iwb) ,	// output [width-1:0] out_sig
	.clk(clk1) ,	// input  clk_sig
	.reset(reset) 	// input  reset_sig
);*/
assign dmout_pipe_iwb = dmout;
//############################################





mux2to1 mux2to1_inst3
(
	.din_0(aluout_pipe_iwb) ,	// input [31:0] din_0_sig
	.din_1(dmout_pipe_iwb) ,	// input [31:0] din_1_sig
	.sel(wbsel_pipe_iwb[2]) ,	// input  sel_sig
	.mux_out(adm) 	// output  mux_out_sig
);
mux2to1 mux2to1_inst4
(
	.din_0(adm) ,	// input [31:0] din_0_sig
	.din_1(pc_out_pipe_iwb) ,	// input [31:0] din_1_sig
	.sel(wbsel_pipe_iwb[1]) ,	// input  sel_sig
	.mux_out(rd) 	// output  mux_out_sig
);



//Control Start
Branch_comp branch
(
	.A(rs1_br) ,	// input [31:0] A_sig
	.B(rs2_br) ,	// input [31:0] B_sig
	.BrUn(BrUn) ,	// input  BrUn_sig
	.BrLT(BrLT) ,	// output  BrLT_sig
	.BrEq(BrEq) 	// output  BrEq_sig
);
Control_Path Control
(
	.instruction(instruction_pipe_id) ,	// input [31:0] instruction_sig
	.BrEq(BrEq) ,	// input  BrEq_sig
	.BrLt(BrLT) ,	// input  BrLt_sig
	.BrUn(BrUn),
	.PCSel(pc_sel) ,	// output  PCSel_sig
	.Immsel(immsel) ,	// output  Immsel_sig
	.RegWEn(RegWEn) ,	// output  RegWEn_sig
	.Asel(Asel),
	.Bsel(Bsel) ,	// output  Bsel_sig
	.Asel_br(Asel_br) ,	// output  Asel_sig
	.Bsel_br(Bsel_br),
	.ALUSel(ALUsel) ,	// output [3:0] ALUSel_sig
	.MemRW(memWrite) ,	// output  MemRW_sig
	.WBsel(wbSel), 	// output [1:0] WBsel_sig
	.flush_ID(flushid),
	.br_true(br_true)
);
//Control End


endmodule
