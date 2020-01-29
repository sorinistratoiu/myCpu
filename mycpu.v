module mycpu(
   output[7:0] acc,alu_r,pc,st_p,//iesiri microprocesor
	output[4:0] ir,
   output [2:0] fl,//flag-uri unitatea aritmetico-logica
   input clk,//intrare semnal ceas
   input reset//intrare reset
   
 
   );


	//variabile temporare
	wire [7:0] second_operand,first_operand,Q1,Q2,Y,counter_out,stack_ptr,data_mem_out;
	wire [2:0] flags;
	wire[7:0] stack_out,stack_mux_out;
	wire [17:0] ctrl;
	wire [12:0] instr_mem_out;
	wire [4:0] opcode;

	//unitate de control
	cu control_unit(opcode,flags,ctrl);

	//unitate aritmetico-logica
	ual ual1 (Y,C,V,Z,first_operand,second_operand,ctrl[8:6]);

	//registru
	register W(Q1,Y,clk,reset,ctrl[0]);

	//registru
	register T(Q2,instr_mem_out[7:0],clk,reset,ctrl[1]);
	
	
	//multiplexor 4 la 1
	mux_4to1 sel1(Q1,Q2,Y,data_mem_out ,ctrl[3:2],first_operand);

	//multiplexor 4 la 1
	mux_4to1 sel2(Q1,Q2,8'b0, data_mem_out,ctrl[5:4],second_operand);

	//stiva
	Stack st1(~clk,reset,stack_out, counter_out + 8'd1 ,ctrl[12],ctrl[11],stack_ptr);

	//multiplexor 2 la 1
	mux2to1_8bit stack_mux(Q2,stack_out,ctrl[11],stack_mux_out);

	//program counter
	p_counter pc1(stack_mux_out,clk,ctrl[9],ctrl[10],reset,counter_out);

	//memorie instructiuni
	instr_mem rom(instr_mem_out,counter_out,1'b1);

	//memorie date
	//data_mem dm1(data_mem_out,Q2,1'b1);

	data_mem dm1(Y,instr_mem_out[7:0],ctrl[14],clk,data_mem_out);

	//flip flop-uri pentru flag-uri
	flip_flop carry(flags[0],C,clk,reset,ctrl[15]);

	flip_flop overflow(flags[1],V,clk,reset,ctrl[16]);

	flip_flop zero(flags[2],Z,clk,reset,ctrl[17]);

	//registru instructiuni
	i_register IR(opcode,instr_mem_out[12:8],clk,reset,1'b1);






	// asignare iesiri

	assign acc = Q1;
	assign  alu_r = Y;
	assign pc = counter_out;
	assign fl = flags;
	assign st_p = stack_ptr;
	assign ir = opcode;
	



endmodule//end mycpu

//INSTR_MEM

module instr_mem (data, addr, en);
   output [12:0] data;
   input 	en;
   input [7:0] addr;

   wire [13:0] 	odata;

 assign odata = (addr == 8'd0)   ? 13'b00001_10000000 /* MOV      */
               :(addr == 8'd1)   ? 13'b00010_01000000 // add
               :(addr == 8'd2)   ? 13'b01010_10000000 // jmp to 128
					:(addr == 8'd3)   ? 13'b01011_10000000 // call
					:(addr == 8'd128) ? 13'b01100_10000000 // ret
					

				  
						:13'b00000_00000000; //nop  


		
   assign data = en ? odata : 8'bz;   
endmodule//END INSTR_MEM


//IR
module i_register(Q,D,clk,reset,en);

	input[4:0] D;//intrare de date
	input clk,reset,en;

	//clk => semnal de ceas
	//reset => semnal de reset
	//en => semnal de enable --- en=1 D va fi retinut in registru


	//output registru
	output reg [4:0] Q;

	always @(posedge clk , posedge reset) 
		if(reset) 
			 Q <= 0;
		else if(en )
			 Q <= D;
   
endmodule//END IR


//REGISTER
module register(Q,D,clk,reset,en);

	input[7:0] D;//intrare de date
	input clk,reset,en;

	//clk => semnal de ceas
	//reset => semnal de reset
	//en => semnal de enable --- en=1 D va fi retinut in registru


	//output registru
	output reg [7:0] Q;

	always @(posedge clk , posedge reset) 
		if(reset) 
			 Q <= 0;
		else if(en )
			 Q <= D;
   
endmodule//END REGISTER


//FLIP FLOP
module flip_flop(Q,D,clk,reset,en);

	input D;//intrare de date
	input clk,reset,en;

	//clk => semnal de ceas
	//reset => semnal de reset
	//en => semnal de enable --- en=1 D va fi retinut in registru


	//output registru
	output reg Q;

	always @(posedge clk , posedge reset) 
		if(reset) 
			 Q <= 0;
		else if(en )
			 Q <= D;
   
endmodule // END FLIP FLOP
/*
module data_mem (data, addr, en);
   output [7:0] data;
   input  en;
   input [7:0] addr;

   wire [7:0]   odata;

   assign odata = (addr == 8'd0)   ? 8'd128   
                  :(addr == 8'd1)   ? 8'd32   
      
      : 8'd1;                       
   assign data = en ? odata : 8'bz;   
endmodule

*/

//DATA MEM

module data_mem( input [7:0] data,
			input [7:0] addr,
			input we,clk,
			output [7:0] q

);



	reg[7:0] ram[255:0];
	reg [7:0] addr_reg;

	always @(posedge clk)
	begin
		if(we)
			ram[addr] <= data;
		else
		addr_reg <= addr;
	end


	assign q = ram[addr_reg];


endmodule // data_mem	






/*******************
PROGRAM COUNTER

*******************/
module p_counter(input [7:0] data ,
   input clk,
    input inc,
    input ld_data, 
    input reset, 
    output[7:0] counter
    );

	//daca inc=1 -> q = q + 1
	//daca ld_data=1 -> q = d
	reg [7:0] counter_up;

	// up counter
	always @(posedge clk or posedge reset)
	begin
	if(reset)
	begin
	 counter_up <= 8'd0;
		
	end
	else if(ld_data == 1'b1)
		counter_up <= data;

	else if(inc == 1'b1)
	 counter_up <= counter_up + 8'd1;

	end 
	assign counter = counter_up;
	
endmodule//end pc

//MULTIPLEXOR 4 LA 1


module mux_4to1( input [7:0] a,//intrare1
                  input [7:0] b,//intrare2
                  input [7:0] c,//intrare3
                  input [7:0] d,//intrare4
                  input [1:0] sel,//selectie
                  output [7:0] out//iesire
						);
 
   // 00 -> a, 01 -> b, 10 -> c, 11 -> d
   assign out = sel[1] ? (sel[0] ? d : c) : (sel[0] ? b : a); 
 
endmodule // end mux 4 to 1


//MUX 2 To 1
module  mux2to1(
	din_0      , //intrare1
	din_1      , //intrare2
	sel        , //selectie
	mux_out      //iesire
);

	input [17:0] din_0, din_1;input sel ;

	output [17:0]mux_out;

	wire  mux_out;

	assign mux_out = (sel) ? din_1 : din_0;

endmodule //End Of Module mux



//Mux 32 To 1

module mux32to1(X,A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15,
A16,A17,A18,A19,A20,A21,A22,A23,A24,A25,A26,A27,A28,A29,A30,A31,S);//multiplexor 32 la 1
	output[17:0]X;//iesirea multiplexorului
	
	input[17:0]A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15,A16,
	A17,A18,A19,A20,A21,A22,A23,A24,A25,A26,A27,A28,A29,A30,A31;//intrarile multiplexorului
	
	input[4:0]S;//intrarea de selectie a multiplexorului
	
	reg X;
	
	always @(S or A0 or A1 or A2 or A3 or A4 or A5 or A6 or A7 or A8 or A9 
		or A10 or A11 or A12 or A13 or A14 or A15 
		or A16 or A17 or A18 or A19 or A20 or A21 or A22 or A23 or A24 or A25 or 
		A26 or A27 or A28 or A29 or A30 or A31)
		case({S})
			5'd0:X=A0;
			5'd1:X=A1;
			5'd2:X=A2;
			5'd3:X=A3;
			5'd5:X=A5;
			5'd5:X=A5;
			5'd6:X=A6;
			5'd7:X=A7;
			5'd8:X=A8;
			5'd9:X=A9;
			5'd10:X=A10;
			5'd11:X=A11;
			5'd12:X=A12;
			5'd13:X=A13;
			5'd15:X=A15;
			5'd15:X=A15;
			5'd16:X=A16;
			5'd17:X=A17;
			5'd18:X=A18;
			5'd19:X=A19;
			5'd20:X=A20;
			5'd21:X=A21;
			5'd22:X=A22;
			5'd23:X=A23;
			5'd24:X=A24;
			5'd25:X=A25;
			5'd26:X=A26;
			5'd27:X=A27;
			5'd28:X=A28;
			5'd29:X=A29;
			5'd30:X=A30;
			5'd31:X=A31;
			
		endcase
endmodule // end mux32to1


//Mux 2 To 1 // cu intrari pe 8 bit

module  mux2to1_8bit(
	din_0      , //intrare1
	din_1      , //intrare2
	sel        , //selectie
	mux_out      //iesire
	);

	input [7:0] din_0, din_1;input sel ;

	output [7:0]mux_out;

	wire  mux_out;

	assign mux_out = (sel) ? din_1 : din_0;

endmodule //End Of Module mux


//UNITATE ARITMETICO-LOGICA PE 8 BITI




module ual(Y, C,V, Z, A, B, Op);//unitatea aritmetico-logica pe 8 biti
   output[7:0]Y; //rezultat
   output C,V,Z;//indicatori de carry/borrow,overflow si zero  
   input [7:0]A,B; //operanzi
   input [2:0]Op; //operatie
   wire  [7:0]AS,And,Or,Xor,Not;//rezultatele modulelor individuale
   wire s; 
   wire Cr,Of;//carry/overflow temporar(deoarece carry si overflow 
			  //nu sunt valabile pentru operatiile logice)
   
   // Operatii
   sum8bit s8bit(AS, Cr,Of, A, B, Op[0]);// Op == 3'b000, 3'b001
   andop ual_and(And, A, B);             // Op == 3'b010
   orop ual_or(Or, A, B);                // Op == 3'b011
   xorop ual_xor(Xor, A, B);             // Op == 3'b100
   notop ual_not(Not, A);                // Op == 3'b101
   
   //multiplexor 8 la 1 care selecteaza operatia
   multiplexor8_1 mux81(Y, AS, AS, And, Or, Xor, Not, 8'b0, 8'b0, Op);
 
   nor(s, Op[1], Op[2]); //daca s=0 =>operatie logica,daca s=1 =>operatie aritmetica  
   and(C, Cr, s);//carry/borrow final
   and(V, Of, s);//overflow final         
   zero z(Z, Y); //indicator de zero(valabil tuturor functiilor)         
endmodule 

module andop(Y, A, B);//modul care calculeaza functia si logic
   output [7:0] Y; //rezultat 
   input [7:0]  A,B; //operanzi  
   and(Y[0], A[0], B[0]);
   and(Y[1], A[1], B[1]);
   and(Y[2], A[2], B[2]);
   and(Y[3], A[3], B[3]);
   and(Y[4], A[4], B[4]);
   and(Y[5], A[5], B[5]);
   and(Y[6], A[6], B[6]);
   and(Y[7], A[7], B[7]);
endmodule 

module orop(Y, A, B);//modul care calculeaza functia sau logic
   output [7:0] Y;//rezultat
   input  [7:0] A,B;//operanzi
   or(Y[0], A[0], B[0]);
   or(Y[1], A[1], B[1]);
   or(Y[2], A[2], B[2]);
   or(Y[3], A[3], B[3]);
   or(Y[4], A[4], B[4]);
   or(Y[5], A[5], B[5]);
   or(Y[6], A[6], B[6]);
   or(Y[7], A[7], B[7]);
endmodule 

module xorop(Y, A, B);//modul care calculeaza functia sau exclusiv
   output [7:0] Y;//rezultat 
   input  [7:0] A,B;//operanzi
   xor(Y[0], A[0], B[0]);
   xor(Y[1], A[1], B[1]);
   xor(Y[2], A[2], B[2]);
   xor(Y[3], A[3], B[3]);
   xor(Y[4], A[4], B[4]);
   xor(Y[5], A[5], B[5]);
   xor(Y[6], A[6], B[6]);
   xor(Y[7], A[7], B[7]);
endmodule 

module notop(Y, A);//modul care calculeaza functia nu logic
   output [7:0] Y;//rezultat 
   input [7:0]  A;//operand 
   not(Y[0], A[0]);
   not(Y[1], A[1]);
   not(Y[2], A[2]);
   not(Y[3], A[3]);
   not(Y[4], A[4]);
   not(Y[5], A[5]);
   not(Y[6], A[6]);
   not(Y[7], A[7]);
endmodule 

module sum8bit(S, C,V, A, B, Op);//sumator/scazator pe 8 biti
   output[7:0]S;//suma/diferenta
   output 	  C,V;//indicatori de carry/borrow si overflow   
   input [7:0]A,B;//operanzi  
   input 	  Op; //operatie(adunare/scadere,0/1) 
   
   wire C0,C1,C2,C3,C4,C5,C6,C7;
   wire B0,B1,B2,B3,B4,B5,B6,B7;

   xor(B0, B[0], Op);
   xor(B1, B[1], Op);
   xor(B2, B[2], Op);
   xor(B3, B[3], Op);
   xor(B4, B[4], Op);
   xor(B5, B[5], Op);
   xor(B6, B[6], Op);
   xor(B7, B[7], Op);
   xor(C, C7, Op); //calcul carry/borrow
   xor(V, C7, C6); //calcul overflow  
      
   //se folosesc 8 instante ale unui sumator pe 1 bit
   sum1bit s0(S[0], C0, A[0], B0, Op);    
   sum1bit s1(S[1], C1, A[1], B1, C0);
   sum1bit s2(S[2], C2, A[2], B2, C1);
   sum1bit s3(S[3], C3, A[3], B3, C2);
   sum1bit s4(S[4], C4, A[4], B4, C3);    
   sum1bit s5(S[5], C5, A[5], B5, C4);
   sum1bit s6(S[6], C6, A[6], B6, C5);
   sum1bit s7(S[7], C7, A[7], B7, C6);
   
endmodule 


module sum1bit(S, Cout, A, B, Cin);//sumator pe 1 bit
   output S;//rezultat
   output Cout;//carry out 
   input  A,B;//operanzi
   input  Cin;//carry in
   
   wire w1,w2,w3,w4;
  
   xor(w1, A, B);
   xor(S, Cin, w1);
   and(w2, A, B);   
   and(w3, A, Cin);
   and(w4, B, Cin);   
   or(Cout, w2, w3, w4);
   
endmodule 

module zero(Z, A);//modul care calculeaza indicatorul de zero
   output Z;//rezultat(0 sau 1)       
   input [7:0]  A;//operand
   wire  [7:0] 	Y;//variabila care mentine valoare lui xnor(A,0) 
   xnor(Y[0], A[0], 0);
   xnor(Y[1], A[1], 0);
   xnor(Y[2], A[2], 0);
   xnor(Y[3], A[3], 0);
   xnor(Y[4], A[4], 0);
   xnor(Y[5], A[5], 0);
   xnor(Y[6], A[6], 0);
   xnor(Y[7], A[7], 0);
   and(Z,Y[0],Y[1],Y[2],Y[3],Y[4],Y[5],Y[6],Y[7]);//calcularea efectiva a indicatorului de zero
endmodule 

 
module multiplexor8_1(X,A0,A1,A2,A3,A4,A5,A6,A7,S);//multiplexor 8 la 1
output[7:0]X;//iesirea multiplexorului
input[7:0]A0,A1,A2,A3,A4,A5,A6,A7;//intrarile multiplexorului
input[2:0]S;//intrarea de selectie a multiplexorului
reg X;
always @(S or A0 or A1 or A2 or A3 or A4 or A5 or A6 or A7 )
	case({S})
		3'd0:X=A0;//adunare
		3'd1:X=A1;//scadere
		3'd2:X=A2;//and
		3'd3:X=A3;//or
		3'd4:X=A4;//xor
		3'd5:X=A5;//not
		3'd6:X=A6;//operatie nedefinita,iesirea multiplexorului va fi 0
		3'd7:X=A7;//operatie nedefinita,iesirea multiplexorului va fi 0
		
	endcase
endmodule

//STIVA
module Stack (
  clk,
  reset,
  q,
  d,
  push,
  pop,
  sp
);
	//daca push=1 -> d se memoreaza in stiva
	//daca pop=1 -> q ia valoarea care se afla la adresa stack pointer

  parameter WIDTH = 8; // marime nivel
  parameter DEPTH = 8; // niveluri stiva

  input                    clk;
  input                    reset;
  input      [WIDTH - 1:0] d;
  output reg [WIDTH - 1:0] q;
  input                    push;
  input                    pop;
  output [DEPTH -1:0] sp;
  reg [DEPTH - 1:0] ptr;
  reg [WIDTH - 1:0] stack [0:(1 << DEPTH) - 1];

  always @(posedge clk) begin
    if (reset)
      ptr <= 0;
    else if (push)
      ptr <= ptr + 1;
    else if (pop)
      ptr <= ptr - 1;
  end

  always @(posedge clk) begin
    if (push || pop) begin
      if(push)
        stack[ptr] <= d;


      q <= stack[ptr - 1];
    end
  end


assign sp = ptr;

endmodule


/* Structura comanda




            +--------------------------------> 0-read_dmem//1-write_to_dmem
            | +------------------------------> en_alu  
            | |  +---------------------------> push/pop                  
            | |  | +-------------------------> load pc
            | |  | | +-----------------------> increment pc
            | |  | | |  +--------------------> ual op   //000-add // 001-sub 
            | |  | | |  |   +----------------> select 2 //00-w // 01-t //10-zero // 11-data_mem_out
            | |  | | |  |   |  +-------------> select 1 //00-w // 01-t //10-alu_out  // 11 -data_mem_out          
            | |  | | |  |   |  | +-----------> enable t 
            | |  | | |  |   |  | | +---------> enable w                             
            | |  | | |  |   |  | | |                      
            | |  | | |  |   |  | | |
            | |  | | |  |   |  | | |
      0_0_0_0_0_00_1_0_000_10_00_1_0
      | | |
      | | |
      | | +----------------------------------> enable carry
      | +------------------------------------> enable overflow
      +--------------------------------------> enable zero
  



                              */

module cu(
	
	input [4:0] opcode,
	input [2:0] fl,
    output [17:0] control
	);

	wire [17:0] jmp_zero,jmp_carry;
	wire [17:0] ctrl;


	parameter [17:0] NOP = 18'b000011111000000;//0 




	parameter [17:0] MOVFW = 18'b1_1_1_0_1_00_0_1_000_10_01_1_1;//1


	parameter [17:0] ADDWF = 18'b1_1_1_0_0_00_0_1_000_01_00_1_1;//2

	parameter [17:0] SUBWF = 18'b1_1_1_0_0_00_0_1_001_01_00_1_1;//3

	parameter [17:0] ANDWF = 18'b1_1_1_0_0_00_0_1_010_01_00_1_1;//4

	parameter [17:0] ORWF =  18'b1_1_1_0_0_00_0_1_011_01_00_1_1;//5

	parameter [17:0] XORWF = 18'b1_1_1_0_0_00_0_1_100_01_00_1_1;//6

	parameter [17:0] NOTW =  18'b1_1_1_0_0_00_0_1_101_01_00_0_1;//7

	parameter [17:0] JMP =   18'b1_1_1_0_0_00_1_1_000_01_00_1_0;//8

	mux2to1 cond_jmp1(18'b1_1_1_0_0_00_0_1_000_01_00_1_0,18'b1_1_1_0_0_00_1_1_000_01_00_1_0,fl[2],jmp_zero); //9
	mux2to1 cond_jmp2(18'b1_1_1_0_0_00_0_1_000_01_00_1_0,18'b1_1_1_0_0_00_1_1_000_01_00_1_0,fl[0],jmp_carry); //10


	parameter [17:0] CALL =  18'b0_0_0_0_0_10_1_1_000_01_00_1_0;// 11

	parameter [17:0] RET =   18'b0_0_0_0_0_01_1_1_000_01_00_1_0;//12

	parameter [17:0] MOVAW = 18'b1_1_1_0_0_00_0_1_000_10_11_1_1;//13


	parameter [17:0] ADDWA = 18'b1_1_1_0_0_00_0_1_000_11_00_1_1;//19

	parameter [17:0] SUBWA = 18'b1_1_1_0_0_00_0_1_001_11_00_1_1;//15

	parameter [17:0] ANDWA = 18'b1_1_1_0_0_00_0_1_010_11_00_1_1;//16

	parameter [17:0] ORWA =  18'b1_1_1_0_0_00_0_1_011_11_00_1_1;//17

	parameter [17:0] XORWA = 18'b1_1_1_0_0_00_0_1_100_11_00_1_1;//18

	parameter [17:0] WRT  =  18'b1_1_1_1_0_00_0_1_000_01_00_1_1;//20

	mux32to1 decoder(ctrl,NOP,MOVFW,ADDWF,SUBWF,ANDWF,ORWF,XORWF,NOTW,JMP,jmp_zero,jmp_carry,
		CALL,RET,MOVAW,ADDWA,SUBWA,ANDWA,ORWA,XORWA,ADDWA,WRT,,,,,,,,,,,,opcode);

	assign control = ctrl;







endmodule // cu

module simulare;
reg clk,reset;
wire[4:0] ir;
wire[2:0] fl;
wire[7:0] o1,o2,o3,pc,sp;
mycpu mc1(o1,o3,pc,sp,ir,fl,clk,reset);

initial
      clk=1'b1;
   always
   #1 clk=~clk;
   initial
      begin
    



            reset=1'b1;
        // #10  reset=1'b0;wr_i=1'b1;instr=13'b00000_00000000;
		 #20 reset=1'b0;
         #100   $finish;

      end
      initial
      begin
      
      $monitor($time," R:%d     W:%d   T:%d   Y:%d   PC %d  ZVC %b  SP %d",reset,o1,ir,o3,pc,fl,sp);
end



endmodule