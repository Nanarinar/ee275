module my_float_calc #( parameter s = 2'b01) (x,y,opcode,result,clk);

parameter	REG_SIZE = s[1] ? 64: 	(s[0] ? 32: 16);
parameter	EXP_SIZE = s[1] ? 11: 	(s[0] ? 8: 5);
parameter	FRA_SIZE = s[1] ? 52: 	(s[0] ? 23: 10);
parameter	BIAS	 = s[1] ? 1023: (s[0] ? 127: 15);

input clk, opcode; //opcode : 0 for add, 1 for multiply operation;
input [REG_SIZE-1:0] x,y; // size: 16 for half, 32 for single, 64 for double
output wire [REG_SIZE-1:0] result;

//wires to add and mult blocks
wire [REG_SIZE-1:0] x_add, y_add, x_mult, y_mult; 
//wire from add and mult blocks containing result
wire [REG_SIZE-1:0] r_mult, r_add;

//route the input values to approprite module via custom demux
my_demux #(.REG_SIZE(REG_SIZE)) demux_module(.Data_in1(x), .Data_in2(y), .sel(opcode), .Data_out_0(x_add), .Data_out_1(y_add), .Data_out_2(x_mult), .Data_out_3(y_mult));

// multiplier module
my_float_mult #(.s(s)) mult_module(.x(x_mult),.y(y_mult),.r(r_mult),.clk(clk));

//adder module
my_float_adder #(.s(s)) add_module(.x(x_add),.y(y_add),.r(r_add),.clk(clk));

//choose the appropriate output
assign result = opcode ? r_mult :r_add;

endmodule


/* ``````````````````````````````````````````````````````TESTBENCH MODULE``````````````````````````````````````````````` */
module my_float_calc_tb ();
parameter 	SIZE = 32,
			SIZE_MOD = (SIZE == 64) ? 2'b10:((SIZE ==32) ? 2'b01:2'b00);
reg [SIZE-1:0] x,y;
reg opcode;
wire [SIZE-1:0] r;
reg clk;

always #2 clk = ~clk;
my_float_calc #(.s(SIZE_MOD)) m1(.x(x),.y(y),.opcode(opcode),.result(r),.clk(clk));

initial begin
clk = 0;
opcode = 1; // 0 for add, 1 for mult
#2
//test case for half-precision (SIZE = 16)
/*
x = 16'h42B6; // 3.355
y = 16'h4D06; // 20.1 multiplied should result in or 5437
			  // 	  added should result in 4DDD
#5
if (opcode) $display("%h * %h= %h",x, y, r); 
else		$display("%h + %h= %h",x, y, r);
*/

/*x = 16'hb5ae; // -0.355
y = 16'h4D06; // 20.1 should result in -7.133 or 0xc722
#5
$display("%h * %h= %h",x, y, r);*/

//test case for single precision (SIZE = 32)

x = 'h420a449c;//34.567
y = 'hc2f6e666;//-123.45 multiplied should be -4267.2964 or c5855a5e 
			   //        added      should be -88.882996 or C2B1C418
#20
if (opcode) $display("%h * %h= %h",x, y, r); 
else		$display("%h + %h= %h",x, y, r); 


//test case for double precision (Size = 64)

/*
x = 'h407C81C6A7EF9DB2;// 456.111
y = 'hC0743CA031CEAF25;// -323.78911 should be -147683.77475121 or C102071E32B0C32C
#10
if (opcode) $display("%h * %h= %h",x, y, r); 
else		$display("%h + %h= %h",x, y, r);
*/

$finish;
end

initial begin

    $dumpfile("my_calc.vcd");
    $dumpvars();

end
endmodule

/* ``````````````````````````````````````````````````````ADDER MODULE``````````````````````````````````````````````` */
/*s = 	00 - half
		01 - single
		10 - double*/
	
module my_float_adder #( parameter s = 2'b01 ) (x,y,r,clk);

parameter	REG_SIZE = s[1] ? 64: (s[0] ? 32: 16);
parameter	EXP_SIZE = s[1] ? 11: (s[0] ? 8: 5);
parameter	FRA_SIZE = s[1] ? 52: (s[0] ? 23: 10);
parameter	BIAS	 = s[1] ? 1023: (s[0] ? 127: 15);

input clk;
input [REG_SIZE-1:0] x,y; // 16 for half, 32 for single, 64 for double
output reg [REG_SIZE-1:0] r;
reg s1,s2,sr; //sign bits, 1 for all sizes
reg [EXP_SIZE-1:0] e1,e2,er,abs_e1,abs_e2; // 5 bits for half, 8 bits for single, 11 for double
reg [FRA_SIZE:0] f1_t,f2_t;
reg [EXP_SIZE-1:0] exp_diff, mod_exp_diff;
reg [FRA_SIZE+1:0] sum,save;
reg [FRA_SIZE-1:0] f1,f2;

// read the numbers into appropriate representation
/* if half-precision use s|eeee_e|ffff_ffff_ff
if single-rprecision use s|eeee_eeee|ffff_ffff_ffff_ffff_ffff_fff
if douple precision use 1|11|52*/ 


always @(posedge clk)
begin
s1 = x[REG_SIZE-1];
s2 = y[REG_SIZE-1];
e1 = x[REG_SIZE-2:REG_SIZE-EXP_SIZE-1]-BIAS;
e2 = y[REG_SIZE-2:REG_SIZE-EXP_SIZE-1]-BIAS;
f1 = x[FRA_SIZE-1:0];
f2 = y[FRA_SIZE-1:0];
f1_t = {1'b1, f1};
f2_t = {1'b1, f2};
abs_e1= e1[EXP_SIZE-1] ? -e1 : e1;
abs_e2= e2[EXP_SIZE-1] ? -e2 : e2;
exp_diff = e1-e2;
exp_diff = exp_diff[EXP_SIZE-1] ? -exp_diff : exp_diff; 
if(abs_e1>abs_e2) begin
	f2_t = f2_t >> exp_diff;
	sr = s1;
	er = e1+BIAS;
end

else if(abs_e1<abs_e2) begin
	f1_t = f1_t >> exp_diff;
	sr = s2;
	er = e2+BIAS; 
end

else begin
	sr = (f1>f2) ? s1 : s2;
	er = e1+BIAS;
end

if (s1==1 && s2==0) begin
	f1_t = ~f1_t;
	f1_t = f1_t+1;
end

if (s1==0 && s2==1) begin
	f2_t = ~f2_t;
	f2_t = f2_t+1;
end

sum = f1_t+f2_t;

if (sr==1 && s1!=s2) sum = -sum; //if the bigger value of inputs is negative while the other is positive -  complement the result
if (s1 != s2) sum[FRA_SIZE+1] = 1'b0; // drop the carry if adding numbers with different sign bits

//normalize by rotating left or right depending on carry
while (sum[FRA_SIZE]==0 || sum[FRA_SIZE+1]!=0) 
begin
	if (sum[FRA_SIZE+1]==1) begin 
		sum = sum >> 1;
		er = er+1;
	end
	else begin
		sum = sum << 1;
		er = er-1;
	end
end

r = {sr, er,  sum[FRA_SIZE-1:0]}; //rebuild the final answer into reg r

end
endmodule



/* ``````````````````````````````````````````````````````MULT MODULE``````````````````````````````````````````````` */
module my_float_mult #( parameter s = 1 ) (x,y,r,clk);
//s = 	00 - half 01 - single 10 - double	
parameter	REG_SIZE = s[1] ? 64: 	(s[0] ? 32: 16);
parameter	EXP_SIZE = s[1] ? 11: 	(s[0] ? 8: 5);
parameter	FRA_SIZE = s[1] ? 52: 	(s[0] ? 23: 10);
parameter	BIAS	 = s[1] ? 1023: (s[0] ? 127: 15);

input clk;
integer i;
input [REG_SIZE-1:0] x,y; // 16 for half, 32 for single, 64 for double
output reg [REG_SIZE-1:0] r;
reg s1,s2,sr; //sign bits, 1 for all sizes
reg [EXP_SIZE-1:0] e1,e2,er; // 5 bits for half, 8 bits for single, 11 for double
reg [FRA_SIZE:0] f1,f2;
reg [EXP_SIZE-1:0] exp_diff, mod_exp_diff;
reg [2*FRA_SIZE+1:0] f_mult;

/* read the numbers into appropriate representation
if half-precision use s|eeee_e|ffff_ffff_ff
if single-rprecision use s|eeee_eeee|ffff_ffff_ffff_ffff_ffff_fff
if douple precision use 1|11|52
*/ 

always @(posedge clk)
begin
//break into sign-exponent-fraction form
s1 = x[REG_SIZE-1];
s2 = y[REG_SIZE-1];
e1 = x[REG_SIZE-2:REG_SIZE-EXP_SIZE-1];
e2 = y[REG_SIZE-2:REG_SIZE-EXP_SIZE-1];
f1 = {1'b1, x[FRA_SIZE-1:0]};
f2 = {1'b1, y[FRA_SIZE-1:0]};
//sign of the result
sr = s1^s2;
//exponent of the result
er = e1+e2-BIAS;
//multiply the fractional parts with appended 1
f_mult = 0;
for (i = 0; i < FRA_SIZE+1; i = i+1) begin
	if (f2[i]==1) begin
	f_mult = f_mult + (f1<<i);
	end
end
f_mult = f1*f2;
//normalize
if (f_mult[FRA_SIZE*2+1]==1) begin 
	f_mult = f_mult >> 1;
	er = er+1'b1;
end
//rebuild the final answer into reg r

r = {sr, er, f_mult[FRA_SIZE*2-1:FRA_SIZE]};
end
endmodule




/* ``````````````````````````````````````````````````````CUSTOM DEMUX MODULE``````````````````````````````````````````````` */
//Verilog module for 1:4 DEMUX with default size 32 fro registers
module my_demux #( parameter REG_SIZE = 32) (
     Data_in1,
	 Data_in2,
     sel,
    Data_out_0,
    Data_out_1,
    Data_out_2,
    Data_out_3
    );

//inputs and their sizes
    input [REG_SIZE-1:0] Data_in1, Data_in2;
    input sel;
//outputs and their sizes 
    output [REG_SIZE-1:0] Data_out_0;
    output [REG_SIZE-1:0] Data_out_1;
    output [REG_SIZE-1:0] Data_out_2;
    output [REG_SIZE-1:0] Data_out_3;
//Internal variables
    reg [REG_SIZE-1:0] Data_out_0;
    reg [REG_SIZE-1:0] Data_out_1;
    reg [REG_SIZE-1:0] Data_out_2;
    reg [REG_SIZE-1:0] Data_out_3;  

//always block with Data_in and sel in its sensitivity list
    always @(Data_in1 or Data_in2 or sel)
    begin
        case (sel)  
            2'b0 : begin
						$display("chose add");
                        Data_out_0 = Data_in1;
                        Data_out_1 = Data_in2;
                        Data_out_2 = {REG_SIZE{1'bz}};
                        Data_out_3 = {REG_SIZE{1'bz}};
                      end
            2'b1 : begin
                        Data_out_0 = {REG_SIZE{1'bz}};
                        Data_out_1 = {REG_SIZE{1'bz}};
                        Data_out_2 = Data_in1;
                        Data_out_3 = Data_in2;
                      end

        endcase
    end
    
endmodule