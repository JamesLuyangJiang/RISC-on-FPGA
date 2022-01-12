module datapath(clk, readnum, vsel, loada, loadb, shift, asel, bsel, ALUop, 
                loadc, loads, writenum, write, Z_out, datapath_out,
                mdata, sximm8, sximm5, PC, N_out, V_out, opcode);
    
    input clk, loada, loadb, asel, bsel, loadc, loads, write;
    input [3:0] vsel;
    input [1:0] shift, ALUop;
    input [2:0] readnum, writenum;
    input [15:0] mdata, sximm8, sximm5; //datapath_in;
    input [7:0] PC;
    input [2:0] opcode;

    output Z_out;
    output N_out;
    output V_out;
    
    output [15:0] datapath_out;

    reg [15:0] data_in;
    wire [15:0] A_out;
    wire [15:0] B_out;
    wire [15:0] A_ALU_in;
    wire [15:0] B_ALU_in;
    wire [15:0] data_out;
    wire [15:0] sout;
    wire [15:0] ALU_out;
    wire Z;
    wire N;
    wire V;
    wire [15:0] outwire;
    
    //------multiplexer-vsel--------//
    //assign data_in = vsel ? datapath_in : outwire;

    //------New-vsel-Mux--------//
    always @(*) begin
        case(vsel)
            4'b0001: data_in = mdata;
            4'b0010: data_in = sximm8;
            4'b0100: data_in = {8'b0, PC};
            4'b1000: data_in = outwire;
            default: data_in = 16'bx;
        endcase
    end

    regfile REGFILE(data_in, writenum, write, readnum, clk, data_out);

    vDFF1 A_block(clk, loada, data_out, A_out);
    vDFF1 B_block(clk, loadb, data_out, B_out);

    shifter SHIFTER(B_out, opcode, shift, sout);
    
    assign A_ALU_in = asel ? 16'b0 : A_out;
    assign B_ALU_in = bsel ? sximm5 : sout;

    ALU ALU(A_ALU_in, B_ALU_in, ALUop, ALU_out, Z, N, V);

    vDFF1 C_block(clk, loadc, ALU_out, datapath_out);

    vDFF1 #(1) status_blockZ(clk, loads, Z, Z_out);
    vDFF1 #(1) status_blockN(clk, loads, N, N_out);
    vDFF1 #(1) status_blockV(clk, loads, V, V_out);

    assign outwire = datapath_out;

endmodule


//-------------------------------Regfile------------------------------------//
module regfile(data_in,writenum,write,readnum,clk,data_out); 
    input [15:0] data_in;
    input [2:0] writenum, readnum; input write, clk;
    output [15:0] data_out;
    // fill out the rest

    wire [7:0]state;
    wire [7:0]multiwrite;
    wire [15:0]R0, R1, R2, R3, R4, R5, R6, R7;

    wire [7:0] writenum_Out;
    wire [7:0] readnum_Out;

    reg [15:0] data_out;

    DEC writeDecoder(writenum, writenum_Out);
    DEC readDecoder(readnum, readnum_Out);

    assign multiwrite = {8{write}};
    assign state = multiwrite & writenum_Out;

    vDFF1 R_0(clk,state[0],data_in,R0);
    vDFF1 R_1(clk,state[1],data_in,R1);
    vDFF1 R_2(clk,state[2],data_in,R2);
    vDFF1 R_3(clk,state[3],data_in,R3);
    vDFF1 R_4(clk,state[4],data_in,R4);
    vDFF1 R_5(clk,state[5],data_in,R5);
    vDFF1 R_6(clk,state[6],data_in,R6);
    vDFF1 R_7(clk,state[7],data_in,R7);

    always @(*) begin
        case (readnum_Out)
            8'b00000001 : data_out = R0;
            8'b00000010 : data_out = R1;
            8'b00000100 : data_out = R2;
            8'b00001000 : data_out = R3;
            8'b00010000 : data_out = R4;
            8'b00100000 : data_out = R5;
            8'b01000000 : data_out = R6;
            8'b10000000 : data_out = R7;
            default: data_out = {16{1'bx}};
        endcase
    end

endmodule

module vDFF1 (clk,load,in,out);
    parameter n = 16;

    input [n-1:0] in;
    input clk,load;
    output [n-1:0] out;

    reg [n-1:0] out;
    wire [n-1:0] next_out;

    assign next_out= load ? in : out;

    always @(posedge clk) begin
        out=next_out;
    end
endmodule

module DEC (in,out);
parameter n = 3;
parameter m = 8;
    input [n-1:0] in;
    output [m-1:0] out;
    wire [m-1:0] out=1<<in;
endmodule


//-------------------------------ALU------------------------------------//
module ALU(Ain,Bin,ALUop,out,Z,N,V); 
    input [15:0] Ain, Bin;
    input [1:0] ALUop;
    output [15:0] out;
    output Z;
    output N;
    output reg V;
    // fill out the rest

    reg [16:0] overflow;

    reg [15:0] out;
    always @(*) begin
        case (ALUop)
            2'b00:  begin
                        out = Ain + Bin;
                        overflow = Ain + Bin;
                        V = overflow[16];
                    end
            2'b01:  begin
                        out = Ain - Bin;
                        overflow = Ain - Bin;
                        V = overflow[16];
                    end
            2'b10:  begin
                        out = Ain & Bin;
                        overflow = {16{1'b0}};
                        V = 0;
                    end
            2'b11:  begin
                        out = ~Bin;
                        overflow = {16{1'b0}};
                        V = 0;
                    end
            default: begin
                        out = {16{1'bx}};
                        overflow = {17{1'bx}};
                    end
        endcase
    end

    assign Z = ~(|out);
    assign N = out[15];

endmodule 


//-------------------------------Shifter------------------------------------//
module shifter(in,opcode,shift,sout); 
    input [15:0] in;
    input [1:0] shift;
    input [2:0] opcode;
    output [15:0] sout;
    // fill out the rest
    reg [15:0] sout;
    
    always @(*) begin
        if (opcode == 3'b100) begin
            sout = in;
        end
        else begin 
            case (shift)
                2'b00: sout = in;
                2'b01: sout = in<<1;
                2'b10: sout = in>>1;
                2'b11: sout = {in[15], in[15:1]};
                default: sout = {16{1'bx}};
            endcase
        end
    end
endmodule 