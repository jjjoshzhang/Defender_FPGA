`default_nettype none

/*  This code displays a background image (MIF) on the VGA output, then displays ONE
 *  movable object (sprite) read from a small memory. The object is controlled with:
 *
 *        W = up,  S = down,  A = left,  D = right
 *
 *  Usage:
 *    - Use KEY[0] (not pressed = 1) as reset. When released, VGA background appears.
 *    - Press KEY[1] once (active-low) to display the object at its initial position.
 *    - Use W/A/S/D on the PS2 keyboard to move the object.
 */

module vga_demo(
    CLOCK_50, SW, KEY, LEDR, PS2_CLK, PS2_DAT,
    HEX5, HEX4, HEX3, HEX2, HEX1, HEX0,
    VGA_R, VGA_G, VGA_B,
    VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK
);

    // specify the number of bits needed for an X (column) pixel coordinate on the VGA display
    parameter nX = 10;
    // specify the number of bits needed for a Y (row) pixel coordinate on the VGA display
    parameter nY = 9;

    // simple 2-bit FSM for PS2 handling / move step control
    parameter A = 2'b00, B = 2'b01, C = 2'b10, D = 2'b11;

    input  wire CLOCK_50;	
    input  wire [9:0] SW;
    input  wire [3:0] KEY;
    output wire [9:0] LEDR;
    inout  wire PS2_CLK, PS2_DAT;
    output wire [6:0] HEX5, HEX4, HEX3, HEX2, HEX1, HEX0;
    output wire [7:0] VGA_R;
    output wire [7:0] VGA_G;
    output wire [7:0] VGA_B;
    output wire VGA_HS;
    output wire VGA_VS;
    output wire VGA_BLANK_N;
    output wire VGA_SYNC_N;
    output wire VGA_CLK;	

    // *** Single object signals ***
    wire [nX-1:0] O1_x;
    wire [nY-1:0] O1_y;
    wire [8:0]    O1_color;
    wire          O1_write;
    wire          O1_done;
    wire [1:0]    O1_dir;

    // PS/2 handling + small FSM
    reg  prev_ps2_clk;
    wire negedge_ps2_clk;
    wire ps2_rec;
    reg  [32:0] Serial;
    reg  [3:0]  Packet;
    wire [7:0]  scancode;
    reg  Esc;
    reg  step;
    reg  [1:0]  y_Q, Y_D;

    wire Resetn, KEY1;
    wire PS2_CLK_S, PS2_DAT_S;
    wire done;

    // Reset is KEY[0]; active-high "not reset"
    assign Resetn = KEY[0];

    // KEY[1] is used as "go" (spawn object) – synchronize and invert because button is active-low
    sync S1 (~KEY[1], Resetn, CLOCK_50, KEY1);

    // synchronize PS2 CLK and DAT
    sync S3 (PS2_CLK, Resetn, CLOCK_50, PS2_CLK_S);
    sync S4 (PS2_DAT, Resetn, CLOCK_50, PS2_DAT_S);

    // record previous PS2 clock
    always @(posedge CLOCK_50)
        prev_ps2_clk <= PS2_CLK_S;

    // detect falling edge on PS2 clock
    assign negedge_ps2_clk = (prev_ps2_clk & !PS2_CLK_S);

    // shift in PS2 serial data
    always @(posedge CLOCK_50) begin
        if (Resetn == 0)
            Serial <= 33'b0;
        else if (negedge_ps2_clk) begin
            Serial[31:0] <= Serial[32:1];
            Serial[32]   <= PS2_DAT_S;
        end
    end

    // count PS2 bits
    always @(posedge CLOCK_50) begin
        if (!Resetn || Packet == 'd11)
            Packet <= 'b0;
        else if (negedge_ps2_clk)
            Packet <= Packet + 'b1;
    end

    // detect "complete packet" (scancode/release/scancode)
    assign ps2_rec = (Packet == 'd11) && (Serial[30:23] == Serial[8:1]);

    // latch scancode (Serial[8:1]) when Esc=1
    regn USC (Serial[8:1], Resetn, Esc, CLOCK_50, scancode);
    assign LEDR = {2'b0, scancode};

    // === WASD mapping ===
    // Set of valid movement keys:
    //   W = 0x1D, S = 0x1B, A = 0x1C, D = 0x23
    reg [1:0] O1_dir_reg;

    always @(*) begin
        case (scancode)
            8'h1D: O1_dir_reg = 2'b01; // W -> UP
            8'h1B: O1_dir_reg = 2'b10; // S -> DOWN
            8'h1C: O1_dir_reg = 2'b00; // A -> LEFT
            8'h23: O1_dir_reg = 2'b11; // D -> RIGHT
            default: O1_dir_reg = 2'b00;
        endcase
    end

    assign O1_dir = O1_dir_reg;

    wire wasd_key;
    assign wasd_key = (scancode == 8'h1D) || // W
                      (scancode == 8'h1B) || // S
                      (scancode == 8'h1C) || // A
                      (scancode == 8'h23);   // D

    // Small FSM around ps2_rec and object "done"
    always @(*) begin
        case (y_Q)
            A:  if (!ps2_rec) Y_D = A;
                else          Y_D = B;
            B:  Y_D = C;        // latch scancode
            C:  Y_D = D;        // give step pulse (if WASD)
            D:  if (done == 1'b0) Y_D = D;
                else              Y_D = A;
            default: Y_D = A;
        endcase
    end

    // FSM outputs
    always @(*) begin
        Esc  = 1'b0;
        step = 1'b0;
        case (y_Q)
            A:  ;               // idle
            B:  Esc = 1'b1;     // latch scancode
            C:  if (wasd_key)
                    step = 1'b1; // one move step for valid WASD key
            D:  ;               // wait for object to finish
        endcase
    end

    // FSM state FFs
    always @(posedge CLOCK_50)
        if (!Resetn)
            y_Q <= A;
        else
            y_Q <= Y_D;

    // === Single object instance ===
    //  go      = KEY1 (spawn / initial draw)
    //  ps2_rec = step (one pulse per valid WASD key press)
    object O1 (
        .Resetn    (Resetn),
        .Clock     (CLOCK_50),
        .go        (KEY1),
        .ps2_rec   (step),
        .dir       (O1_dir),
        .VGA_x     (O1_x),
        .VGA_y     (O1_y),
        .VGA_color (O1_color),
        .VGA_write (O1_write),
        .done      (O1_done)
    );
    // Direction encoding inside object:
    // LEFT, RIGHT, UP, DOWN
    defparam O1.LEFT  = 2'b00;
    defparam O1.RIGHT = 2'b11;
    defparam O1.UP    = 2'b01;
    defparam O1.DOWN  = 2'b10;

    // You can customize these for your sprite:
    // defparam O1.XOFFSET   = 320;
    // defparam O1.YOFFSET   = 240;
    // defparam O1.INIT_FILE = "./MIF/your_sprite_16_16_9.mif";

    assign done = O1_done;

    // display bits of Serial on HEX displays (same as original)
    hex7seg H0 (Serial[4:1],   HEX0);
    hex7seg H1 (Serial[8:5],   HEX1);
    hex7seg H2 (Serial[15:12], HEX2);
    hex7seg H3 (Serial[19:16], HEX3);
    hex7seg H4 (Serial[26:23], HEX4);
    hex7seg H5 (Serial[30:27], HEX5);

    // connect to VGA controller
    vga_adapter VGA (
        .resetn      (Resetn),
        .clock       (CLOCK_50),
        .color       (O1_color),
        .x           (O1_x),
        .y           (O1_y),
        .write       (O1_write),
        .VGA_R       (VGA_R),
        .VGA_G       (VGA_G),
        .VGA_B       (VGA_B),
        .VGA_HS      (VGA_HS),
        .VGA_VS      (VGA_VS),
        .VGA_BLANK_N (VGA_BLANK_N),
        .VGA_SYNC_N  (VGA_SYNC_N),
        .VGA_CLK     (VGA_CLK)
    );

endmodule


// ================== SUPPORT MODULES (same as your original) ==================

// synchronizer, implemented as two FFs in series
module sync(D, Resetn, Clock, Q);
    input wire D;
    input wire Resetn, Clock;
    output reg Q;

    reg Qi; // internal node

    always @(posedge Clock)
        if (Resetn == 0) begin
            Qi <= 1'b0;
            Q  <= 1'b0;
        end
        else begin
            Qi <= D;
            Q  <= Qi;
        end
endmodule

// n-bit register with enable
module regn(R, Resetn, E, Clock, Q);
    parameter n = 8;
    input wire [n-1:0] R;
    input wire Resetn, E, Clock;
    output reg [n-1:0] Q;

    always @(posedge Clock)
        if (!Resetn)
            Q <= 0;
        else if (E)
            Q <= R;
endmodule

// n-bit up/down-counter with reset, load, enable, and direction control
module upDn_count (R, Clock, Resetn, L, E, Dir, Q);
    parameter n = 8;
    input wire [n-1:0] R;
    input wire Clock, Resetn, E, L, Dir;
    output reg [n-1:0] Q;

    always @ (posedge Clock)
        if (Resetn == 0)
            Q <= {n{1'b0}};
        else if (L == 1)
            Q <= R;
        else if (E)
            if (Dir)
                Q <= Q + {{n-1{1'b0}},1'b1};
            else
                Q <= Q - {{n-1{1'b0}},1'b1};
endmodule

module hex7seg (hex, display);
    input wire [3:0] hex;
    output reg [6:0] display;

    /*
     *       0  
     *      ---  
     *     |   |
     *    5|   |1
     *     | 6 |
     *      ---  
     *     |   |
     *    4|   |2
     *     |   |
     *      ---  
     *       3  
     */
    always @ (hex)
        case (hex)
            4'h0: display = 7'b1000000;
            4'h1: display = 7'b1111001;
            4'h2: display = 7'b0100100;
            4'h3: display = 7'b0110000;
            4'h4: display = 7'b0011001;
            4'h5: display = 7'b0010010;
            4'h6: display = 7'b0000010;
            4'h7: display = 7'b1111000;
            4'h8: display = 7'b0000000;
            4'h9: display = 7'b0011000;
            4'hA: display = 7'b0001000;
            4'hB: display = 7'b0000011;
            4'hC: display = 7'b1000110;
            4'hD: display = 7'b0100001;
            4'hE: display = 7'b0000110;
            4'hF: display = 7'b0001110;
        endcase
endmodule

// implements a movable object
module object (
    Resetn, Clock, go, ps2_rec, dir,
    VGA_x, VGA_y, VGA_color, VGA_write, done
);
    // specify the number of bits needed for an X (column) pixel coordinate on the VGA display
    parameter nX = 10;
    // specify the number of bits needed for a Y (row) pixel coordinate on the VGA display
    parameter nY = 9;
    // by default, use offsets to center the object on the VGA display
    parameter XOFFSET = 320;
    parameter YOFFSET = 240;
    parameter LEFT  = 2'b00;
    parameter RIGHT = 2'b11;
    parameter UP    = 2'b01;
    parameter DOWN  = 2'b10;
    parameter xOBJ = 4, yOBJ = 4;   // object size is 2^xOBJ x 2^yOBJ
    parameter BOX_SIZE_X = 1 << xOBJ;
    parameter BOX_SIZE_Y = 1 << yOBJ;
    parameter Mn = xOBJ + yOBJ;     // address lines needed for the object memory
    parameter INIT_FILE = "./MIF/object_mem_16_16_9.mif";

    // state names for the FSM that draws the object
    parameter A = 3'b000, B = 3'b001, C = 3'b010, D = 3'b011,
              E = 3'b100, F = 3'b101, G = 3'b110, H = 3'b111;
    
    input wire Resetn, Clock;
    input wire go;                  // draw object at initial position
    input wire ps2_rec;             // PS2 data received (one step pulse)
    input wire [1:0] dir;           // movement direction
    output wire [nX-1:0] VGA_x;     // pixel X
    output wire [nY-1:0] VGA_y;     // pixel Y
    output wire [8:0]   VGA_color;  // pixel color
    output wire         VGA_write;  // pixel write control
    output reg          done;       // done drawing cycle

    wire [nX-1:0] X, X0;
    wire [nY-1:0] Y, Y0;
    wire [nX-1:0] size_x = BOX_SIZE_X;
    wire [nY-1:0] size_y = BOX_SIZE_Y;
    wire [xOBJ-1:0] XC;
    wire [yOBJ-1:0] YC;
    reg write, Lxc, Lyc, Exc, Eyc;
    reg erase;
    wire Right, Left, Up, Down;
    reg Lx, Ly, Ex, Ey;
    reg [2:0] y_Q, Y_D;

    wire [8:0] obj_color;

    // object (x,y) location counters
    assign X0 = XOFFSET;
    assign Y0 = YOFFSET;

    upDn_count UX (X0, Clock, Resetn, Lx, Ex, Right, X);
        defparam UX.n = nX;

    upDn_count UY (Y0, Clock, Resetn, Ly, Ey, Down, Y);
        defparam UY.n = nY;

    // counters for object pixels (local coordinates)
    upDn_count U3 ({xOBJ{1'd0}}, Clock, Resetn, Lxc, Exc, 1'b1, XC);
        defparam U3.n = xOBJ;

    upDn_count U4 ({yOBJ{1'd0}}, Clock, Resetn, Lyc, Eyc, 1'b1, YC);
        defparam U4.n = yOBJ;

    // direction decoding
    assign Left  = (dir == LEFT);
    assign Right = (dir == RIGHT);
    assign Up    = (dir == UP);
    assign Down  = (dir == DOWN);

    // FSM state table
    always @(*) begin
        case (y_Q)
            A:  Y_D = B;                       // load (x,y) counters
            B:  if (go)       Y_D = F;        // KEY to show object
                else if (ps2_rec) Y_D = C;    // PS2 move command
                else           Y_D = B;
            C:  if (XC != size_x-1) Y_D = C;  // erase row
                else                Y_D = D;
            D:  if (YC != size_y-1) Y_D = C;  // next row erase
                else                Y_D = E;  // done erase
            E:  Y_D = F;                       // move and draw
            F:  if (XC != size_x-1) Y_D = F;  // draw row
                else                Y_D = G;
            G:  if (YC != size_y-1) Y_D = F;  // next row draw
                else                Y_D = H;  // done draw
            H:  if (go) Y_D = H;              // wait for KEY release
                else     Y_D = B;
            default: Y_D = A;
        endcase
    end

    // FSM outputs
    always @(*) begin
        // default
        Lx = 1'b0; Ly = 1'b0; Ex = 1'b0; Ey = 1'b0;
        write = 1'b0;
        Lxc = 1'b0; Lyc = 1'b0; Exc = 1'b0; Eyc = 1'b0;
        erase = 1'b0;
        done = 1'b0;

        case (y_Q)
            A:  begin Lx = 1'b1; Ly = 1'b1; end
            B:  begin Lxc = 1'b1; Lyc = 1'b1; end
            C:  begin Exc = 1'b1; write = 1'b1; erase = 1'b1; end
            D:  begin Lxc = 1'b1; Eyc = 1'b1; erase = 1'b1; end
            E:  begin Ex = Right | Left; Ey = Up | Down; end
            F:  begin Exc = 1'b1; write = 1'b1; end
            G:  begin Lxc = 1'b1; Eyc = 1'b1; end
            H:  done = 1'b1;
        endcase
    end

    // FSM state FFs
    always @(posedge Clock)
        if (!Resetn)
            y_Q <= 3'b0;
        else
            y_Q <= Y_D;

    // read pixel from object memory (ROM)
    object_mem U6 ({YC,XC}, Clock, obj_color);
        defparam U6.n   = 9;
        defparam U6.Mn  = xOBJ + yOBJ;
        defparam U6.INIT_FILE = INIT_FILE;

    // compute absolute VGA (x,y) by centering sprite around (X,Y)
    regn U7 (X - (size_x >> 1) + XC, Resetn, 1'b1, Clock, VGA_x);
        defparam U7.n = nX;

    regn U8 (Y - (size_y >> 1) + YC, Resetn, 1'b1, Clock, VGA_y);
        defparam U8.n = nY;

    // sync write with VGA_x, VGA_y, VGA_color 
    regn U9 (write, Resetn, 1'b1, Clock, VGA_write);
        defparam U9.n = 1;

    // erase = draw black, else draw sprite color
    assign VGA_color = erase ? {9{1'b0}} : obj_color;

endmodule