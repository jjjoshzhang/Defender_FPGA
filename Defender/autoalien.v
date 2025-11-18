`default_nettype none

/*  This code displays a background image (MIF) and a single object, whose pixels
 *  are read from an object MIF in on-chip memory. The object automatically moves
 *  from right to left across the screen. No PS/2 keyboard is needed.
 *
 *  To use:
 *    - Press and release KEY[0] to reset.
 *    - The background MIF appears.
 *    - The sprite object starts near the right edge and keeps sliding left.
 */

module vga_demo(
    CLOCK_50,
    SW,
    KEY,
    LEDR,
    VGA_R,
    VGA_G,
    VGA_B,
    VGA_HS,
    VGA_VS,
    VGA_BLANK_N,
    VGA_SYNC_N,
    VGA_CLK
);
    // specify the number of bits needed for an X (column) pixel coordinate on the VGA display
    parameter nX = 10;
    // specify the number of bits needed for a Y (row) pixel coordinate on the VGA display
    parameter nY = 9;

    input  wire        CLOCK_50;
    input  wire [9:0]  SW;
    input  wire [3:0]  KEY;
    output wire [9:0]  LEDR;
    output wire [7:0]  VGA_R;
    output wire [7:0]  VGA_G;
    output wire [7:0]  VGA_B;
    output wire        VGA_HS;
    output wire        VGA_VS;
    output wire        VGA_BLANK_N;
    output wire        VGA_SYNC_N;
    output wire        VGA_CLK;

    wire Resetn;
    assign Resetn = KEY[0];          // active-high reset (KEY[0] low = reset)

    // sprite outputs
    wire [nX-1:0] sprite_x;
    wire [nY-1:0] sprite_y;
    wire [8:0]    sprite_color;
    wire          sprite_write;

    // instantiate the auto-moving sprite object
    object_auto O1 (
        .Resetn(Resetn),
        .Clock(CLOCK_50),
        .VGA_x(sprite_x),
        .VGA_y(sprite_y),
        .VGA_color(sprite_color),
        .VGA_write(sprite_write)
    );
        defparam O1.nX       = nX;
        defparam O1.nY       = nY;
        // 2^4 x 2^4 = 16 x 16 sprite
        defparam O1.xOBJ     = 4;
        defparam O1.yOBJ     = 4;
        // start near right edge, around middle vertically
        defparam O1.X_INIT   = 10'd640;
        defparam O1.Y_INIT   = 9'd240;
        // your sprite MIF file:
        defparam O1.INIT_FILE = "./MIF/spacecraft_32_32_9.mif";

    // connect to VGA controller
    vga_adapter VGA (
        .resetn(Resetn),
        .clock(CLOCK_50),
        .color(sprite_color),
        .x(sprite_x),
        .y(sprite_y),
        .write(sprite_write),
        .VGA_R(VGA_R),
        .VGA_G(VGA_G),
        .VGA_B(VGA_B),
        .VGA_HS(VGA_HS),
        .VGA_VS(VGA_VS),
        .VGA_BLANK_N(VGA_BLANK_N),
        .VGA_SYNC_N(VGA_SYNC_N),
        .VGA_CLK(VGA_CLK)
    );
        

    // no LEDs used
    assign LEDR = 10'b0;

endmodule

// ====================================================================
//  n-bit register with enable
// ====================================================================
module regn(R, Resetn, E, Clock, Q);
    parameter n = 8;
    input  wire [n-1:0] R;
    input  wire         Resetn, E, Clock;
    output reg  [n-1:0] Q;

    always @(posedge Clock)
        if (!Resetn)
            Q <= {n{1'b0}};
        else if (E)
            Q <= R;
endmodule

// ====================================================================
//  n-bit up/down-counter with reset, load, enable, and direction control
// ====================================================================
module upDn_count (R, Clock, Resetn, L, E, Dir, Q);
    parameter n = 8;
    input  wire [n-1:0] R;
    input  wire         Clock, Resetn, E, L, Dir;
    output reg  [n-1:0] Q;

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

// ====================================================================
//  Auto-moving object (sprite from MIF) that goes RIGHT -> LEFT
// ====================================================================
module object_auto(
    Resetn,
    Clock,
    VGA_x,
    VGA_y,
    VGA_color,
    VGA_write
);
    // VGA coordinate bit-widths
    parameter nX = 10;
    parameter nY = 9;

    // sprite size: 2^xOBJ by 2^yOBJ
    parameter xOBJ = 5;
    parameter yOBJ = 5;

    // initial position
    parameter X_INIT = 10'd630;
    parameter Y_INIT = 9'd240;

    // MIF file for sprite pixels
    parameter INIT_FILE = "./MIF/spacecraft_32_32_9.mif";

    // derived constants
    localparam BOX_SIZE_X = 1 << xOBJ;        // sprite width
    localparam BOX_SIZE_Y = 1 << yOBJ;        // sprite height
    localparam Mn         = xOBJ + yOBJ;      // address lines to cover sprite

    // erasure color (assumes black background)
    localparam [8:0] ALT = 9'b0;

    input  wire        Resetn;
    input  wire        Clock;
    output wire [nX-1:0] VGA_x;
    output wire [nY-1:0] VGA_y;
    output wire [8:0]  VGA_color;
    output wire        VGA_write;

    // ----------------------------------------------------------------
    // Slow counter to control movement speed
    // ----------------------------------------------------------------
    reg [22:0] slow;           // adjust width for different speeds
    wire       step;           // one-cycle pulse when we should move

    // 'step' becomes 1 when all bits are 1 (and then the counter wraps)
    assign step = &slow;

    always @(posedge Clock)
        if (!Resetn)
            slow <= 23'd0;
        else
            slow <= slow + 23'd1;

    // ----------------------------------------------------------------
    // Object position (X,Y) and sprite traversing counters (XC,YC)
    // ----------------------------------------------------------------
    wire [nX-1:0] X0, X;
    wire [nY-1:0] Y0, Y;
    wire [nX-1:0] size_x;
    wire [nY-1:0] size_y;

    assign X0     = X_INIT;
    assign Y0     = Y_INIT;
    assign size_x = BOX_SIZE_X;
    assign size_y = BOX_SIZE_Y;

    // X position moves left each cycle; Y stays essentially constant
    reg Lx, Ly, Ex, Ey;

    upDn_count UX (X0, Clock, Resetn, Lx, Ex, 1'b0 /*Dir=0 => decrement*/, X);
        defparam UX.n = nX;

    upDn_count UY (Y0, Clock, Resetn, Ly, Ey, 1'b1 /*Dir=1, but E=0 so no move*/, Y);
        defparam UY.n = nY;

    // sprite pixel counters
    wire [xOBJ-1:0] XC;
    wire [yOBJ-1:0] YC;
    reg  Lxc, Lyc, Exc, Eyc;

    upDn_count U3 ({xOBJ{1'd0}}, Clock, Resetn, Lxc, Exc, 1'b1, XC); // column counter
        defparam U3.n = xOBJ;
    upDn_count U4 ({yOBJ{1'd0}}, Clock, Resetn, Lyc, Eyc, 1'b1, YC); // row counter
        defparam U4.n = yOBJ;

    // ----------------------------------------------------------------
    // FSM to erase, move, and redraw the sprite
    // ----------------------------------------------------------------
    // state names
    localparam SA = 3'b000,
               SB = 3'b001,
               SC = 3'b010,
               SD = 3'b011,
               SE = 3'b100,
               SF = 3'b101,
               SG = 3'b110,
               SH = 3'b111;

    reg [2:0] y_Q, Y_D;

    reg erase;
    reg write;

    // FSM next-state logic
    always @(*)
        case (y_Q)
            SA:  Y_D = SB;                      // load initial (X,Y)
            SB:  if (!step) Y_D = SB;           // wait for step (movement tick)
                 else       Y_D = SC;
            SC:  if (XC != size_x-1) Y_D = SC;  // erase current row
                 else                  Y_D = SD;
            SD:  if (YC != size_y-1) Y_D = SC;  // erase next rows
                 else                  Y_D = SE;
            SE:  Y_D = SF;                      // move one step left
            SF:  if (XC != size_x-1) Y_D = SF;  // draw current row
                 else                  Y_D = SG;
            SG:  if (YC != size_y-1) Y_D = SF;  // draw next rows
                 else                  Y_D = SH;
            SH:  Y_D = SB;                      // done; wait for next step
            default: Y_D = SA;
        endcase

    // FSM outputs
    always @(*)
    begin
        // default assignments
        Lx   = 1'b0; Ly   = 1'b0; Ex   = 1'b0; Ey   = 1'b0;
        Lxc  = 1'b0; Lyc  = 1'b0; Exc  = 1'b0; Eyc  = 1'b0;
        erase = 1'b0;
        write = 1'b0;

        case (y_Q)
            SA: begin
                    Lx  = 1'b1;      // load initial X
                    Ly  = 1'b1;      // load initial Y
                end
            SB: begin
                    Lxc = 1'b1;      // load XC
                    Lyc = 1'b1;      // load YC
                end
            // erase sprite at old position
            SC: begin
                    Exc   = 1'b1;    // increment XC
                    write = 1'b1; 
                    erase = 1'b1;    // draw background (ALT)
                end
            SD: begin
                    Lxc   = 1'b1;    // reset XC
                    Eyc   = 1'b1;    // increment YC
                    erase = 1'b1;
                end
            // move one pixel left
            SE: begin
                    Ex = 1'b1;       // X-- (left)
                end
            // draw sprite at new position
            SF: begin
                    Exc   = 1'b1;    // increment XC
                    write = 1'b1;
                end
            SG: begin
                    Lxc = 1'b1;      // reset XC
                    Eyc = 1'b1;      // increment YC
                end
            SH: begin
                    // nothing; wait to go back to SB
                end
        endcase
    end

    // FSM state FFs
    always @(posedge Clock)
        if (!Resetn)
            y_Q <= SA;
        else
            y_Q <= Y_D;

    // ----------------------------------------------------------------
    // Sprite memory: returns color for each (XC,YC)
    // ----------------------------------------------------------------
    wire [8:0] obj_color;



    object_mem U6 (
        .address({YC, XC}),
        .clock(Clock),
        .q(obj_color)
    );
        defparam U6.n         = 9;        // 9-bit color sprite
        defparam U6.Mn        = Mn;       // Mn = xOBJ + yOBJ (e.g., 8 for 16x16)
        defparam U6.INIT_FILE = INIT_FILE;

    // ----------------------------------------------------------------
    // Compute (x,y) of current pixel and align with memory latency
    // ----------------------------------------------------------------
    // Add XC,YC to object origin, and center the sprite at (X,Y).
    // We register the computed coordinates and the write signal so
    // they stay aligned with the color from object_mem.
    regn U7 (X - (size_x >> 1) + {{(nX-xOBJ){1'b0}}, XC}, Resetn, 1'b1, Clock, VGA_x);
        defparam U7.n = nX;

    regn U8 (Y - (size_y >> 1) + {{(nY-yOBJ){1'b0}}, YC}, Resetn, 1'b1, Clock, VGA_y);
        defparam U8.n = nY;

    regn U9 ({write}, Resetn, 1'b1, Clock, VGA_write);
        defparam U9.n = 1;

    // Use ALT when erasing, otherwise sprite color
    assign VGA_color = erase ? ALT : obj_color;

endmodule