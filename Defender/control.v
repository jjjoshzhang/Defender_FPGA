`default_nettype none

/*  This code first displays a background image (MIF) on the VGA output. Then, the code
 *  displays ONE object, which is read from a small memory, on the screen. The object can be
 *  moved left/right/up/down by pressing PS2 keyboard keys (a/s/w/z – WASD-style).
 *
 *  To use the circuit:
 *    - Use KEY[0] to perform a reset. The background MIF should appear on the VGA output. 
 *    - Press KEY[1] once to display the object at its initial position.
 *    - Use PS2 keys a/s/w/z to move the object.
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

    // state names for the FSM that controls reading scancodes & stepping the object
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
    wire [nX-1:0] O1_x;           // x coordinate
    wire [nY-1:0] O1_y;           // y coordinate
    wire [8:0]    O1_color;       // color
    wire          O1_write;       // write enable
    wire          O1_done;        // object move/draw cycle done
    wire [1:0]    O1_dir;         // direction for the object

    // PS/2 handling + FSM
    reg  prev_ps2_clk;            // ps2_clk value in the previous clock cycle
    wire negedge_ps2_clk;         // used for PS2 keyboard signals
    wire ps2_rec;                 // set when a PS2 packet has been received
    reg  [32:0] Serial;           // PS2 serial data shift register
    reg  [3:0]  Packet;           // count bits in a PS2 frame
    wire [7:0]  scancode;         // saved PS2 scancode
    reg  Esc;                     // enable scancode register
    reg  step;                    // pulse used to move the object (one step per key press)
    reg  [1:0]  y_Q, Y_D;         // small FSM for ps2_rec -> step

    wire Resetn, KEY1;            // Reset, and synchronized version of KEY[1]
    wire PS2_CLK_S, PS2_DAT_S;    // synchronized versions of PS2 signals
    wire done;                    // indicates object cycle done (alias of O1_done)

    // Reset: KEY[0] is active-high "not reset"
    assign Resetn = KEY[0];

    // sync KEY[1] (active-low pushbutton)
    sync S1 (~KEY[1], Resetn, CLOCK_50, KEY1);

    // synchronize PS2 lines
    sync S3 (PS2_CLK, Resetn, CLOCK_50, PS2_CLK_S);
    sync S4 (PS2_DAT, Resetn, CLOCK_50, PS2_DAT_S);

    // record PS2 clock value in previous CLOCK_50 cycle
    always @(posedge CLOCK_50)
        prev_ps2_clk <= PS2_CLK_S;

    // check when PS2_CLK has changed from 1 to 0
    assign negedge_ps2_clk = (prev_ps2_clk & !PS2_CLK_S);

    // save PS2 data packet into Serial
    always @(posedge CLOCK_50) begin
        if (Resetn == 0)
            Serial <= 33'b0;
        else if (negedge_ps2_clk) begin
            Serial[31:0] <= Serial[32:1];
            Serial[32]   <= PS2_DAT_S;
        end
    end
        
    // 'count' ps2 data bits
    always @(posedge CLOCK_50) begin
        if (!Resetn || Packet == 'd11)
            Packet <= 'b0;
        else if (negedge_ps2_clk)
            Packet <= Packet + 'b1;
    end
        
    // used to detect when a full PS2 "scancode/release/scancode" is received
    assign ps2_rec = (Packet == 'd11) && (Serial[30:23] == Serial[8:1]);

    // ps2 scancode is in Serial[8:1]
    regn USC (Serial[8:1], Resetn, Esc, CLOCK_50, scancode);
    assign LEDR = {2'b0, scancode};

    // For the single player, the direction bits are just from scancode[1:0],
    // same mapping as original example (a/s/w/z).
    assign O1_dir = scancode[1:0];

    // === Small FSM: ps2_rec -> store scancode -> generate one-step pulse ===
    always @(*) begin
        case (y_Q)
            A:  if (!ps2_rec) Y_D = A;
                else          Y_D = B;
            B:  Y_D = C;        // enable scancode register
            C:  Y_D = D;        // send step signal to object
            D:  if (done == 1'b0) Y_D = D;
                else              Y_D = A;
            default: Y_D = A;
        endcase
    end

    // FSM outputs
    always @(*) begin
        // default assignments
        Esc  = 1'b0;
        step = 1'b0;
        case (y_Q)
            A:  ;          // idle
            B:  Esc  = 1'b1; // latch scancode
            C:  step = 1'b1; // one-cycle "move" pulse
            D:  ;          // wait for object to finish
        endcase
    end

    // FSM state FFs
    always @(posedge CLOCK_50)
        if (!Resetn)
            y_Q <= A;
        else
            y_Q <= Y_D;

    // === Single object instance ===
    // go  = KEY1  (press once to draw initially)
    // ps2_rec input to object = step (one pulse per move)
    object O1 (
        .Resetn   (Resetn),
        .Clock    (CLOCK_50),
        .go       (KEY1),
        .ps2_rec  (step),
        .dir      (O1_dir),
        .VGA_x    (O1_x),
        .VGA_y    (O1_y),
        .VGA_color(O1_color),
        .VGA_write(O1_write),
        .done     (O1_done)
    );
    defparam O1.LEFT  = 2'b00;  // 'a'
    defparam O1.RIGHT = 2'b11;  // 's'
    defparam O1.UP    = 2'b01;  // 'w'
    defparam O1.DOWN  = 2'b10;  // 'z'
    // You can also set:
    // defparam O1.XOFFSET = 320;
    // defparam O1.YOFFSET = 240;
    // defparam O1.INIT_FILE = "./MIF/your_sprite_16_16_9.mif";

    assign done = O1_done;

    // display PS2 data on HEX
    hex7seg H0 (Serial[4:1],   HEX0);
    hex7seg H1 (Serial[8:5],   HEX1);
    hex7seg H2 (Serial[15:12], HEX2);
    hex7seg H3 (Serial[19:16], HEX3);
    hex7seg H4 (Serial[26:23], HEX4);
    hex7seg H5 (Serial[30:27], HEX5);

    // connect directly to VGA controller with the single object's signals
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


