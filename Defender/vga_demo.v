`default_nettype none

module vga_demo(
    CLOCK_50, SW, KEY, LEDR, PS2_CLK, PS2_DAT,
    HEX5, HEX4, HEX3, HEX2, HEX1, HEX0,
    VGA_R, VGA_G, VGA_B,
    VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK
);
    parameter nX = 10;
    parameter nY = 9;

    input  wire        CLOCK_50;
    input  wire [9:0]  SW;
    input  wire [3:0]  KEY;
    output wire [9:0]  LEDR;
    inout  wire        PS2_CLK, PS2_DAT;
    output wire [6:0]  HEX5, HEX4, HEX3, HEX2, HEX1, HEX0;
    output wire [7:0]  VGA_R;
    output wire [7:0]  VGA_G;
    output wire [7:0]  VGA_B;
    output wire        VGA_HS;
    output wire        VGA_VS;
    output wire        VGA_BLANK_N;
    output wire        VGA_SYNC_N;
    output wire        VGA_CLK;

    wire Resetn;
    assign Resetn = KEY[0];

    // Key sync
    wire KEY1_sync, KEY2_sync;
    sync S1 (~KEY[1], Resetn, CLOCK_50, KEY1_sync);
    sync S2 (~KEY[2], Resetn, CLOCK_50, KEY2_sync);

    // Fire pulse from KEY[3]
    wire KEY3_sync;
    reg  KEY3_sync_d;
    wire fire_pulse;

    sync S3 (~KEY[3], Resetn, CLOCK_50, KEY3_sync);
    always @(posedge CLOCK_50) begin
        if (!Resetn)
            KEY3_sync_d <= 1'b0;
        else
            KEY3_sync_d <= KEY3_sync;
    end
    assign fire_pulse = KEY3_sync & ~KEY3_sync_d;

    // Spacecraft wires
    wire [nX-1:0] sc_x;
    wire [nY-1:0] sc_y;
    wire [8:0]    sc_color;
    wire          sc_write;
    wire [nX-1:0] sc_obj_x;
    wire [nY-1:0] sc_obj_y;
    wire [7:0]    sc_scancode;

    spacecraft SC1(
        .Clock        (CLOCK_50),
        .Resetn       (Resetn),
        .go           (KEY1_sync),
        .PS2_CLK      (PS2_CLK),
        .PS2_DAT      (PS2_DAT),
        .VGA_x        (sc_x),
        .VGA_y        (sc_y),
        .VGA_color    (sc_color),
        .VGA_write    (sc_write),
        .obj_x        (sc_obj_x),
        .obj_y        (sc_obj_y),
        .scancode_out (sc_scancode)
    );

    // Score / hit wires
    wire hit1, hit2, hit3, hit4;
    wire missile_hit = hit1 | hit2 | hit3 | hit4;

    wire [7:0] score;

    score score_count(
        .Clock       (CLOCK_50),
        .Resetn      (Resetn),
        .missile_hit (missile_hit),
        .score       (score)
    );

    // NOTE: hex7seg expects 4 bits, so use lower nibble of score
    hex7seg H2 (score[3:0], HEX2);

    // Alien wires
    wire [nX-1:0] al_x;
    wire [nY-1:0] al_y;
    wire [8:0]    al_color;
    wire          al_write;
    wire [nX-1:0] al_obj_x;
    wire [nY-1:0] al_obj_y;
    wire          al1_alive;

    alien AL1(
        .Clock     (CLOCK_50),
        .Resetn    (Resetn),
        .go        (KEY2_sync),
        .hit       (hit1),
        .alive     (al1_alive),
        .VGA_x     (al_x),
        .VGA_y     (al_y),
        .VGA_color (al_color),
        .VGA_write (al_write),
        .obj_x     (al_obj_x),
        .obj_y     (al_obj_y)
    );

    // Missile wires
    wire [nX-1:0] missile_x;
    wire [nY-1:0] missile_y;
    wire [8:0]    missile_color;
    wire          missile_write;
    wire          missile_active;
    wire [nX-1:0] missile_posX;
    wire [nY-1:0] missile_posY;

    missile M1(
        .Clock     (CLOCK_50),
        .Resetn    (Resetn),
        .fire      (fire_pulse),
        .hit       (missile_hit),
        .player_x  (sc_obj_x),
        .player_y  (sc_obj_y),
        .VGA_x     (missile_x),
        .VGA_y     (missile_y),
        .VGA_color (missile_color),
        .VGA_write (missile_write),
        .active    (missile_active),
        .posX      (missile_posX),
        .posY      (missile_posY)
    );

    // Collision
    collision C1(
        .missile_x (missile_posX),
        .missile_y (missile_posY),
        .alien_x   (al_obj_x),
        .alien_y   (al_obj_y),
        .alive     (al1_alive),
        .hit       (hit1)
    );

    // Simple priority mux: missile > spacecraft > alien
    wire [nX-1:0] MUX_x;
    wire [nY-1:0] MUX_y;
    wire [8:0]    MUX_color;
    wire          MUX_write;

    assign MUX_x = missile_write ? missile_x :
                   sc_write     ? sc_x      :
                   al_write     ? al_x      :
                   {nX{1'b0}};

    assign MUX_y = missile_write ? missile_y :
                   sc_write     ? sc_y      :
                   al_write     ? al_y      :
                   {nY{1'b0}};

    assign MUX_color = missile_write ? missile_color :
                       sc_write     ? sc_color      :
                       al_write     ? al_color      :
                       9'b0;

    assign MUX_write = missile_write | sc_write | al_write;

    assign LEDR = {2'b00, sc_scancode};

    hex7seg H0 (sc_scancode[3:0], HEX0);
    hex7seg H1 (sc_scancode[7:4], HEX1);
    assign HEX3 = 7'b1111111;
    assign HEX4 = 7'b1111111;
    assign HEX5 = 7'b1111111;

    vga_adapter VGA (
        .resetn      (Resetn),
        .clock       (CLOCK_50),
        .color       (MUX_color),
        .x           (MUX_x),
        .y           (MUX_y),
        .write       (MUX_write),
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


//============================================================
//  SPACECRAFT (player, controlled by PS2)
//============================================================

//============================================================
//  SPACECRAFT (player, controlled by PS2)
//============================================================
module spacecraft(
    Clock, Resetn, go, PS2_CLK, PS2_DAT,
    VGA_x, VGA_y, VGA_color, VGA_write, obj_x, obj_y, scancode_out
);
    parameter nX = 10;
    parameter nY = 9;

    input  wire Clock;
    input  wire Resetn;
    input  wire go;
    inout  wire PS2_CLK;
    inout  wire PS2_DAT;
    output wire [nX-1:0] VGA_x;
    output wire [nY-1:0] VGA_y;
    output wire [8:0]    VGA_color;
    output wire          VGA_write;
    output wire [nX-1:0] obj_x;
    output wire [nY-1:0] obj_y;
    output wire [7:0]    scancode_out;

    wire PS2_CLK_S, PS2_DAT_S;
    sync S0 (PS2_CLK, Resetn, Clock, PS2_CLK_S);
    sync S1 (PS2_DAT, Resetn, Clock, PS2_DAT_S);

    reg  prev_ps2_clk;
    wire negedge_ps2_clk;

    always @(posedge Clock)
        prev_ps2_clk <= PS2_CLK_S;

    assign negedge_ps2_clk = prev_ps2_clk & ~PS2_CLK_S;

    reg  [32:0] Serial;
    reg  [3:0]  Packet;

    always @(posedge Clock) begin
        if (!Resetn)
            Serial <= 33'b0;
        else if (negedge_ps2_clk) begin
            Serial[31:0] <= Serial[32:1];
            Serial[32]   <= PS2_DAT_S;
        end
    end

    always @(posedge Clock) begin
        if (!Resetn || Packet == 4'd11)
            Packet <= 4'd0;
        else if (negedge_ps2_clk)
            Packet <= Packet + 4'd1;
    end

    wire ps2_rec;
    assign ps2_rec = (Packet == 4'd11) && (Serial[30:23] == Serial[8:1]);

    wire [7:0] scancode;
    reg  Esc;
    reg  step;
    reg  [1:0] y_Q, Y_D;

    // latch last valid scancode when Esc is 1
    regn USS (Serial[8:1], Resetn, Esc, Clock, scancode);
    assign scancode_out = scancode;

    // Decode WASD directions
    reg [1:0] dir_reg;
    always @(*) begin
        case (scancode)
            8'h1D: dir_reg = 2'b01; // W up
            8'h1B: dir_reg = 2'b10; // S down
            8'h1C: dir_reg = 2'b00; // A left
            8'h23: dir_reg = 2'b11; // D right
            default: dir_reg = 2'b00;
        endcase
    end
    wire [1:0] dir = dir_reg;

    wire wasd_key;
    assign wasd_key = (scancode == 8'h1D) ||
                      (scancode == 8'h1B) ||
                      (scancode == 8'h1C) ||
                      (scancode == 8'h23);

    // Small FSM to capture one step per key event
    localparam S_A = 2'b00;
    localparam S_B = 2'b01;
    localparam S_C = 2'b10;
    localparam S_D = 2'b11;

    wire obj_done;

    // state transitions
    always @(*) begin
        case (y_Q)
            S_A: if (!ps2_rec) Y_D = S_A; else Y_D = S_B;
            S_B: Y_D = S_C;
            S_C: Y_D = S_D;
            S_D: if (!obj_done) Y_D = S_D; else Y_D = S_A;
            default: Y_D = S_A;
        endcase
    end

    // control: latch scancode (Esc) and generate one movement step (step)
    always @(*) begin
        Esc  = 1'b0;
        step = 1'b0;
        case (y_Q)
            S_A: begin
                // idle: waiting for ps2_rec in FSM logic
            end

            S_B: begin
                // only store the byte if it's NOT F0 (break prefix)
                if (Serial[8:1] != 8'hF0)
                    Esc = 1'b1;
            end

            S_C: begin
                // generate one movement pulse per valid WASD make code
                if (wasd_key)
                    step = 1'b1;
            end

            S_D: begin
                // wait for object to finish drawing (obj_done)
            end

            default: ;
        endcase
    end

    always @(posedge Clock) begin
        if (!Resetn)
            y_Q <= S_A;
        else
            y_Q <= Y_D;
    end

    // Player object instance
    object O1(
        .Resetn    (Resetn),
        .Clock     (Clock),
        .go        (go),
        .ps2_rec   (step),
        .dir       (dir),
        .hit       (1'b0),
        .alive     (),          // not used for player
        .init_y    (9'd240),
        .VGA_x     (VGA_x),
        .VGA_y     (VGA_y),
        .VGA_color (VGA_color),
        .VGA_write (VGA_write),
        .done      (obj_done),
        .obj_x     (obj_x),
        .obj_y     (obj_y)
    );

    // Make spacecraft move faster: 4 pixels per step
    defparam O1.UX.STEP = 5;
    defparam O1.UY.STEP = 5;

endmodule


//============================================================
//  ALIEN (automatic movement)
//============================================================
module alien(
    Clock, Resetn, go, hit, alive,
    VGA_x, VGA_y, VGA_color, VGA_write, obj_x, obj_y
);
    parameter nX = 10;
    parameter nY = 9;

    input  wire Clock;
    input  wire Resetn;
    input  wire go;
    input  wire hit;
    output wire alive;
    output wire [nX-1:0] VGA_x;
    output wire [nY-1:0] VGA_y;
    output wire [8:0]    VGA_color;
    output wire          VGA_write;
    output wire [nX-1:0] obj_x;
    output wire [nY-1:0] obj_y;

    // Slow counter controls how often the alien moves
    reg [20:0] slow;   // 21 bits => faster than original 23 bits
    wire       step;

    always @(posedge Clock) begin
        if (!Resetn || go)
            slow <= 21'd0;
        else
            slow <= slow + 21'd1;
    end

    assign step = &slow;  // pulse when all bits = 1

    wire [1:0] dir;
    assign dir = 2'b00;   // always moving LEFT

    wire dummy_done;
    wire [8:0] init_y;

    random_y R1(
        .Clock        (Clock),
        .Resetn       (!Resetn),
        .alien_init_y (init_y)
    );

    object O2(
        .Resetn    (Resetn),
        .Clock     (Clock),
        .go        (go),
        .ps2_rec   (step),
        .dir       (dir),
        .hit       (hit),
        .alive     (alive),
        .init_y    (init_y),
        .VGA_x     (VGA_x),
        .VGA_y     (VGA_y),
        .VGA_color (VGA_color),
        .VGA_write (VGA_write),
        .done      (dummy_done),
        .obj_x     (obj_x),
        .obj_y     (obj_y)
    );

    defparam O2.XOFFSET   = 10'd600;
    defparam O2.YOFFSET   = 9'd240;
    defparam O2.xOBJ      = 5;
    defparam O2.yOBJ      = 5;
    defparam O2.INIT_FILE = "./MIF/alien_32_32_9.mif";
endmodule


//============================================================
//  OBJECT (generic sprite: spacecraft or alien)
//============================================================
module object(
    Resetn, Clock, go, ps2_rec, dir, hit, alive, init_y,
    VGA_x, VGA_y, VGA_color, VGA_write, done,
    obj_x, obj_y
);
    parameter nX = 10;
    parameter nY = 9;
    parameter XOFFSET = 320;
    parameter YOFFSET = 240;
    parameter LEFT  = 2'b00;
    parameter RIGHT = 2'b11;
    parameter UP    = 2'b01;
    parameter DOWN  = 2'b10;
    parameter xOBJ = 5;
    parameter yOBJ = 5;
    parameter BOX_SIZE_X = 1 << xOBJ;
    parameter BOX_SIZE_Y = 1 << yOBJ;
    parameter Mn = xOBJ + yOBJ;
    parameter INIT_FILE = "./MIF/spacecraft_32_32_9.mif";

    parameter A = 3'b000;
    parameter B = 3'b001;
    parameter C = 3'b010;
    parameter D = 3'b011;
    parameter E = 3'b100;
    parameter F = 3'b101;
    parameter G = 3'b110;
    parameter H = 3'b111;
    
    input  wire Resetn;
    input  wire Clock;
    input  wire go;
    input  wire ps2_rec;
    input  wire [1:0] dir;
    input  wire hit;
    input  wire [nY-1:0] init_y;

    output wire [nX-1:0] VGA_x;
    output wire [nY-1:0] VGA_y;
    output wire [8:0]    VGA_color;
    output wire          VGA_write;
    output reg           done;

    output wire [nX-1:0] obj_x;
    output wire [nY-1:0] obj_y;
    output reg           alive;

    wire [nX-1:0] X;
    wire [nX-1:0] X0;
    wire [nY-1:0] Y;
    wire [nY-1:0] Y0;
    wire [nX-1:0] size_x;
    wire [nY-1:0] size_y;
    wire [xOBJ-1:0] XC;
    wire [yOBJ-1:0] YC;
    reg  write;
    reg  Lxc;
    reg  Lyc;
    reg  Exc;
    reg  Eyc;
    reg  erase;
    wire Right;
    wire Left;
    wire Up;
    wire Down;
    reg  Lx;
    reg  Ly;
    reg  Ex;
    reg  Ey;
    reg  [2:0] y_Q;
    reg  [2:0] Y_D;
    wire [8:0] obj_color;
    reg  kill;
    
    assign X0 = XOFFSET;
    assign Y0 = init_y; // random/init Y
    assign size_x = BOX_SIZE_X;
    assign size_y = BOX_SIZE_Y;

    // Position counters (STEP overridden only for O1 in spacecraft)
    upDn_count UX (X0, Clock, Resetn, Lx, Ex, Right, X);
        defparam UX.n = nX;

    upDn_count UY (Y0, Clock, Resetn, Ly, Ey, Down, Y);
        defparam UY.n = nY;

    // Counters for indexing sprite memory
    upDn_count U3 ({xOBJ{1'd0}}, Clock, Resetn, Lxc, Exc, 1'b1, XC);
        defparam U3.n = xOBJ;

    upDn_count U4 ({yOBJ{1'd0}}, Clock, Resetn, Lyc, Eyc, 1'b1, YC);
        defparam U4.n = yOBJ;

    assign Left  = (dir == LEFT);
    assign Right = (dir == RIGHT);
    assign Up    = (dir == UP);
    assign Down  = (dir == DOWN);

    // FSM
    always @(*) begin
        case (y_Q)
            A:  Y_D = B;
            B:  if (go)          Y_D = F;
                else if (kill)   Y_D = C;
                else if (ps2_rec)Y_D = C;
                else             Y_D = B;
            C:  if (XC != size_x-1) Y_D = C;
                else                Y_D = D;
            D:  if (YC != size_y-1) Y_D = C;
                else                Y_D = E;
            E:  Y_D = F;
            F:  if (XC != size_x-1) Y_D = F;
                else                Y_D = G;
            G:  if (YC != size_y-1) Y_D = F;
                else                Y_D = H;
            H:  if (go) Y_D = H;
                else     Y_D = B;
            default: Y_D = A;
        endcase
    end

    always @(*) begin
        Lx = 1'b0;
        Ly = 1'b0;
        Ex = 1'b0;
        Ey = 1'b0;
        write = 1'b0;
        Lxc = 1'b0;
        Lyc = 1'b0;
        Exc = 1'b0;
        Eyc = 1'b0;
        erase = 1'b0;
        done = 1'b0;

        case (y_Q)
            A:  begin Lx = 1'b1; Ly = 1'b1; end
            B:  begin Lxc = 1'b1; Lyc = 1'b1; end
            C:  begin Exc = 1'b1; write = 1'b1; erase = 1'b1; end
            D:  begin Lxc = 1'b1; Eyc = 1'b1; erase = 1'b1; end
            E:  begin 
                    if (alive && !kill && !hit) begin
                        Ex = Right | Left; 
                        Ey = Up | Down; 
                    end
                end
            F:  begin 
                    Exc = 1'b1; 
                    write = 1'b1; 
                    if (!alive || kill)
                        erase = 1'b1;
                end
            G:  begin Lxc = 1'b1; Eyc = 1'b1; end
            H:  done = 1'b1;
        endcase
    end

    always @(posedge Clock) begin
        if (!Resetn)
            y_Q <= 3'b000;
        else
            y_Q <= Y_D;
    end

    // Alive / kill logic (for alien use)
    always @(posedge Clock) begin
        if (!Resetn) begin
            alive <= 1'b1;
            kill  <= 1'b0;
        end
        else begin
            if (hit && alive && (y_Q == B))
                kill <= 1'b1;

            // only set alive=0 when FSM reaches H and kill=1
            if ((y_Q == H) && kill) begin 
                alive <= 1'b0;
                kill  <= 1'b0;
            end
        end
    end

    // Sprite memory
    object_mem U6 ({YC,XC}, Clock, obj_color);
        defparam U6.n         = 9;
        defparam U6.Mn        = xOBJ + yOBJ;
        defparam U6.INIT_FILE = INIT_FILE;

    // Output registers to VGA
    regn U7 (X - (size_x >> 1) + XC, Resetn, 1'b1, Clock, VGA_x);
        defparam U7.n = nX;

    regn U8 (Y - (size_y >> 1) + YC, Resetn, 1'b1, Clock, VGA_y);
        defparam U8.n = nY;

    regn U9 (write, Resetn, 1'b1, Clock, VGA_write);
        defparam U9.n = 1;

    assign VGA_color = erase ? {9{1'b0}} : obj_color;
    assign obj_x = X;
    assign obj_y = Y;
endmodule


//============================================================
//  MISSILE
//============================================================
module missile(
    Clock, Resetn, fire, hit,
    player_x, player_y,
    VGA_x, VGA_y, VGA_color, VGA_write, active, posX, posY
);
    parameter nX = 10;
    parameter nY = 9;

    input  wire Clock;
    input  wire Resetn;
    input  wire fire;
    input  wire [nX-1:0] player_x;
    input  wire [nY-1:0] player_y;
    input  wire hit;

    output wire [nX-1:0] VGA_x;
    output wire [nY-1:0] VGA_y;
    output wire [8:0]    VGA_color;
    output wire          VGA_write;
    output wire          active;
    output reg  [nX-1:0] posX; 
    output reg  [nY-1:0] posY;

    parameter ScreenX          = 640;
    parameter missile_length   = 25;
    parameter missile_thickness= 3;
    parameter N                = 16;

    parameter [8:0] RED   = 9'b111000000;
    parameter [8:0] BLACK = 9'b0; 

    parameter [nX-1:0] Max_X = ScreenX;

    reg [nX-1:0] XC; 
    reg [nY-1:0] YC; 

    reg erase;
    reg write;

    wire sync;
    reg [N-1:0] cnt;

    reg [3:0] state;
    reg [3:0] next_state;

    localparam IDLE    = 4'b0000;
    localparam INITIAL = 4'b0001;
    localparam DRAW    = 4'b0010;
    localparam WAIT    = 4'b0011;
    localparam ERASE   = 4'b0100;
    localparam MOVE    = 4'b0101;
    localparam DONE    = 4'b0110;

    assign active = (state != IDLE) && (state != DONE);

    always @(posedge Clock) begin 
        if (!Resetn)
            cnt <= {N{1'b0}};
        else 
            cnt <= cnt + 1'b1;
    end
    
    assign sync = (cnt == {N{1'b1}});

    assign VGA_x     = posX + XC;
    assign VGA_y     = posY + YC;
    assign VGA_color = erase ? BLACK : RED;
    assign VGA_write = write;

    // FSM control
    always @(*) begin
        next_state = state;
        write      = 1'b0;
        erase      = 1'b0;

        case (state)
            IDLE: begin
                if (fire)
                    next_state = INITIAL;
            end
            INITIAL: begin
                next_state = DRAW;
            end
            DRAW: begin 
                write = 1'b1;
                erase = 1'b0;
                if ((XC == missile_length-1) && (YC == missile_thickness-1))
                    next_state = WAIT;
                else
                    next_state = DRAW;
            end 
            WAIT: begin 
                if (hit)
                    next_state = ERASE;
                else if (sync)
                    next_state = ERASE;
                else
                    next_state = WAIT;
            end 
            ERASE: begin 
                write = 1'b1;
                erase = 1'b1;
                if ((XC == missile_length-1) && (YC == missile_thickness-1))
                    if (hit)
                        next_state = DONE;
                    else
                        next_state = MOVE;
                else
                    next_state = ERASE;
            end
            MOVE: begin
                if (posX < Max_X)
                    next_state = DRAW; 
                else
                    next_state = DONE;
            end
            DONE: begin 
                if (fire)
                    next_state = INITIAL;
            end
            default: begin 
                next_state = IDLE;
            end
        endcase
    end

    // Data path
    always @(posedge Clock) begin
        if (!Resetn) begin
            state <= IDLE;
            posX  <= player_x + 16;
            posY  <= player_y;
            XC    <= 0;
            YC    <= 0;
        end
        else begin
            state <= next_state;
            case (state)
                IDLE: begin
                    XC <= 0;
                    YC <= 0;
                    if (fire) begin 
                        posX <= player_x + 16;
                        posY <= player_y;
                    end
                end
                INITIAL: begin
                    posX <= player_x + 16;
                    posY <= player_y;
                    XC   <= 0;
                    YC   <= 0;
                end
                DRAW: begin 
                    if (XC < missile_length-1) begin
                        XC <= XC + 1'b1;
                    end
                    else begin 
                        XC <= 0;
                        if (YC < missile_thickness-1) begin
                            YC <= YC + 1'b1;
                        end
                        else 
                            YC <= 0;
                    end
                end
                ERASE: begin 
                    if (XC < missile_length-1) begin
                        XC <= XC + 1'b1;
                    end
                    else begin
                        XC <= 0;
                        if (YC < missile_thickness-1) begin
                            YC <= YC + 1'b1;
                        end
                        else begin
                            YC <= 0;
                        end
                    end
                end
                MOVE: begin
                    if (posX < Max_X)
                        posX <= posX + 1'b1;
                end
            endcase
        end
    end
endmodule


//============================================================
//  Basic utility modules
//============================================================
module sync(D, Resetn, Clock, Q);
    input  wire D;
    input  wire Resetn;
    input  wire Clock;
    output reg  Q;

    reg Qi;

    always @(posedge Clock) begin
        if (!Resetn) begin
            Qi <= 1'b0;
            Q  <= 1'b0;
        end
        else begin
            Qi <= D;
            Q  <= Qi;
        end
    end
endmodule

module regn(R, Resetn, E, Clock, Q);
    parameter n = 8;
    input  wire [n-1:0] R;
    input  wire         Resetn;
    input  wire         E;
    input  wire         Clock;
    output reg  [n-1:0] Q;

    always @(posedge Clock) begin
        if (!Resetn)
            Q <= {n{1'b0}};
        else if (E)
            Q <= R;
    end
endmodule

module upDn_count(R, Clock, Resetn, L, E, Dir, Q);
    parameter n    = 8;
    parameter STEP = 1;  // how many pixels per step

    input  wire [n-1:0] R;
    input  wire         Clock;
    input  wire         Resetn;
    input  wire         L;
    input  wire         E;
    input  wire         Dir;
    output reg  [n-1:0] Q;

    always @(posedge Clock) begin
        if (!Resetn)
            Q <= {n{1'b0}};
        else if (L)
            Q <= R;
        else if (E) begin
            if (Dir)
                Q <= Q + STEP; // move STEP pixels
            else
                Q <= Q - STEP;
        end
    end
endmodule

module hex7seg(hex, display);
    input  wire [3:0] hex;
    output reg  [6:0] display;

    always @(*) begin
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
    end
endmodule


//============================================================
//  COLLISION + SCORE
//============================================================
module collision(
    missile_x,
    missile_y,
    alien_x,
    alien_y,
    alive,
    hit
);
    parameter nX = 10;
    parameter nY = 9;
    parameter missile_length   = 25;
    parameter missile_thickness= 3;
    parameter alien_width      = 32;
    parameter alien_height     = 32;

    input wire [nX-1:0] missile_x;
    input wire [nY-1:0] missile_y;
    input wire [nX-1:0] alien_x;  // center of alien
    input wire [nY-1:0] alien_y;
    input wire alive;
    output wire hit;

    // missile bounds
    wire [nX-1:0] missile_left   = missile_x; 
    wire [nX-1:0] missile_right  = missile_x + (missile_length - 1);
    wire [nY-1:0] missile_top    = missile_y;
    wire [nY-1:0] missile_bottom = missile_y + (missile_thickness - 1);

    // alien bounds (alien_x, alien_y is center -> shift by 16)
    wire [nX-1:0] alien_left   = alien_x - 16; 
    wire [nX-1:0] alien_right  = alien_x + (alien_width - 1) - 16;
    wire [nY-1:0] alien_top    = alien_y - 16;
    wire [nY-1:0] alien_bottom = alien_y + (alien_height - 1) - 16;

    wire collision = (missile_left   <= alien_right ) &&
                     (missile_right  >= alien_left  ) &&
                     (missile_top    <= alien_bottom) &&
                     (missile_bottom >= alien_top   );

    assign hit = alive && collision;
endmodule


module score (
    Clock,
    Resetn,
    missile_hit,
    score
);
    input wire Clock;
    input wire Resetn;
    input wire missile_hit;

    output reg [7:0] score;

    reg missile_hit_delay;
    
    always @(posedge Clock) begin
        if (!Resetn) begin   
            score            <= 8'd0;
            missile_hit_delay<= 1'b0;
        end
        else begin
            missile_hit_delay <= missile_hit;

            // count rising edges of missile_hit
            if (missile_hit && !missile_hit_delay)
                score <= score + 8'd1;
        end
    end
endmodule


//============================================================
//  RANDOM Y generator for alien start position
//============================================================
module random_y (
    Clock, 
    Resetn, 
    alien_init_y
);
    input  wire Clock;
    input  wire Resetn;
    output reg [8:0] alien_init_y;

    wire [15:0] seed;
    wire [8:0]  rand;

    lfsr_fib_16 #(.INITIAL_SEED(16'hBEEF)) RNG (
        .reset(~Resetn),
        .clk  (Clock), 
        .seed (seed)
    );

    assign rand = seed[8:0];

    always @(posedge Clock) begin
        if (!Resetn)
            alien_init_y <= 9'd240;
        else if (rand < 9'd16)
            alien_init_y <= 9'd16;
        else if (rand > 9'd464)
            alien_init_y <= 9'd464;
        else
            alien_init_y <= rand;
    end
endmodule    

module lfsr_fib_16 #(parameter INITIAL_SEED = 16'hBEEF) (
    input  wire reset,
    input  wire clk,
    output reg  [15:0] seed
);
    wire next_bit;
    assign next_bit = ((seed[15] ^ seed[13]) ^ seed[12]) ^ seed[10];

    always @(posedge clk) begin
        if (reset)
            seed <= INITIAL_SEED;
        else
            seed <= {seed[14:0], next_bit};
    end
endmodule