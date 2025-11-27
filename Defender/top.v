`default_nettype none

module top(
    CLOCK_50, SW, KEY, LEDR, PS2_CLK, PS2_DAT,
    HEX5, HEX4, HEX3, HEX2, HEX1, HEX0,
    VGA_R,VGA_G, VGA_B,
    VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK
);
    parameter nX = 10;
    parameter nY = 9;


    input wire CLOCK_50;
    input wire [9:0] SW;
    input wire [3:0] KEY;
    output wire [9:0] LEDR;
    inout wire PS2_CLK, PS2_DAT;
    output wire [6:0] HEX5, HEX4, HEX3, HEX2, HEX1, HEX0;
    output wire [7:0] VGA_R;
    output wire [7:0] VGA_G;
    output wire [7:0] VGA_B;
    output wire VGA_HS;
    output wire VGA_VS;
    output wire VGA_BLANK_N;
    output wire VGA_SYNC_N;
    output wire VGA_CLK;




    wire Resetn;
    assign Resetn = KEY[0];




    // KEY1: start game


    wire KEY1_sync;
    sync S1 (~KEY[1], Resetn, CLOCK_50, KEY1_sync);




// call game fsm
    wire in_menu;
    wire in_clear;
    wire in_play;
    wire clear_done;


    game_fsm GFSM(
        .Clock (CLOCK_50),
        .Resetn (Resetn),
        .start_game (KEY1_sync),
        .clear_done (clear_done),
        .in_menu (in_menu),
        .in_clear (in_clear),
        .in_play (in_play)
    );


// need for sc
 
    wire [nX-1:0] sc_x;
    wire [nY-1:0] sc_y;
    wire [8:0] sc_color;
    wire sc_write;
    wire [nX-1:0] sc_obj_x;
    wire [nY-1:0] sc_obj_y;
    wire [7:0]sc_scancode;


// score counter
    wire hit1;
    wire missile_hit = hit1;


    wire [7:0] score;


    score score_count(
        .Clock(CLOCK_50),
        .Resetn (Resetn),
        .missile_hit (missile_hit),
        .score (score)
    );


    // connect score with HEX
    hex7seg H2 (score[3:0], HEX2);


 // 1 = win, 0 = lose
    wire win, lose;
    reg endgame;
    reg result;      


    assign win = (score >= 8'd10);


    always @(posedge CLOCK_50) begin
        if (!Resetn) begin
            endgame <= 1'b0;
            result <= 1'b0;
        end
        else if (!endgame && in_play) begin
            if (win) begin
                endgame <= 1'b1;
                result <= 1'b1; // activate green
            end
            else if (lose) begin
                endgame <= 1'b1;
                result <= 1'b0;  // activate red
            end
        end
    end


// sc
    wire sc_go;
    assign sc_go = in_play & KEY1_sync & ~endgame;


    spacecraft SC1(
        .Clock (CLOCK_50),
        .Resetn Resetn),
        .go (sc_go),
        .PS2_CLK (PS2_CLK),
        .PS2_DAT (PS2_DAT),
        .VGA_x (sc_x),
        .VGA_y (sc_y),
        .VGA_color(sc_color),
        .VGA_write (sc_write),
        .obj_x(sc_obj_x),
        .obj_y (sc_obj_y),
        .scancode_out (sc_scancode)
    );


// press m for missile
    reg  m_pressed_d;
    wire m_pressed;
    wire fire_pulse;
    wire fire_enabled;




    assign m_pressed = (sc_scancode == 8'h3A);


    always @(posedge CLOCK_50) begin
        if (!Resetn)
            m_pressed_d <= 1'b0;
        else
            m_pressed_d <= m_pressed;
    end


    assign fire_pulse = m_pressed & ~m_pressed_d;
    assign fire_enabled = fire_pulse & in_play & ~endgame;




    // Alien wires


    wire [nX-1:0] al_x;
    wire [nY-1:0] al_y;
    wire [8:0] al_color;
    wire al_write;
    wire [nX-1:0] al_obj_x;
    wire [nY-1:0] al_obj_y;
    wire al_alive;
    wire al_resetn;
    wire al_go;






    alien AL1(
        .Clock (CLOCK_50),
        .Resetn (al_resetn),
        .go (al_go),
        .hit (hit1),
        .score (score),
        .alive (al_alive),
        .VGA_x (al_x),
        .VGA_y (al_y),
        .VGA_color (al_color),
        .VGA_write (al_write),
        .obj_x (al_obj_x),
        .obj_y (al_obj_y)
    );
     
     wire [9:0] al_x_safe;
     assign al_x_safe = (al_obj_x == 10'd0)?10'd1 :al_obj_x;
    
// alien auto spawn 


    spawn spawn1(
        .Clock (CLOCK_50),
        .Resetn  (Resetn),
        .in_play (in_play),
        .al_spawn_reset(al_resetn),
        .al_alive (al_alive)
    );




    // game_lose detection


    game_lose G1(
          .Clock (CLOCK_50),
          .Resetn(KEY[1]),
        .al_x (al_x_safe),
        .al_alive(al_alive),
        .lose (lose)
    );




    // Missile 


    wire [nX-1:0] missile_x;
    wire [nY-1:0] missile_y;
    wire [8:0] missile_color;
    wire missile_write;
    wire missile_active;
    wire [nX-1:0] missile_posX;
    wire [nY-1:0] missile_posY;


    missile M1(
        .Clock (CLOCK_50),
        .Resetn (Resetn),
        .fire (fire_enabled),
        .hit (missile_hit),
        .player_x (sc_obj_x),
        .player_y sc_obj_y),
        .VGA_x (missile_x),
        .VGA_y (missile_y),
        .VGA_color (missile_color),
        .VGA_write (missile_write),
        .active (missile_active),
        .posX (missile_posX),
        .posY (missile_posY)
    );




    // Collision module 


    collision C1(
        .missile_x (missile_posX),
        .missile_y (missile_posY),
        .alien_x (al_obj_x),
        .alien_y (al_obj_y),
        .alive (al_alive),
        .hit (hit1)
    );


    // Object MUX: missile > spacecraft > alien


    wire [nX-1:0] game_x;
    wire [nY-1:0] game_y;
    wire [8:0] game_color;
    wire game_write;


    assign game_x = missile_write ? missile_x :
                    sc_write ? sc_x :
                    al_write ? al_x :
                    {nX{1'b0}};


    assign game_y = missile_write ? missile_y :
                    sc_write     ? sc_y      :
                    al_write     ? al_y      :
                    {nY{1'b0}};


    assign game_color = missile_write ? missile_color :
                        sc_write     ? sc_color      :
                        al_write     ? al_color      :
                        9'b0;


    assign game_write = missile_write | sc_write | al_write;


  
    // Black bg


    wire [nX-1:0] clr_x;
    wire [nY-1:0] clr_y;
    wire [8:0]    clr_color;
    wire          clr_write;
    wire          clr_busy;


    clear_screen #(
        .nX(10),
        .nY(9),
        .COLS(640),
        .ROWS(480),
        .COLOR_DEPTH(9)
    ) CLEAR1 (
        .Clock      (CLOCK_50),
        .Resetn     (Resetn),
        .start      (in_clear),    
        .busy       (clr_busy),
        .done       (clear_done),
        .VGA_x      (clr_x),
        .VGA_y      (clr_y),
        .VGA_color  (clr_color),
        .VGA_write  (clr_write)
    );




    // RESULT: green (win) or red (lose)
 
    wire [nX-1:0] rs_x;
    wire [nY-1:0] rs_y;
    wire [8:0]    rs_color;
    wire          rs_write;


    result_screen RESULT(
        .Clock     (CLOCK_50),
        .Resetn    (Resetn),
        .show      (endgame), // start when game ends
        .result    (result),  // 1=win (green), 0=lose (red)
        .VGA_x     (rs_x),
        .VGA_y     (rs_y),
        .VGA_color (rs_color),
        .VGA_write (rs_write)
    );


 
    // FINAL MUX


    reg [nX-1:0] VGA_x_drv;
    reg [nY-1:0] VGA_y_drv;
    reg [8:0]    VGA_color_drv;
    reg          VGA_write_drv;


    always @(*) begin
        if (rs_write) begin
            // Game over screen on top of everything
            VGA_x_drv     = rs_x;
            VGA_y_drv     = rs_y;
            VGA_color_drv = rs_color;
            VGA_write_drv = rs_write;
        end
        else if (in_clear) begin
            // Clearing screen to black
            VGA_x_drv     = clr_x;
            VGA_y_drv     = clr_y;
            VGA_color_drv = clr_color;
            VGA_write_drv = clr_write;
        end
        else if (in_play) begin
            // Normal game drawing
            VGA_x_drv     = game_x;
            VGA_y_drv     = game_y;
            VGA_color_drv = game_color;
            VGA_write_drv = game_write;
        end
        else begin
            // MENU: just show initial bg.mif, no writes
            VGA_x_drv     = {nX{1'b0}};
            VGA_y_drv     = {nY{1'b0}};
            VGA_color_drv = 9'b0;
            VGA_write_drv = 1'b0;
        end
    end




    // Debug LEDs + HEX
 
    assign LEDR = {2'b00, sc_scancode};


    hex7seg H0 (sc_scancode[3:0], HEX0);
    hex7seg H1 (sc_scancode[7:4], HEX1);
    assign HEX3 = 7'b1111111;
    assign HEX4 = 7'b1111111;
    assign HEX5 = 7'b1111111;




    // VGA ADAPTER start menu is bg.mif


    vga_adapter VGA (
        .resetn      (Resetn),
        .clock       (CLOCK_50),
        .color       (VGA_color_drv),
        .x           (VGA_x_drv),
        .y           (VGA_y_drv),
        .write       (VGA_write_drv),
        .VGA_R       (VGA_R),
        .VGA_G       (VGA_G),
        .VGA_B       (VGA_B),
        .VGA_HS      (VGA_HS),
        .VGA_VS      (VGA_VS),
        .VGA_BLANK_N (VGA_BLANK_N),
        .VGA_SYNC_N  (VGA_SYNC_N),
        .VGA_CLK     (VGA_CLK)
    );
    defparam VGA.RESOLUTION       = "640x480";
    defparam VGA.COLOR_DEPTH      = 9;
    defparam VGA.BACKGROUND_IMAGE = "./MIF/bg.mif";  // start menu image


endmodule




//  SPACECRAFT 


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


   // ps2
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


    regn USS (Serial[8:1], Resetn, Esc, Clock, scancode);
    assign scancode_out = scancode;




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




    parameter S_A = 2'b00;
    parameter S_B = 2'b01;
    parameter S_C = 2'b10;
    parameter S_D = 2'b11;


    wire obj_done;


    // state transitions
    always @(*) begin
        case (y_Q)
            S_A: if (!ps2_rec) Y_D = S_A;
                 else          Y_D = S_B;
            S_B: Y_D = S_C;
            S_C: Y_D = S_D;
            S_D: if (!obj_done) Y_D = S_D;
                 else           Y_D = S_A;
            default: Y_D = S_A;
        endcase
    end




    always @(*) begin
        Esc  = 1'b0;
        step = 1'b0;
        case (y_Q)
            S_A: ;                 
            S_B: Esc  = 1'b1;     
            S_C: step = 1'b1;     
            S_D: ;                 
        endcase
    end


    always @(posedge Clock) begin
        if (!Resetn)
            y_Q <= S_A;
        else
            y_Q <= Y_D;
    end


// call object module 
    object O1(
        .Resetn    (Resetn),
        .Clock     (Clock),
        .go        (go),
        .ps2_rec   (step),
        .dir       (dir),
        .hit       (1'b0),
        .alive     (),          
        .init_y    (9'd240),
        .VGA_x     (VGA_x),
        .VGA_y     (VGA_y),
        .VGA_color (VGA_color),
        .VGA_write (VGA_write),
        .done      (obj_done),
        .obj_x     (obj_x),
        .obj_y     (obj_y)
    );


    // move faster 
    defparam O1.UX.STEP = 7;
    defparam O1.UY.STEP = 7;


endmodule




//  ALIEN 


module alien(
    Clock, Resetn, go, hit, score, alive,
    VGA_x, VGA_y, VGA_color, VGA_write, obj_x, obj_y
);
    parameter nX = 10;
    parameter nY = 9;


    input  wire Clock;
    input  wire Resetn;
    input  wire go;
    input  wire hit;
    input  wire [7:0] score;   
    output wire alive;
    output wire [nX-1:0] VGA_x;
    output wire [nY-1:0] VGA_y;
    output wire [8:0]    VGA_color;
    output wire          VGA_write;
    output wire [nX-1:0] obj_x;
    output wire [nY-1:0] obj_y;


    // speed adjustment by changing bit width
    reg  [19:0] slow;
    wire        step;


    always @(posedge Clock) begin
        if (!Resetn || go)
            slow <= 20'd0;
        else
            slow <= slow + 20'd1;
    end


    assign step = &slow;  


    wire [1:0] dir;
    assign dir = 2'b00;   


    wire dummy_done;
    wire [8:0] init_y;


    random_y R1(
        .Clock        (Clock),
        .Resetn       (Resetn),
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
    defparam O2.xOBJ      = 4;
    defparam O2.yOBJ      = 4;
    defparam O2.INIT_FILE = "./MIF/alien.mif";
endmodule


//  OBJECT (engine)


module object(
    Resetn, Clock, go, ps2_rec, dir, hit, alive, init_y,
    VGA_x, VGA_y, VGA_color, VGA_write, done,
    obj_x, obj_y
);
    parameter nX = 10;
    parameter nY = 9;
    parameter XOFFSET = 40;
    parameter YOFFSET = 240;
    parameter LEFT  = 2'b00;
    parameter RIGHT = 2'b11;
    parameter UP    = 2'b01;
    parameter DOWN  = 2'b10;
    parameter xOBJ = 4;
    parameter yOBJ = 4;
    parameter BOX_SIZE_X = 1 << xOBJ;
    parameter BOX_SIZE_Y = 1 << yOBJ;
    parameter Mn = xOBJ + yOBJ;
    parameter INIT_FILE = "./MIF/sc.mif";


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
    assign Y0 = init_y;
    assign size_x = BOX_SIZE_X;
    assign size_y = BOX_SIZE_Y;
	// from examples
    // Position counters
    upDn_count UX (X0, Clock, Resetn, Lx, Ex, Right, X);
        defparam UX.n = nX;


    upDn_count UY (Y0, Clock, Resetn, Ly, Ey, Down, Y);
        defparam UY.n = nY;




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




//  MISSILE


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


    parameter ScreenX           = 640;
    parameter missile_length    = 25;
    parameter missile_thickness = 3;
    parameter N                 = 16;


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


    parameter IDLE    = 4'b0000;
    parameter INITIAL = 4'b0001;
    parameter DRAW    = 4'b0010;
    parameter WAIT    = 4'b0011;
    parameter ERASE   = 4'b0100;
    parameter MOVE    = 4'b0101;
    parameter DONE    = 4'b0110;


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


module game_fsm(
    Clock, Resetn, start_game, clear_done,
    in_menu, in_clear, in_play
);
    input  wire Clock;
    input  wire Resetn;   
    input  wire start_game;  // from KEY1
    input  wire clear_done;  // from clear_screen module


    output reg  in_menu;
    output reg  in_clear;
    output reg  in_play;


    parameter S_MENU  = 2'b00;
    parameter S_CLEAR = 2'b01;
    parameter S_PLAY  = 2'b10;


    reg [1:0] y_Q;
    reg [1:0] Y_D;


    // Next-state
    always @(*) begin
        Y_D = y_Q;
        case (y_Q)
            S_MENU: begin
                if (start_game)
                    Y_D = S_CLEAR;
            end
            S_CLEAR: begin
                if (clear_done)
                    Y_D = S_PLAY;
            end
            S_PLAY: begin
                Y_D = S_PLAY; // stay in play
            end
            default: Y_D = S_MENU;
        endcase
    end


    // State
    always @(posedge Clock) begin
        if (!Resetn)
            y_Q <= S_MENU;
        else
            y_Q <= Y_D;
    end


    // Outputs to adapter
    always @(*) begin
        in_menu  = (y_Q == S_MENU);
        in_clear = (y_Q == S_CLEAR);
        in_play  = (y_Q == S_PLAY);
    end
endmodule


// erase it with color black (1’b0)
module clear_screen #(
    parameter nX = 10,
    parameter nY = 9,
    parameter COLS = 640,
    parameter ROWS = 480,
    parameter COLOR_DEPTH = 9
)(
    input  wire                Clock,
    input  wire                Resetn,
    input  wire                start,     // level: asserted in CLEAR state
    output reg                 busy,
    output reg                 done,
    output reg  [nX-1:0]       VGA_x,
    output reg  [nY-1:0]       VGA_y,
    output wire [COLOR_DEPTH-1:0] VGA_color,
    output reg                 VGA_write
);
    // clear color = black
    assign VGA_color = {COLOR_DEPTH{1'b0}};


    reg [nX-1:0] x;
    reg [nY-1:0] y;
    reg          active;


    always @(posedge Clock) begin
        if (!Resetn) begin
            x      <= {nX{1'b0}};
            y      <= {nY{1'b0}};
            active <= 1'b0;
            done   <= 1'b0;
        end
        else begin
            if (start && !active && !done) begin
                // start clearing
                active <= 1'b1;
                x      <= {nX{1'b0}};
                y      <= {nY{1'b0}};
                done   <= 1'b0;
            end
            else if (active) begin
                // advance pixel coordinates
                if (x == COLS-1) begin
                    x <= 0;
                    if (y == ROWS-1) begin
                        y      <= 0;
                        active <= 1'b0;
                        done   <= 1'b1;
                    end
                    else begin
                        y <= y + 1'b1;
                    end
                end
                else begin
                    x <= x + 1'b1;
                end
            end
        end
    end


    always @(*) begin
        if (active) begin
            VGA_x = x;
            VGA_y = y;
            VGA_write = 1'b1;
            busy = 1'b1;
        end
        else begin
            VGA_x     = {nX{1'b0}};
            VGA_y     = {nY{1'b0}};
            VGA_write = 1'b0;
            busy      = 1'b0;
        end
    end
endmodule


// Given Module re-use them 
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
    parameter STEP = 1;


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
                Q <= Q + STEP;
            else
                Q <= Q - STEP;
        end
    end
endmodule
// debug + score
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
    parameter missile_length = 25;
    parameter missile_thickness= 3;
    parameter alien_width = 32;
    parameter alien_height = 32;


    input wire [nX-1:0] missile_x;
    input wire [nY-1:0] missile_y;
    input wire [nX-1:0] alien_x;
    input wire [nY-1:0] alien_y;
    input wire alive;
    output wire hit;


    wire [nX-1:0] missile_left = missile_x;
    wire [nX-1:0] missile_right = missile_x + (missile_length - 1);
    wire [nY-1:0] missile_top = missile_y;
    wire [nY-1:0] missile_bottom = missile_y + (missile_thickness - 1);


    wire [nX-1:0] alien_left = alien_x - 6;
    wire [nX-1:0] alien_right = alien_x + (alien_width - 1) - 6;
    wire [nY-1:0] alien_top = alien_y - 8;
    wire [nY-1:0] alien_bottom = alien_y + (alien_height - 1) - 8;


    wire collision = (missile_left   <= alien_right ) &&
                     (missile_right  >= alien_left  ) &&
                     (missile_top <= alien_bottom) &&
                     (missile_bottom >= alien_top );


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
        if (rand < 9'd16)
            alien_init_y <= 9'd16;
        else if (rand > 9'd464)
            alien_init_y <= 9'd464;
        else
            alien_init_y <= rand;
    end
endmodule    
//module to generate 16-bit random seed, posted on Piazza by the instructor
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


//============================================================
//  game_lose detection (combinational)
//============================================================
module game_lose(
     Clock,
     Resetn,
    al_x,
    al_alive,
    lose
);  
     input wire Clock;
     input wire Resetn;
    input  wire [9:0] al_x;
    input  wire       al_alive;


    output reg       lose;


    // lose when alien is alive and reaches the left side
     always @ (posedge Clock) begin
        if (!Resetn) begin
            lose <= 1'b0;
        end else begin
            if ((al_alive) && (al_x>10'd2) && (al_x<10'd5)) begin
                lose <= 1'b1;
            end
        end
     end
endmodule


//============================================================
//  Alien spawn: generates reset pulses for alien
//============================================================
module spawn(
    Clock,
    Resetn,
    in_play,
    al_spawn_reset,
    al_alive
);


    input wire Clock;
    input wire Resetn;
    input wire in_play;
    input wire al_alive;
    output wire al_spawn_reset;


    reg[25:0]timer;
    reg waiting;
    reg prev_alive;
    reg respawn_pulse;


    always @(posedge Clock) begin
        if (!Resetn) begin
            prev_alive <= 1'b0;
            waiting <= 1'b0;
            timer <= 26'd0;
            respawn_pulse <= 1'b0;
        end
        else begin
            prev_alive <= al_alive;
            respawn_pulse <= 1'b0;


            if (!in_play) begin
                waiting <= 1'b0;
                timer <= 26'd0;
            end
            else begin
                if (!waiting) begin
                    // detect if alien just died (alive -> 0)
                    if (prev_alive && !al_alive) begin
                        waiting <= 1'b1;
                        timer <= 26'd0;
                    end
                end
                else begin
                    if (timer == 26'd50_000_000 -1) begin
                        waiting <= 1'b0;
                        respawn_pulse <= 1'b1; 
                    end
                    else begin
                        timer <= timer + 26'd1;
                    end
                end
            end
        end
    end


    // active-low reset for alien: momentarily low when respawn_pulse = 1
    assign al_spawn_reset = Resetn & ~respawn_pulse;


endmodule




//============================================================
//  RESULT SCREEN: full-screen green (win) / red (lose)
//============================================================
module result_screen(
    Clock, Resetn, show, result,
    VGA_x, VGA_y, VGA_color, VGA_write
);
    parameter nX = 10;
    parameter nY = 9;
    parameter COLS = 640;
    parameter ROWS = 480;
    parameter COLOR_DEPTH = 9;


    input  wire Clock;
    input  wire Resetn;
    input  wire show;    // 1 = start drawing result screen
    input  wire result;    // 1 = win (green), 0 = lose (red)


    output reg [nX-1:0]VGA_x;
    output reg [nY-1:0]VGA_y;
    output reg [COLOR_DEPTH-1:0] VGA_color;
    output reg VGA_write;


    parameter [COLOR_DEPTH-1:0] GREEN = 9'b000111000;
    parameter [COLOR_DEPTH-1:0] RED   = 9'b111000000;


    reg [nX-1:0] x;
    reg [nY-1:0] y;
    reg active;


    // Scan the whole screen once when show=1
    always @(posedge Clock) begin
        if (!Resetn) begin
            x <= {nX{1'b0}};
            y <= {nY{1'b0}};
            active <= 1'b0;
        end
        else begin
            if (show && !active) begin
                active <= 1'b1;
                x <= {nX{1'b0}};
                y <= {nY{1'b0}};
            end
            else if (active) begin
                if (x == COLS-1) begin
                    x <= 0;
                    if (y == ROWS-1) begin
                        y <= 0;
                        active <= 1'b0; // finished fill once
                    end
                    else begin
                        y <= y + 1'b1;
                    end
                end
                else begin
                    x <= x + 1'b1;
                end
            end
        end
    end


    always @(*) begin
        if (active) begin
            VGA_x = x;
            VGA_y = y;
            VGA_write = 1'b1;
            VGA_color = result ? GREEN : RED;
        end
        else begin
            VGA_x= {nX{1'b0}};
            VGA_y = {nY{1'b0}};
            VGA_write = 1'b0;
            VGA_color = {COLOR_DEPTH{1'b0}};
        end
    end
endmodule
