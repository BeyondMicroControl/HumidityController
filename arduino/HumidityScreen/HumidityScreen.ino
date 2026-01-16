/*
  64x32 + Tiny4kOLED double-buffered version of your animation.

  Key idea:
  - Render into a small 64x32 1bpp buffer (256 bytes)
  - Push that buffer into the *non-displayed* SSD1306 RAM half
  - Swap render/display with oled.switchFrame() for flicker-free updates

  switchFrame() swaps both render+display frames.  :contentReference[oaicite:2]{index=2}
*/

//#define DEBUG_FPS
//#define SIM_VALUES
//#define SHOW_DEBUG

#include <Arduino.h>
#include <Wire.h>
#include <Tiny4kOLED.h>


struct Particle 
{
  int16_t xq, yq;     // Q4.4
  int16_t vxq, vyq;   // Q4.4
  int16_t rq;         // Q4.4
};

// ---------------------- Screen config ----------------------
static constexpr uint8_t W = 64;
static constexpr uint8_t H = 32;


static inline uint8_t log2_pow2_u8(uint8_t p);  // instatiate helper function
static const uint8_t W2 = log2_pow2_u8(W);
static constexpr uint8_t PAGES = H >> 3;
static uint8_t fb[W * PAGES]; // 64 * 4 = 256 bytes

static uint16_t   debug_val = 0;
static uint16_t  btn_status = 0;
static uint16_t   menu_status = 0;

// ---------------------- Fixed-point helpers ----------------------
static inline int16_t q4_4(int16_t x)      { return (int16_t)(x << 4); }
static inline int16_t i4_4(int16_t x_q)    { return (int16_t)(x_q >> 4); }

// ---------------------- Minimal framebuffer drawing ----------------------
static inline void fbClear()
{
  memset(fb, 0x00, sizeof(fb));
}

static inline void fbSetPixel(int16_t x, int16_t y, bool on=true) 
{
  if ((uint16_t)x >= W || (uint16_t)y >= H) return;
  uint16_t idx = (uint16_t)(y >> 3 << W2) + (uint16_t)x;
  uint8_t  bit = (uint8_t)(1u << (y & 7));
  if (on) fb[idx] |= bit;
  else    fb[idx] &= (uint8_t)~bit;
}

// Filled circle via vertical spans (OK for small radii)
static void fbFillCircle(int16_t cx, int16_t cy, uint8_t r) 
{
  int16_t rr = (int16_t)r;
  int16_t r2 = rr * rr;

  for (int16_t dy = -rr; dy <= rr; dy++) {
    int16_t y = cy + dy;
    if ((uint16_t)y >= H) continue;

    int16_t rem = r2 - dy * dy;
    int16_t dx = 0;
    while ((dx + 1) * (dx + 1) <= rem) dx++; // small r => fine

    int16_t x0 = cx - dx;
    int16_t x1 = cx + dx;

    if (x1 < 0 || x0 >= W) continue;
    if (x0 < 0) x0 = 0;
    if (x1 >= W) x1 = W - 1;

    for (int16_t x = x0; x <= x1; x++) fbSetPixel(x, y, true);
  }
}

static inline void fbHLine(int16_t x0, int16_t x1, int16_t y) 
{
  if ((uint16_t)y >= H) return;
  if (x0 > x1) { int16_t t = x0; x0 = x1; x1 = t; }
  if (x1 < 0 || x0 >= W) return;
  if (x0 < 0) x0 = 0;
  if (x1 >= W) x1 = W - 1;
  for (int16_t x = x0; x <= x1; x++) fbSetPixel(x, y, true);
}

// Filled pyramid/triangle:
// (px,py) = bottom-center of the base
// h = height in pixels (>=1)
// w = base width in pixels (>=1)
static void fbFillPyramid(int16_t px, int16_t py, uint8_t h, uint8_t w) 
{
  if (h == 0 || w == 0) return;

  // If height is 1: just draw the base line (width w) at y=py
  if (h == 1) {
    int16_t left  = (int16_t)(w >> 1);
    int16_t right = (int16_t)(w - 1 - left);
    fbHLine(px - left, px + right, py);
    return;
  }

  // Draw from bottom row (row=0) to top row (row=h-1)
  // Bottom width = w, top width = 1
  const uint16_t denom = (uint16_t)(h - 1);
  const uint16_t wminus1 = (uint16_t)(w - 1);

  for (uint8_t row = 0; row < h; row++) {
    int16_t y = (int16_t)py - (int16_t)row;
    if ((uint16_t)y >= H) continue;

    // frac = (denom - row)/denom   => width goes w .. 1
    // widthRow = 1 + (w-1) * (denom-row) / denom   (rounded)
    uint16_t numer = (uint16_t)(denom - row);
    uint16_t widthRow = 1u + (uint16_t)((wminus1 * numer + (denom >> 1)) / denom);

    int16_t left  = (int16_t)(widthRow >> 1);
    int16_t right = (int16_t)(widthRow - 1u - (uint16_t)left);

    fbHLine(px - left, px + right, y);
  }
}


// Push framebuffer into the CURRENT *render* frame (Tiny4kOLED handles frame offset)
static void fbBlitToOLED() 
{
  for (uint8_t page = 0; page < PAGES; page++) {
    oled.setCursor(0, page);
    oled.startData();
    const uint8_t* row = &fb[(uint16_t)page << W2];
    for (uint8_t x = 0; x < W; x++) oled.sendData(row[x]);
    oled.endData();
  }
}

// ---------------------- Animation ----------------------
static constexpr uint8_t NUM_PARTICLES = 12;
static constexpr uint8_t INIT_SPREAD   = 10;

static Particle particles[NUM_PARTICLES];

static void resetParticle(Particle* p) 
{
  // Center for 64x32 is (32,16)
  p->xq = q4_4(32) + q4_4(randRange(-INIT_SPREAD, INIT_SPREAD));
  p->yq = q4_4(16) + q4_4(randRange(-INIT_SPREAD, INIT_SPREAD));

  // Slightly gentler motion
  p->vxq = (int16_t)((randRange(-4, 5) << 2)<< 1);
  p->vyq = (int16_t)((randRange(-4, 5) << 2) << 1);
  p->rq  = q4_4(4);
}

static inline void initParticles() 
{
  //randomSeed(analogRead(A4));
  for (uint8_t i = 0; i < NUM_PARTICLES; i++) resetParticle(&particles[i]);
}

// Tiny sine-ish LUT (0..63) mapped to about [-1..+1]
static const int8_t sinLUT64[] PROGMEM = 
{
   0, 0, 0, 1, 1, 1, 1, 1,
   1, 1, 1, 1, 1, 1, 0, 0,
   0, 0, 0,-1,-1,-1,-1,-1,
  -1,-1,-1,-1,-1,-1, 0, 0,
   0, 0, 0, 1, 1, 1, 1, 1,
   1, 1, 1, 1, 1, 1, 0, 0,
   0, 0, 0,-1,-1,-1,-1,-1,
  -1,-1,-1,-1,-1,-1, 0, 0,
};

static uint16_t frameCount = 0;
static uint8_t  charge = 0;
static char     chargeBuf[8];

#ifdef DEBUG_FPS
  static uint32_t lastMs = 0;
  static uint16_t fps = 0;
#endif

static void drawOverlayText(bool bInv) 
{
  // All of this is drawn into the same render frame (before switchFrame()).
  oled.setFont(FONT6X8);

#ifdef DEBUG_FPS
  debug_val = fps;
#endif

#ifdef SHOW_DEBUG
  // FPS top-left
  oled.setCursor(0, 0);
  oled.print(debug_val);
#endif

  // Charge centered-ish on line 2 (page 2 is y=16..23)
  oled.setCursor(23, 2);
  oled.invertOutput(bInv);
  oled.print(chargeBuf);
  oled.invertOutput(false);
}

static void renderFrameIntoCurrentRenderBuffer() 
{
  fbClear();

  // particles
  for (uint8_t i = 0; i < NUM_PARTICLES; i++)
  {
    Particle& p = particles[i];

    p.xq += p.vxq;
    p.yq += p.vyq;

    // r *= 0.92 approx => r = r * 59 / 64
    p.rq = (int16_t)((p.rq * 59) >> 6);

    int16_t x = i4_4(p.xq);
    int16_t y = i4_4(p.yq);
    int16_t r = i4_4(p.rq);

    if (r > 0) fbFillCircle(x, y, (uint8_t)r);

    if (x < -8 || x > (W + 8) || y < -8 || y > (H + 8) || p.rq < (q4_4(1) >> 1))
      resetParticle(&p);
  }

  // pulsing center disc: base 12 +/- 1
  uint8_t idx = (uint8_t)(frameCount & 63);
  int8_t  s   = (int8_t)pgm_read_byte(&sinLUT64[idx]);
  uint8_t radius = (uint8_t)(12 + s);
  fbFillCircle(32, 11+8, radius);
  fbFillPyramid(32, 11+8 - (radius>>1) , radius+2, (radius<<1)-2);       // pointy top blended in

}

static inline uint16_t readButtons(uint16_t bs, uint8_t timeout)
{
    // bitmap [8-bit counter] [00] [2xbtn2] [2xbtn1] [pre_btn2] [pre_btn1] [btn2] [btn1]
    uint16_t b = (digitalRead(PIN_PA2)==LOW?2:0) | (digitalRead(PIN_PA3)==LOW?1:0);
    uint16_t b2 = (bs==(timeout<<8) ? (timeout<<8) : (b==0? (bs&0xFF00)+0x0100 : 0 )) | ((b&bs&2)<<4) | ((bs&3)<<2) | b;

    debug_val = b2 & 3;

    return b2;
}

// ---------------------- Helpers ----------------------


static inline int16_t randRange(int16_t a, int16_t b) 
{
  return (int16_t)random(a, b); // [a..b-1]
}

static inline uint8_t log2_pow2_u8(uint8_t p)
{
  uint8_t x = 0;
  while (p >>= 1) x++;
  return x;
}

// ---------------------- Main ----------------------

void setup() 
{
  // 64x32 init sequence
  oled.begin(W, H, sizeof(tiny4koled_init_64x32r), tiny4koled_init_64x32r);
  oled.setFont(FONT6X8);

  // Clear BOTH frames once:
  oled.clear();
  oled.switchRenderFrame();
  oled.clear();
  oled.switchRenderFrame();

  oled.on();

  initParticles();
  strcpy(chargeBuf, "0%");


  #ifndef SIM_VALUES
    pinMode(PIN_PA2, INPUT_PULLUP);
    pinMode(PIN_PA3, INPUT_PULLUP);
  #endif


  #ifdef DEBUG_FPS
    lastMs = millis();
  #endif

  menu_status ^= 0b1; // trigger menu 1
}

uint8_t btn_timeout = 64;

void loop() {
  frameCount++;


switch(menu_status)
{
  case 1:
  {
    #ifndef SIM_VALUES
      // update charge every 2 frames
    if (menu_status&1==1 && (frameCount & 0b1) == 0b1) 
    {
      btn_status = readButtons(btn_status,btn_timeout);   // status, timeout

        if((btn_status & 2) == 2) { if(charge<100) charge++; else charge=100; };
        if((btn_status & 1) == 1) { if(charge>0) charge--; else charge=0; };

        //if(digitalRead(2)==LOW) { if(charge<100) charge++; else charge=100; };
        //if(digitalRead(3)==LOW) { if(charge>0) charge--; else charge=0; };
    }
    #endif

    if((btn_status>>8) < btn_timeout)
    {

      // print charge every 8 frames
      if ((frameCount & 0b111) == 0b110) 
      {
        #ifdef SIM_VALUES
        charge++;
        #endif
        snprintf(chargeBuf, sizeof(chargeBuf), "%u%%", charge);
      }

      // 1) draw into local fb
      renderFrameIntoCurrentRenderBuffer();
      // 2) push fb into the current RENDER frame in SSD1306 RAM
      fbBlitToOLED();
      // 3) draw text into the same render frame
      drawOverlayText(true);
      // 4) swap render/display for flicker-free animation
      //    switchFrame() == switchRenderFrame() + switchDisplayFrame() :contentReference[oaicite:3]{index=3}
      oled.switchFrame();
    }
    else
    {
      menu_status ^= 0b1; // trigger menu 1
      fbClear();
      fbBlitToOLED(); 
      drawOverlayText(false);
      // 4) swap render/display for flicker-free animation
      //    switchFrame() == switchRenderFrame() + switchDisplayFrame() :contentReference[oaicite:3]{index=3}
      oled.switchFrame();
    }
  }
}

#ifdef DEBUG_FPS
  // FPS calc
  uint32_t now = millis();
  uint32_t dt = now - lastMs;
  if (dt > 0) fps = (uint16_t)(1000u / dt);
  lastMs = now;
#endif
}
