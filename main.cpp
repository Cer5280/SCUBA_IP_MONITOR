// ============================================================
//  SCUBA INTERMEDIATE PRESSURE MONITOR
//  Target:  Waveshare ESP32-S3-Touch-LCD-7
//           800x480 RGB LCD, GT911 touch, CH422G IO expander
//           8MB OPI PSRAM, 8MB Flash
//
//  Sensor:  FUSCH 300 PSI, 1/8" NPT, 0.5-4.5V analog output
//           Wired through 10k/20k voltage divider to GPIO2
//
//  Features:
//    - 25 Hz live pressure readout, color-coded zones
//    - 800-sample scrolling waveform graph
//    - PSI/minute flow-rate indicator
//    - Touch graph tooltip
//    - Units toggle PSI / mbar
//    - ATMO CAL and 2-POINT CAL saved to NVS flash
//    - IP BASELINE + peak-drop tracking
//    - MIN / MAX statistics
//    - CSV serial output
// ============================================================

#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <Wire.h>
#include <Preferences.h>

// ============================================================
//  SECTION 1: CH422G IO EXPANDER
//  Controls backlight, LCD reset, touch reset via I2C
// ============================================================
#define CH422G_I2C_OE   0x24
#define CH422G_I2C_OUT  0x25
#define CH422G_I2C_SDA  8
#define CH422G_I2C_SCL  9

#define EXIO_TP_RST   (1 << 1)
#define EXIO_LCD_BL   (1 << 2)
#define EXIO_LCD_RST  (1 << 3)

static uint8_t ch422g_out = 0xFF;

void ch422g_write(uint8_t value) {
    Wire.beginTransmission(CH422G_I2C_OE);
    Wire.write(0x01);
    Wire.endTransmission();
    Wire.beginTransmission(CH422G_I2C_OUT);
    Wire.write(value);
    Wire.endTransmission();
    ch422g_out = value;
}

void ch422g_set(uint8_t pin_mask)   { ch422g_write(ch422g_out |  pin_mask); }
void ch422g_clear(uint8_t pin_mask) { ch422g_write(ch422g_out & ~pin_mask); }

// ============================================================
//  SECTION 2: DISPLAY CONFIGURATION
//  GPIO assignments for Waveshare ESP32-S3-Touch-LCD-7
// ============================================================
class LGFX : public lgfx::LGFX_Device {
    lgfx::Bus_RGB     _bus;
    lgfx::Panel_RGB   _panel;
    lgfx::Touch_GT911 _touch;
public:
    LGFX() {
        {
            auto cfg  = _bus.config();
            cfg.panel = &_panel;
            cfg.pin_d0  = GPIO_NUM_14;   // B0
            cfg.pin_d1  = GPIO_NUM_38;   // B1
            cfg.pin_d2  = GPIO_NUM_18;   // B2
            cfg.pin_d3  = GPIO_NUM_17;   // B3
            cfg.pin_d4  = GPIO_NUM_10;   // B4
            cfg.pin_d5  = GPIO_NUM_39;   // G0 changd 39 to 37 back to 39
            cfg.pin_d6  = GPIO_NUM_0;    // G1
            cfg.pin_d7  = GPIO_NUM_45;   // G2
            cfg.pin_d8  = GPIO_NUM_48;   // G3
            cfg.pin_d9  = GPIO_NUM_47;   // G4
            cfg.pin_d10 = GPIO_NUM_21;   // G5
            cfg.pin_d11 = GPIO_NUM_1;    // R0
            cfg.pin_d12 = GPIO_NUM_2;    // R1
            cfg.pin_d13 = GPIO_NUM_42;   // R2
            cfg.pin_d14 = GPIO_NUM_41;   // R3
            cfg.pin_d15 = GPIO_NUM_40;   // R4
            cfg.pin_henable = GPIO_NUM_5;
            cfg.pin_vsync   = GPIO_NUM_3;
            cfg.pin_hsync   = GPIO_NUM_46;
            cfg.pin_pclk    = GPIO_NUM_7;
            cfg.freq_write        = 16000000;
            cfg.hsync_polarity    = 0;
            cfg.hsync_front_porch = 40;
            cfg.hsync_pulse_width = 48;
            cfg.hsync_back_porch  = 88;
            cfg.vsync_polarity    = 0;
            cfg.vsync_front_porch = 13;
            cfg.vsync_pulse_width = 3;
            cfg.vsync_back_porch  = 32;
            cfg.pclk_active_neg   = 1;   //was 1 back to 1
            cfg.de_idle_high      = 0;    //was 0 back to 0
            cfg.pclk_idle_high    = 0;
            _bus.config(cfg);
        }
        {
            auto cfg          = _panel.config();
            cfg.memory_width  = 800;
            cfg.memory_height = 480;
            cfg.panel_width   = 800;
            cfg.panel_height  = 480;
            cfg.offset_x      = 0;
            cfg.offset_y      = 0;
            _panel.config(cfg);
            auto det       = _panel.config_detail();
            det.use_psram  = 1;                              //was 1
            _panel.config_detail(det);
        }
        {
            auto cfg            = _touch.config();
            cfg.x_min           = 0;
            cfg.x_max           = 799;
            cfg.y_min           = 0;
            cfg.y_max           = 479;
            cfg.pin_int         = -1;
            cfg.pin_rst         = -1;
            cfg.bus_shared      = false;
            cfg.offset_rotation = 0;
            cfg.i2c_port        = 0;
            cfg.pin_sda         = GPIO_NUM_8;
            cfg.pin_scl         = GPIO_NUM_9;
            cfg.freq            = 400000;
            cfg.i2c_addr        = 0x5D;
            _touch.config(cfg);
        }
        _panel.setBus(&_bus);
        _panel.setTouch(&_touch);
        setPanel(&_panel);
    }
};
static LGFX lcd;

// ============================================================
//  SECTION 3: SENSOR & CALIBRATION
// ============================================================
#define SENSOR_PIN  2
#define PSI_RANGE   300.0f

Preferences prefs;

float calLowV    = 0.5f;
float calHighV   = 4.5f;
float calLowPSI  = 0.0f;
float calHighPSI = 300.0f;

float atmoOffsetPSI = 0.0f;
bool  atmoCalSet    = false;
int   calMode       = 0;   // 0=uncal 1=atmo 2=two-point

void loadCal() {
    if (!prefs.begin("pmon", true)) return;
    calLowV       = prefs.getFloat("calLowV",    0.5f);
    calHighV      = prefs.getFloat("calHighV",   4.5f);
    calLowPSI     = prefs.getFloat("calLowPSI",  0.0f);
    calHighPSI    = prefs.getFloat("calHighPSI", 300.0f);
    atmoOffsetPSI = prefs.getFloat("atmoOff",    0.0f);
    atmoCalSet    = prefs.getBool("atmoSet",     false);
    calMode       = prefs.getInt("calMode",      0);
    prefs.end();
}

void saveCal() {
    if (!prefs.begin("pmon", false)) return;
    prefs.putFloat("calLowV",    calLowV);
    prefs.putFloat("calHighV",   calHighV);
    prefs.putFloat("calLowPSI",  calLowPSI);
    prefs.putFloat("calHighPSI", calHighPSI);
    prefs.putFloat("atmoOff",    atmoOffsetPSI);
    prefs.putBool("atmoSet",     atmoCalSet);
    prefs.putInt("calMode",      calMode);
    prefs.end();
}

// ============================================================
//  SECTION 4: UNITS
// ============================================================
enum Units { UNIT_PSI, UNIT_MBAR };
Units activeUnit = UNIT_PSI;

float psiToDisplay(float psi) {
    return (activeUnit == UNIT_PSI) ? psi : psi * 68.9476f;
}
const char* unitLabel() {
    return (activeUnit == UNIT_PSI) ? "PSI" : "mbar";
}
const char* calModeLabel() {
    if (calMode == 2) return "2PT-CAL";
    if (calMode == 1) return "ATMO-CAL";
    return "UNCAL";
}

// ============================================================
//  SECTION 5: COLOUR PALETTE
// ============================================================
#define C_BG         lcd.color888( 10,  12,  20)
#define C_PANEL      lcd.color888( 18,  22,  38)
#define C_BORDER     lcd.color888( 40,  55,  90)
#define C_GRID       lcd.color888( 22,  30,  50)
#define C_GRIDMAJ    lcd.color888( 35,  48,  80)
#define C_DIM        lcd.color888( 80,  90, 120)
#define C_WHITE      lcd.color888(255, 255, 255)
#define C_GREEN      lcd.color888(  0, 230,  90)
#define C_YELLOW     lcd.color888(250, 220,   0)
#define C_ORANGE     lcd.color888(255, 130,   0)
#define C_RED        lcd.color888(230,  30,  30)
#define C_CYAN       lcd.color888(  0, 210, 255)
#define C_WAVEFORM   lcd.color888(  0, 200, 140)
#define C_TOOLTIP    lcd.color888( 10,  40,  70)
#define C_BTN_BLUE   lcd.color888( 20,  80, 180)
#define C_BTN_AMBER  lcd.color888(200, 140,   0)
#define C_BTN_RED    lcd.color888(160,  20,  20)
#define C_BTN_GREEN  lcd.color888( 30, 100,  50)
#define C_BTN_PURPLE lcd.color888(100,  30, 160)
#define C_BASELINE   lcd.color888(180, 100, 255)
#define C_FLOWUP     lcd.color888( 50, 255, 120)
#define C_FLOWDOWN   lcd.color888(255,  80,  80)
#define C_FLOWSTABLE lcd.color888(160, 160, 160)

// ============================================================
//  SECTION 6: LAYOUT  800 x 480
// ============================================================
#define HDR_Y    0
#define HDR_H   50
#define MET_Y   50
#define MET_H  110
#define GFX_Y  160
#define GFX_W  800
#define GFX_H  255
#define GFX_BOT (GFX_Y + GFX_H)
#define FLW_Y  415
#define FLW_H   25
#define FTR_Y  440
#define FTR_H   40
#define BTN_H   30
#define BTN_Y  (FTR_Y + 5)

#define BTN_PAUSE_X    4
#define BTN_PAUSE_W  128
#define BTN_RESET_X  136
#define BTN_RESET_W  128
#define BTN_BASE_X   268
#define BTN_BASE_W   128
#define BTN_ATMO_X   400
#define BTN_ATMO_W   128
#define BTN_CAL_X    532
#define BTN_CAL_W    128
#define BTN_UNIT_X   664
#define BTN_UNIT_W   132

// ============================================================
//  SECTION 7: RING BUFFER
// ============================================================
#define BUF_SIZE 800
float gBuf[BUF_SIZE];
int   gHead = 0;
int   gFill = 0;

// ============================================================
//  SECTION 8: RUNTIME STATE
// ============================================================
bool  paused      = false;
float curPSI      = 0.0f;
float minPSI      =  1e9f;
float maxPSI      = -1e9f;
float peakDrop    = 0.0f;
float baselinePSI = 0.0f;
bool  baselineSet = false;

float flowRatePSImin = 0.0f;
#define FLOW_WINDOW_MS 1000
float  flowPrev      = 0.0f;
unsigned long flowPrevTime = 0;

bool  showTooltip = false;
int   ttipX       = 0;
float ttipPSI     = 0.0f;
unsigned long ttipTime = 0;

unsigned long lastSample = 0;
unsigned long lastDraw   = 0;
unsigned long lastStats  = 0;
unsigned long lastTouch  = 0;
#define SAMPLE_MS        40
#define DRAW_MS          40
#define STATS_MS        100
#define TOUCH_DEBOUNCE  300

bool  inCalScreen = false;
int   calStep     = 0;
float calCapLowV  = 0.0f;
float calCapHighV = 0.0f;

// ============================================================
//  SECTION 9: SENSOR FUNCTIONS
// ============================================================
float readRawVoltage() {
    long sum = 0;
    for (int i = 0; i < 16; i++) sum += analogRead(SENSOR_PIN);
    float v_gpio   = (sum / 16.0f / 4095.0f) * 3.3f;
    float v_sensor = v_gpio * 1.5f;   // undo 10k/20k divider
    return v_sensor;
}

float voltageToPSI(float v) {
    if (calHighV <= calLowV) return 0.0f;
    float slope = (calHighPSI - calLowPSI) / (calHighV - calLowV);
    float psi   = calLowPSI + (v - calLowV) * slope;
    return constrain(psi, -50.0f, PSI_RANGE + 10.0f);
}

float readPSI() {
    float psi = voltageToPSI(readRawVoltage()) - atmoOffsetPSI;
    return constrain(psi, -50.0f, PSI_RANGE);
}

// ============================================================
//  SECTION 10: HELPER UTILITIES
// ============================================================
int psiToGfxY(float psi) {
    int y = GFX_BOT - (int)((psi / PSI_RANGE) * GFX_H);
    return constrain(y, GFX_Y, GFX_BOT);
}

float gfxXtoPSI(int x) {
    if (gFill == 0) return 0.0f;
    int back = GFX_W - 1 - x;
    if (back >= gFill) return 0.0f;
    int idx = ((gHead - 1 - back) % BUF_SIZE + BUF_SIZE) % BUF_SIZE;
    return gBuf[idx];
}

uint32_t psiColor(float psi) {
    if (psi < 100.0f) return C_GREEN;
    if (psi < 180.0f) return C_YELLOW;
    if (psi < 250.0f) return C_ORANGE;
    return C_RED;
}

// Avoid conflict with Arduino HIGH macro - use ZONE_ prefix
const char* psiZoneLabel(float psi) {
    if (psi < 100.0f) return "NORMAL";
    if (psi < 180.0f) return "ELEVATED";
    if (psi < 250.0f) return "HIGH ZONE";
    return "CRITICAL";
}

void drawBtn(int x, int w, uint32_t col, const char* label) {
    lcd.fillRoundRect(x, BTN_Y, w, BTN_H, 5, col);
    lcd.drawRoundRect(x, BTN_Y, w, BTN_H, 5, C_WHITE);
    lcd.setTextColor(C_WHITE, col);
    lcd.setTextSize(1);
    int lw = strlen(label) * 6;
    lcd.setCursor(x + (w - lw) / 2, BTN_Y + 11);
    lcd.print(label);
}

// ============================================================
//  SECTION 11: HEADER BAR
// ============================================================
void drawHeader() {
    lcd.fillRect(0, HDR_Y, 800, HDR_H, C_PANEL);
    lcd.drawFastHLine(0, HDR_H - 1, 800, C_BORDER);
    lcd.setTextColor(C_CYAN, C_PANEL);
    lcd.setTextSize(2);
    lcd.setCursor(10, 8);
    lcd.print("SCUBA PRESSURE MONITOR");
    lcd.setTextColor(C_DIM, C_PANEL);
    lcd.setTextSize(1);
    lcd.setCursor(10, 32);
    lcd.print("INTERMEDIATE PRESSURE ANALYZER  25 Hz");

    uint32_t cc = (calMode == 2) ? lcd.color888(0,160,80)
                : (calMode == 1) ? lcd.color888(180,120,0)
                :                  C_BTN_RED;
    lcd.fillRoundRect(430, 8, 75, 16, 3, cc);
    lcd.setTextColor(C_PANEL, cc);
    lcd.setTextSize(1);
    lcd.setCursor(436, 11);
    lcd.print(calModeLabel());

    lcd.fillRoundRect(515, 8, 50, 16, 3, C_BTN_BLUE);
    lcd.setTextColor(C_WHITE, C_BTN_BLUE);
    lcd.setCursor(521, 11);
    lcd.print(unitLabel());

    lcd.fillCircle(775, 25, 7, C_GREEN);
}

// ============================================================
//  SECTION 12: METRICS ROW
// ============================================================
void drawMetricsRow(bool fullRedraw) {
    if (fullRedraw) {
        lcd.fillRect(0, MET_Y, 800, MET_H, C_BG);
        lcd.drawFastHLine(0, MET_Y + MET_H - 1, 800, C_BORDER);
        lcd.drawFastVLine(448, MET_Y + 5, MET_H - 10, C_BORDER);
        lcd.setTextColor(C_DIM, C_BG);
        lcd.setTextSize(1);
        lcd.setCursor(464,      MET_Y + 4);  lcd.print("STAT-MIN");
        lcd.setCursor(464 + 90, MET_Y + 4);  lcd.print("STAT-MAX");
        lcd.setCursor(464,      MET_Y + 60); lcd.print("PEAK DROP");
        lcd.setCursor(464 + 90, MET_Y + 60); lcd.print("IP BASE");
    }

    lcd.fillRect(0, MET_Y, 446, MET_H, C_BG);
    uint32_t col = psiColor(curPSI);
    lcd.setTextColor(col, C_BG);
    lcd.setTextSize(7);
    lcd.setCursor(8, MET_Y + 6);
    char buf[16];
    float disp = psiToDisplay(curPSI);
    if (activeUnit == UNIT_PSI) dtostrf(disp, 6, 1, buf);
    else                        dtostrf(disp, 7, 0, buf);
    lcd.print(buf);

    lcd.setTextColor(C_DIM, C_BG);
    lcd.setTextSize(2);
    lcd.setCursor(350, MET_Y + 16);
    lcd.print(unitLabel());

    lcd.fillRoundRect(8, MET_Y + 84, 130, 18, 4, col);
    lcd.setTextColor(C_PANEL, col);
    lcd.setTextSize(1);
    lcd.setCursor(14, MET_Y + 88);
    lcd.print(psiZoneLabel(curPSI));

    int sx = 464;
    lcd.setTextSize(2);

    lcd.fillRect(sx, MET_Y + 18, 82, 26, C_BG);
    lcd.setTextColor(C_CYAN, C_BG);
    lcd.setCursor(sx, MET_Y + 20);
    if (minPSI < 1e8f) { dtostrf(psiToDisplay(minPSI), 5, activeUnit==UNIT_PSI?1:0, buf); lcd.print(buf); }
    else lcd.print("---");

    lcd.fillRect(sx + 90, MET_Y + 18, 82, 26, C_BG);
    lcd.setTextColor(C_RED, C_BG);
    lcd.setCursor(sx + 90, MET_Y + 20);
    if (maxPSI > -1e8f) { dtostrf(psiToDisplay(maxPSI), 5, activeUnit==UNIT_PSI?1:0, buf); lcd.print(buf); }
    else lcd.print("---");

    lcd.fillRect(sx, MET_Y + 74, 82, 26, C_BG);
    lcd.setTextColor(C_ORANGE, C_BG);
    lcd.setCursor(sx, MET_Y + 76);
    dtostrf(psiToDisplay(peakDrop), 5, activeUnit==UNIT_PSI?1:0, buf);
    lcd.print(buf);

    lcd.fillRect(sx + 90, MET_Y + 74, 82, 26, C_BG);
    if (baselineSet) {
        lcd.setTextColor(C_BASELINE, C_BG);
        lcd.setCursor(sx + 90, MET_Y + 76);
        dtostrf(psiToDisplay(baselinePSI), 5, activeUnit==UNIT_PSI?1:0, buf);
        lcd.print(buf);
    } else {
        lcd.setTextColor(C_DIM, C_BG);
        lcd.setTextSize(1);
        lcd.setCursor(sx + 94, MET_Y + 82);
        lcd.print("TAP SET BASE");
    }
}

// ============================================================
//  SECTION 13: FLOW RATE BAR
// ============================================================
void drawFlowBar() {
    lcd.fillRect(0, FLW_Y, 800, FLW_H, C_PANEL);
    lcd.drawFastHLine(0, FLW_Y, 800, C_BORDER);

    uint32_t fc = (flowRatePSImin >  2.0f) ? C_FLOWUP
                : (flowRatePSImin < -2.0f) ? C_FLOWDOWN
                :                             C_FLOWSTABLE;

    lcd.setTextColor(fc, C_PANEL);
    lcd.setTextSize(1);
    lcd.setCursor(8, FLW_Y + 8);
    lcd.print("FLOW:");

    char buf[40];
    float fr = (activeUnit == UNIT_PSI) ? flowRatePSImin : flowRatePSImin * 68.9476f;
    const char* arrow = (fr > 2.0f) ? " UP " : (fr < -2.0f) ? " DN " : " -- ";
    snprintf(buf, sizeof(buf), "%s%+.1f %s/min", arrow, fr, unitLabel());
    lcd.setTextColor(fc, C_PANEL);
    lcd.setCursor(55, FLW_Y + 8);
    lcd.print(buf);

    int barMid = 500;
    int barMax = 140;
    int barLen = constrain((int)(flowRatePSImin / 50.0f * barMax), -barMax, barMax);
    int barX   = (barLen >= 0) ? barMid : barMid + barLen;
    int barW   = abs(barLen);
    if (barW > 0) lcd.fillRect(barX, FLW_Y + 5, barW, FLW_H - 10, fc);
    lcd.drawFastVLine(barMid, FLW_Y + 2, FLW_H - 4, C_DIM);
}

// ============================================================
//  SECTION 14: GRAPH BACKGROUND
// ============================================================
void drawGraphBackground() {
    lcd.fillRect(0, GFX_Y, GFX_W, GFX_H, C_BG);
    lcd.drawRect(0, GFX_Y, GFX_W, GFX_H, C_BORDER);

    for (int p = 50; p < (int)PSI_RANGE; p += 50) {
        int y = psiToGfxY((float)p);
        uint32_t gc = (p % 100 == 0) ? C_GRIDMAJ : C_GRID;
        lcd.drawFastHLine(1, y, GFX_W - 2, gc);
        lcd.setTextColor(C_DIM, C_BG);
        lcd.setTextSize(1);
        char lbl[10];
        if (activeUnit == UNIT_PSI) snprintf(lbl, sizeof(lbl), "%d", p);
        else snprintf(lbl, sizeof(lbl), "%d", (int)(p * 68.9476f));
        lcd.setCursor(GFX_W - 34, y - 8);
        lcd.print(lbl);
    }

    if (baselineSet) {
        int by = psiToGfxY(baselinePSI);
        for (int x = 0; x < GFX_W; x += 8) lcd.drawFastHLine(x, by, 4, C_BASELINE);
        lcd.setTextColor(C_BASELINE, C_BG);
        lcd.setTextSize(1);
        lcd.setCursor(2, by - 10);
        lcd.print("BASE");
    }

    if (atmoCalSet) {
        int ay = psiToGfxY(0.0f);
        lcd.drawFastHLine(1, ay, GFX_W - 2, lcd.color888(50, 50, 100));
        lcd.setTextColor(lcd.color888(100, 100, 180), C_BG);
        lcd.setTextSize(1);
        lcd.setCursor(2, ay + 2);
        lcd.print("ATMO 0");
    }
}

void drawGraphFull() {
    drawGraphBackground();
    if (gFill < 2) return;
    int count    = min(gFill, GFX_W);
    int startX   = GFX_W - count;
    int startIdx = ((gHead - count) % BUF_SIZE + BUF_SIZE) % BUF_SIZE;
    int px = startX, py = psiToGfxY(gBuf[startIdx]);
    for (int i = 1; i < count; i++) {
        int idx = (startIdx + i) % BUF_SIZE;
        int x = startX + i, y = psiToGfxY(gBuf[idx]);
        lcd.drawLine(px, py, x, y, C_WAVEFORM);
        px = x; py = y;
    }
}

// ============================================================
//  SECTION 15: FOOTER BUTTONS
// ============================================================
void drawFooter() {
    lcd.fillRect(0, FTR_Y, 800, FTR_H, C_PANEL);
    lcd.drawFastHLine(0, FTR_Y, 800, C_BORDER);
    drawBtn(BTN_PAUSE_X, BTN_PAUSE_W, paused ? C_BTN_AMBER : C_BTN_BLUE, paused ? "RESUME" : "PAUSE");
    drawBtn(BTN_RESET_X, BTN_RESET_W, C_BTN_RED, "RESET");
    drawBtn(BTN_BASE_X, BTN_BASE_W,
            baselineSet ? lcd.color888(0,100,50) : C_BTN_GREEN,
            baselineSet ? "CLR BASE" : "SET BASE");
    drawBtn(BTN_ATMO_X, BTN_ATMO_W,
            atmoCalSet ? lcd.color888(140,80,0) : lcd.color888(70,40,0),
            atmoCalSet ? "CLR ATMO" : "ATMO CAL");
    drawBtn(BTN_CAL_X, BTN_CAL_W, C_BTN_PURPLE, "2PT CAL");
    drawBtn(BTN_UNIT_X, BTN_UNIT_W,
            activeUnit == UNIT_PSI ? C_BTN_BLUE : lcd.color888(0,80,140),
            activeUnit == UNIT_PSI ? "UNITS:PSI" : "UNITS:mbar");
}

// ============================================================
//  SECTION 16: GRAPH TOOLTIP
// ============================================================
void drawTooltip() {
    lcd.drawFastVLine(ttipX, GFX_Y, GFX_H, C_WHITE);
    int bx = (ttipX + 10 < GFX_W - 115) ? ttipX + 10 : ttipX - 116;
    int by = GFX_Y + 14;
    lcd.fillRoundRect(bx, by, 112, 36, 5, C_TOOLTIP);
    lcd.drawRoundRect(bx, by, 112, 36, 5, C_CYAN);
    lcd.setTextColor(C_CYAN, C_TOOLTIP);
    lcd.setTextSize(2);
    lcd.setCursor(bx + 6, by + 9);
    char buf[14];
    dtostrf(psiToDisplay(ttipPSI), activeUnit==UNIT_PSI ? 5 : 6,
            activeUnit==UNIT_PSI ? 1 : 0, buf);
    lcd.print(buf);
    lcd.setTextSize(1);
    lcd.print(unitLabel());
}

// ============================================================
//  SECTION 17: 2-POINT CALIBRATION SCREEN
// ============================================================
void drawCalScreen() {
    lcd.fillScreen(TFT_RED);    //was C_BG
    lcd.setTextColor(C_CYAN, C_BG);
    lcd.setTextSize(3);
    lcd.setCursor(20, 15);
    lcd.print("2-POINT CALIBRATION");

    float v = readRawVoltage();
    char buf[48];
    lcd.setTextColor(C_WHITE, C_BG);
    lcd.setTextSize(2);
    lcd.setCursor(20, 60);
    snprintf(buf, sizeof(buf), "Live voltage: %.4f V", v);
    lcd.print(buf);

    lcd.setTextColor(C_DIM, C_BG);
    lcd.setTextSize(1);
    lcd.setCursor(20, 92);
    lcd.print("Step 1: Apply LOW pressure. Tap +/- for reference PSI. Tap CAPTURE LOW.");
    lcd.setCursor(20, 104);
    lcd.print("Step 2: Apply HIGH pressure. Tap +/- for reference PSI. Tap CAPTURE HIGH. Then SAVE.");

    uint32_t s0c = (calStep == 0) ? C_CYAN : C_DIM;
    lcd.drawRoundRect(14, 128, 372, 130, 7, s0c);
    lcd.setTextColor(s0c, C_BG);
    lcd.setTextSize(2);
    lcd.setCursor(24, 140);
    lcd.print("STEP 1 - LOW POINT");
    lcd.setTextSize(1);
    lcd.setCursor(24, 166);
    snprintf(buf, sizeof(buf), "Captured: %.4f V", calCapLowV);
    lcd.print(buf);
    lcd.setCursor(24, 180);
    snprintf(buf, sizeof(buf), "Ref: %.1f PSI / %.0f mbar", calLowPSI, calLowPSI * 68.9476f);
    lcd.print(buf);

    lcd.fillRoundRect(24,  200, 50, 28, 4, C_BTN_BLUE);
    lcd.fillRoundRect(80,  200, 50, 28, 4, C_BTN_BLUE);
    lcd.setTextColor(C_WHITE, C_BTN_BLUE);
    lcd.setTextSize(2);
    lcd.setCursor(37,  206); lcd.print("-");
    lcd.setCursor(94,  206); lcd.print("+");

    lcd.fillRoundRect(24, 232, 130, 22, 4, calStep==0 ? C_BTN_GREEN : lcd.color888(20,50,20));
    lcd.setTextColor(C_WHITE, calStep==0 ? C_BTN_GREEN : lcd.color888(20,50,20));
    lcd.setTextSize(1);
    lcd.setCursor(36, 240);
    lcd.print("CAPTURE LOW");

    uint32_t s1c = (calStep == 1) ? C_CYAN : C_DIM;
    lcd.drawRoundRect(400, 128, 384, 130, 7, s1c);
    lcd.setTextColor(s1c, C_BG);
    lcd.setTextSize(2);
    lcd.setCursor(410, 140);
    lcd.print("STEP 2 - HIGH POINT");
    lcd.setTextSize(1);
    lcd.setCursor(410, 166);
    snprintf(buf, sizeof(buf), "Captured: %.4f V", calCapHighV);
    lcd.print(buf);
    lcd.setCursor(410, 180);
    snprintf(buf, sizeof(buf), "Ref: %.1f PSI / %.0f mbar", calHighPSI, calHighPSI * 68.9476f);
    lcd.print(buf);

    lcd.fillRoundRect(410, 200, 50, 28, 4, C_BTN_BLUE);
    lcd.fillRoundRect(466, 200, 50, 28, 4, C_BTN_BLUE);
    lcd.setTextColor(C_WHITE, C_BTN_BLUE);
    lcd.setTextSize(2);
    lcd.setCursor(423, 206); lcd.print("-");
    lcd.setCursor(480, 206); lcd.print("+");

    lcd.fillRoundRect(410, 232, 140, 22, 4, calStep==1 ? C_BTN_GREEN : lcd.color888(20,50,20));
    lcd.setTextColor(C_WHITE, calStep==1 ? C_BTN_GREEN : lcd.color888(20,50,20));
    lcd.setTextSize(1);
    lcd.setCursor(422, 240);
    lcd.print("CAPTURE HIGH");

    lcd.fillRoundRect(14,  290, 185, 48, 7, C_BTN_GREEN);
    lcd.setTextColor(C_WHITE, C_BTN_GREEN);
    lcd.setTextSize(2);
    lcd.setCursor(44, 308);
    lcd.print("SAVE CAL");

    lcd.fillRoundRect(214, 290, 270, 48, 7, C_BTN_RED);
    lcd.setTextColor(C_WHITE, C_BTN_RED);
    lcd.setTextSize(1);
    lcd.setCursor(234, 311);
    lcd.print("RESET TO FACTORY DEFAULTS");

    lcd.fillRoundRect(500, 290, 185, 48, 7, lcd.color888(55,55,55));
    lcd.setTextColor(C_WHITE, lcd.color888(55,55,55));
    lcd.setTextSize(2);
    lcd.setCursor(534, 308);
    lcd.print("CANCEL");

    lcd.setTextColor(C_DIM, C_BG);
    lcd.setTextSize(1);
    lcd.setCursor(14, 358);
    snprintf(buf, sizeof(buf), "Active: %s  Lo %.4fV=%.1fPSI  Hi %.4fV=%.1fPSI  AtmoOff %.2fPSI",
        calModeLabel(), calLowV, calLowPSI, calHighV, calHighPSI, atmoOffsetPSI);
    lcd.print(buf);
}

// ============================================================
//  SECTION 18: FULL MAIN UI REDRAW
// ============================================================
void drawMainUI() {
    lcd.fillScreen(TFT_RED);
    drawHeader();
    drawMetricsRow(true);
    drawGraphFull();
    drawFlowBar();
    drawFooter();
}

// ============================================================
//  SECTION 19: TOUCH HANDLING - MAIN SCREEN
// ============================================================
void handleTouchMain() {
    lgfx::touch_point_t tp[1];
    int n = lcd.getTouch(tp, 1);
    if (n == 0) {
        if (showTooltip && millis() - ttipTime > 3000) {
            showTooltip = false;
            drawGraphFull();
        }
        return;
    }
    int tx = tp[0].x, ty = tp[0].y;
    if (millis() - lastTouch < TOUCH_DEBOUNCE) return;
    lastTouch = millis();

    auto inBtn = [&](int bx, int bw) {
        return tx >= bx && tx <= bx + bw && ty >= BTN_Y && ty <= BTN_Y + BTN_H;
    };

    if (inBtn(BTN_PAUSE_X, BTN_PAUSE_W)) {
        paused = !paused;
        drawFooter();
        return;
    }
    if (inBtn(BTN_RESET_X, BTN_RESET_W)) {
        minPSI = 1e9f; maxPSI = -1e9f; peakDrop = 0;
        baselineSet = false; gHead = 0; gFill = 0;
        flowRatePSImin = 0.0f;
        memset(gBuf, 0, sizeof(gBuf));
        drawMainUI();
        return;
    }
    if (inBtn(BTN_BASE_X, BTN_BASE_W)) {
        if (baselineSet) { baselineSet = false; peakDrop = 0; }
        else             { baselinePSI = curPSI; baselineSet = true; peakDrop = 0; }
        drawGraphFull(); drawMetricsRow(false); drawFooter();
        return;
    }
    if (inBtn(BTN_ATMO_X, BTN_ATMO_W)) {
        if (atmoCalSet) {
            atmoCalSet = false; atmoOffsetPSI = 0.0f;
            if (calMode == 1) calMode = 0;
        } else {
            float rawPSI  = voltageToPSI(readRawVoltage());
            atmoOffsetPSI = rawPSI - 14.7f;
            atmoCalSet    = true;
            if (calMode < 2) calMode = 1;
        }
        saveCal();
        drawHeader(); drawGraphBackground(); drawMetricsRow(false); drawFooter();
        return;
    }
    if (inBtn(BTN_CAL_X, BTN_CAL_W)) {
        inCalScreen = true; calStep = 0;
        calCapLowV = 0.0f; calCapHighV = 0.0f;
        drawCalScreen();
        return;
    }
    if (inBtn(BTN_UNIT_X, BTN_UNIT_W)) {
        activeUnit = (activeUnit == UNIT_PSI) ? UNIT_MBAR : UNIT_PSI;
        drawMainUI();
        return;
    }
    if (ty >= GFX_Y && ty < GFX_BOT) {
        showTooltip = true; ttipX = tx;
        ttipPSI = gfxXtoPSI(tx); ttipTime = millis();
        drawGraphFull(); drawTooltip();
    }
}

// ============================================================
//  SECTION 20: TOUCH HANDLING - CAL SCREEN
// ============================================================
void handleTouchCal() {
    lgfx::touch_point_t tp[1];
    int n = lcd.getTouch(tp, 1);
    if (n == 0) return;
    int tx = tp[0].x, ty = tp[0].y;
    if (millis() - lastTouch < TOUCH_DEBOUNCE) return;
    lastTouch = millis();

    if (ty >= 200 && ty <= 228 && tx >= 24 && tx <= 74)
        { calLowPSI = max(0.0f, calLowPSI - 5.0f); drawCalScreen(); return; }
    if (ty >= 200 && ty <= 228 && tx >= 80 && tx <= 130)
        { calLowPSI += 5.0f; drawCalScreen(); return; }
    if (ty >= 232 && ty <= 254 && tx >= 24 && tx <= 154)
        { calCapLowV = readRawVoltage(); calStep = 1; drawCalScreen(); return; }

    if (ty >= 200 && ty <= 228 && tx >= 410 && tx <= 460)
        { calHighPSI = max(0.0f, calHighPSI - 5.0f); drawCalScreen(); return; }
    if (ty >= 200 && ty <= 228 && tx >= 466 && tx <= 516)
        { calHighPSI += 5.0f; drawCalScreen(); return; }
    if (ty >= 232 && ty <= 254 && tx >= 410 && tx <= 550)
        { calCapHighV = readRawVoltage(); drawCalScreen(); return; }

    if (ty >= 290 && ty <= 338 && tx >= 14 && tx <= 199) {
        if (calCapHighV > calCapLowV && calCapHighV > 0) {
            calLowV = calCapLowV; calHighV = calCapHighV; calMode = 2;
            saveCal();
        }
        inCalScreen = false; drawMainUI(); return;
    }
    if (ty >= 290 && ty <= 338 && tx >= 214 && tx <= 484) {
        calLowV = 0.5f; calHighV = 4.5f; calLowPSI = 0.0f; calHighPSI = 300.0f;
        atmoOffsetPSI = 0.0f; atmoCalSet = false; calMode = 0;
        saveCal(); inCalScreen = false; drawMainUI(); return;
    }
    if (ty >= 290 && ty <= 338 && tx >= 500)
        { inCalScreen = false; drawMainUI(); return; }
}

// ============================================================
//  SECTION 21: HARDWARE INIT
// ============================================================
void initHardware() {
    Wire.begin(CH422G_I2C_SDA, CH422G_I2C_SCL);
    Wire.setClock(400000);
    delay(50);  // ADD THIS

    Wire.beginTransmission(CH422G_I2C_OE);
    Wire.write(0x01);
    Wire.endTransmission();
    delay(10);  // ADD THIS

    ch422g_write(~(EXIO_TP_RST | EXIO_LCD_BL | EXIO_LCD_RST) & 0xFF);
    delay(50);  // INCREASE from 20
    ch422g_set(EXIO_TP_RST | EXIO_LCD_RST);
    delay(150);  // INCREASE from 120
    ch422g_set(EXIO_LCD_BL);
    delay(50);  // INCREASE from 20
}

// ============================================================
//  SECTION 22: SETUP
// ============================================================
void setup() {
  delay(5000);
    Serial.begin(115200);
    Serial.println("BOOT START");
    Serial.flush();

    loadCal();
    Serial.printf("Cal mode: %s  LoV=%.4f LoP=%.1f  HiV=%.4f HiP=%.1f  AtmoOff=%.2f\n",
        calModeLabel(), calLowV, calLowPSI, calHighV, calHighPSI, atmoOffsetPSI);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(SENSOR_PIN, INPUT);
Serial.println("A");
initHardware();
Serial.println("B");
Serial.println("C");
lcd.init();
bool initOK = lcd.getStartCount() > 0;
Serial.printf("lcd.init done - OK:%d size:%dx%d\n", initOK, lcd.width(), lcd.height());
lcd.setRotation(0);
lcd.fillScreen(TFT_RED);
Serial.println("D");
drawMainUI();
Serial.println("E");


    flowPrev     = readPSI();
    flowPrevTime = millis();

    Serial.println("PSI,STATMIN,STATMAX,DROP,FLOW_PSI_MIN");
}

// ============================================================
//  SECTION 23: MAIN LOOP
// ============================================================
void loop() {
    unsigned long now = millis();

    if (inCalScreen) {
        handleTouchCal();
        static unsigned long lastCalRefresh = 0;
        if (now - lastCalRefresh > 250) { lastCalRefresh = now; drawCalScreen(); }
        return;
    }

    handleTouchMain();

    if (!paused && now - lastSample >= SAMPLE_MS) {
        lastSample = now;
        float psi = readPSI();
        curPSI = psi;

        if (psi < minPSI) minPSI = psi;
        if (psi > maxPSI) maxPSI = psi;
        if (baselineSet) {
            float drop = baselinePSI - psi;
            if (drop > peakDrop) peakDrop = drop;
        }

        unsigned long dt = now - flowPrevTime;
        if (dt >= FLOW_WINDOW_MS) {
            float dpsi = psi - flowPrev;
            float rawFlow = (dpsi / (dt / 1000.0f)) * 60.0f;
            static float smoothFlow = 0.0f;
            smoothFlow     = smoothFlow * 0.8f + rawFlow * 0.2f;
            flowRatePSImin = smoothFlow;
            flowPrev       = psi;
            flowPrevTime   = now;
        }

        gBuf[gHead] = psi;
        gHead = (gHead + 1) % BUF_SIZE;
        if (gFill < BUF_SIZE) gFill++;

        Serial.printf("%.2f,%.2f,%.2f,%.2f,%.1f\n",
            psi,
            minPSI < 1e8f  ? minPSI : 0.0f,
            maxPSI > -1e8f ? maxPSI : 0.0f,
            peakDrop,
            flowRatePSImin);
    }

    if (!paused && now - lastDraw >= DRAW_MS) {
        lastDraw = now;
        if (gFill >= GFX_W) {
            drawGraphFull();
        } else if (gFill >= 2) {
            int c  = gFill;
            int x1 = GFX_W - c,     x2 = GFX_W - c + 1;
            int i1 = ((gHead-c)   % BUF_SIZE + BUF_SIZE) % BUF_SIZE;
            int i2 = ((gHead-c+1) % BUF_SIZE + BUF_SIZE) % BUF_SIZE;
            lcd.drawLine(x1, psiToGfxY(gBuf[i1]), x2, psiToGfxY(gBuf[i2]), C_WAVEFORM);
        }
        if (showTooltip && millis() - ttipTime < 3000) drawTooltip();
    }

    if (now - lastStats >= STATS_MS) {
        lastStats = now;
        drawMetricsRow(false);
        drawFlowBar();
        static bool blink = false;
        uint32_t dotCol = paused ? C_ORANGE : (blink ? C_GREEN : C_PANEL);
        lcd.fillCircle(775, 25, 7, dotCol);
        blink = !blink;
    }
}

