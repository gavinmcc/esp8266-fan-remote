// Auto-generated from devices.json by gen_config.py — do not edit
#pragma once

#define FAN_COUNT     2
#define ALEXA_SLOTS   10

#define PROTO_UC7070T   0
#define PROTO_SMC5060RF 1

struct FanTiming {
    int short_h, long_h, short_l, long_l, gap;
    int sync0_h, sync0_l;   // UC7070T sync pulse; unused for SMC5060RF
};

struct FanCmd {
    int  N;
    int  reps;
    bool fire_always;   // if false, Alexa only fires on turn-on
};

struct FanCmds {
    FanCmd hi, med, low, off, light;
};

struct FanAlexa {
    const char* hi;
    const char* med;
    const char* low;
    const char* off;
    const char* light;
};

struct FanDevice {
    const char* name;
    const char* path;
    int         protocol;
    float       freq_mhz;
    FanTiming   timing;
    FanCmds     cmds;
    FanAlexa    alexa;
};

static const FanDevice FANS[] = {
    {
        "Bedroom", "bedroom",
        PROTO_UC7070T, 303.92f,
        { 10, 24, 18, 32, 320,
          17, 14 },
        { {0, 25, false}, {1, 25, false}, {2, 25, false},
          {4, 50, true}, {5, 4, true} },
        { "Bedroom Fan High", "Bedroom Fan Medium", "Bedroom Fan Low",
          "Bedroom Fan Off", "Bedroom Light" }
    },
    {
        "Girls", "girls",
        PROTO_SMC5060RF, 433.92f,
        { 9, 17, 10, 19, 310,
          0, 0 },
        { {4, 20, false}, {3, 20, false}, {0, 20, false},
          {5, 20, true}, {1, 4, true} },
        { "Girls Fan High", "Girls Fan Medium", "Girls Fan Low",
          "Girls Fan Off", "Girls Light" }
    }
};

// Static thunks for Alexa callbacks — defined in fan_remote.ino
extern void dispatchAlexa(int idx, uint8_t b);

static void alexaThunk0(uint8_t b) { dispatchAlexa(0, b); }
static void alexaThunk1(uint8_t b) { dispatchAlexa(1, b); }
static void alexaThunk2(uint8_t b) { dispatchAlexa(2, b); }
static void alexaThunk3(uint8_t b) { dispatchAlexa(3, b); }
static void alexaThunk4(uint8_t b) { dispatchAlexa(4, b); }
static void alexaThunk5(uint8_t b) { dispatchAlexa(5, b); }
static void alexaThunk6(uint8_t b) { dispatchAlexa(6, b); }
static void alexaThunk7(uint8_t b) { dispatchAlexa(7, b); }
static void alexaThunk8(uint8_t b) { dispatchAlexa(8, b); }
static void alexaThunk9(uint8_t b) { dispatchAlexa(9, b); }

typedef void (*AlexaCallbackFn)(uint8_t);
static const AlexaCallbackFn ALEXA_THUNKS[] = {
    alexaThunk0, alexaThunk1, alexaThunk2, alexaThunk3, alexaThunk4, alexaThunk5, alexaThunk6, alexaThunk7, alexaThunk8, alexaThunk9
};
