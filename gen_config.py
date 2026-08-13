#!/usr/bin/env python3
"""
gen_config.py — reads model_database.json + devices.json and generates fan_remote/fan_config.h

Run before compiling, or use build.sh which does it automatically.
"""

import json
import os
import sys

PROTOCOLS = {
    "UC7070T":   "PROTO_UC7070T",
    "SMC5060RF": "PROTO_SMC5060RF",
}

CMD_ORDER = ["hi", "med", "low", "off", "light"]


def esc(s):
    return s.replace("\\", "\\\\").replace('"', '\\"')


def fmt_cmd(c):
    fa = "true" if c.get("fire_always", False) else "false"
    return f"{{{c['N']}, {c['reps']}, {fa}}}"


def load_devices(here):
    with open(os.path.join(here, "model_database.json")) as f:
        models = {m["model_id"]: m for m in json.load(f)}
    with open(os.path.join(here, "devices.json")) as f:
        raw = json.load(f)

    devices = []
    for dev in raw:
        model_id = dev["model"]
        if model_id not in models:
            sys.exit(f"Unknown model {model_id!r} in devices.json")
        model = models[model_id]

        merged_cmds = {}
        for cmd_name, model_cmd in model["commands"].items():
            cmd = dict(model_cmd)
            cmd.update(dev.get("commands", {}).get(cmd_name, {}))
            if "N" not in cmd:
                sys.exit(
                    f"Device {dev['name']!r} command {cmd_name!r}: "
                    f"N not set in model or device (model {model_id!r} requires per-device N)"
                )
            merged_cmds[cmd_name] = cmd

        devices.append({
            "name":     dev["name"],
            "path":     dev["path"],
            "protocol": model["protocol"],
            "freq_mhz": model["freq_mhz"],
            "timing":   model["timing"],
            "commands": merged_cmds,
            "alexa":    dev["alexa"],
        })
    return devices


def main():
    here     = os.path.dirname(os.path.abspath(__file__))
    out_path = os.path.join(here, "fan_remote", "fan_config.h")

    devices = load_devices(here)

    n = len(devices)
    max_slots = n * 5  # one thunk per device×command

    lines = [
        "// Auto-generated from devices.json by gen_config.py — do not edit",
        "#pragma once",
        "",
        f"#define FAN_COUNT     {n}",
        f"#define ALEXA_SLOTS   {max_slots}",
        "",
        "#define PROTO_UC7070T   0",
        "#define PROTO_SMC5060RF 1",
        "",
        "struct FanTiming {",
        "    int short_h, long_h, short_l, long_l, gap;",
        "    int sync0_h, sync0_l;   // UC7070T sync pulse; unused for SMC5060RF",
        "};",
        "",
        "struct FanCmd {",
        "    int  N;",
        "    int  reps;",
        "    bool fire_always;   // if false, Alexa only fires on turn-on",
        "};",
        "",
        "struct FanCmds {",
        "    FanCmd hi, med, low, off, light;",
        "};",
        "",
        "struct FanAlexa {",
        "    const char* hi;",
        "    const char* med;",
        "    const char* low;",
        "    const char* off;",
        "    const char* light;",
        "};",
        "",
        "struct FanDevice {",
        "    const char* name;",
        "    const char* path;",
        "    int         protocol;",
        "    float       freq_mhz;",
        "    FanTiming   timing;",
        "    FanCmds     cmds;",
        "    FanAlexa    alexa;",
        "};",
        "",
    ]

    entries = []
    for d in devices:
        proto = PROTOCOLS.get(d["protocol"])
        if not proto:
            sys.exit(f"Unknown protocol: {d['protocol']!r}  (choose UC7070T or SMC5060RF)")

        t = d["timing"]
        c = d["commands"]
        a = d["alexa"]

        timing = (
            f"{{ {t['short_h']}, {t['long_h']}, {t['short_l']}, {t['long_l']}, {t['gap']},\n"
            f"          {t.get('sync0_h', 0)}, {t.get('sync0_l', 0)} }}"
        )
        cmds = (
            f"{{ {fmt_cmd(c['hi'])}, {fmt_cmd(c['med'])}, {fmt_cmd(c['low'])},\n"
            f"          {fmt_cmd(c['off'])}, {fmt_cmd(c['light'])} }}"
        )
        alexa = (
            f"{{ \"{esc(a['hi'])}\", \"{esc(a['med'])}\", \"{esc(a['low'])}\",\n"
            f"          \"{esc(a['off'])}\", \"{esc(a['light'])}\" }}"
        )
        entries.append(
            f"    {{\n"
            f"        \"{esc(d['name'])}\", \"{esc(d['path'])}\",\n"
            f"        {proto}, {d['freq_mhz']}f,\n"
            f"        {timing},\n"
            f"        {cmds},\n"
            f"        {alexa}\n"
            f"    }}"
        )

    lines.append("static const FanDevice FANS[] = {")
    lines.append(",\n".join(entries))
    lines.append("};")
    lines.append("")

    # Generate static Alexa thunks (forward-declared; defined in fan_remote.ino)
    lines.append("// Static thunks for Alexa callbacks — defined in fan_remote.ino")
    lines.append("extern void dispatchAlexa(int idx, uint8_t b);")
    lines.append("")
    for i in range(max_slots):
        lines.append(f"static void alexaThunk{i}(uint8_t b) {{ dispatchAlexa({i}, b); }}")
    lines.append("")
    lines.append("typedef void (*AlexaCallbackFn)(uint8_t);")
    lines.append("static const AlexaCallbackFn ALEXA_THUNKS[] = {")
    lines.append("    " + ", ".join(f"alexaThunk{i}" for i in range(max_slots)))
    lines.append("};")
    lines.append("")

    content = "\n".join(lines)
    with open(out_path, "w") as f:
        f.write(content)
    print(f"Generated {out_path}  ({n} device(s), {max_slots} Alexa slots)")


if __name__ == "__main__":
    main()
