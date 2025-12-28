# Quick Fix for Compiler Error

## Internal Compiler Error in Binary.h

If you see this error:
```
internal compiler error: in decl_attributes, at attribs.c:453
```

**This is a bug in the Arduino R4 board support package, not your code.**

### Quick Fix (30 seconds):

**Windows:**
```cmd
cd ptz_controller_r4
fix_compiler_error.bat
```

**Linux/macOS:**
```bash
cd ptz_controller_r4
arduino-cli core update-index
arduino-cli core upgrade arduino:renesas_uno
```

### Alternative: Use Arduino IDE

Arduino IDE 2.3.0+ may have a fixed compiler:
1. Open `ptz_controller_r4.ino` in Arduino IDE
2. Tools → Board → Arduino UNO R4 Minima  
3. Upload

### Still Not Working?

See `TROUBLESHOOTING.md` for detailed solutions.


