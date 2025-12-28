# How to Check Flash Size on FYSETC E4

## Method 1: Using esptool (Recommended)

```bash
# Windows
idf.py -p COM3 flash_id

# Linux/macOS
idf.py -p /dev/ttyUSB0 flash_id
```

Look for output like:
```
Detecting chip type... ESP32
Chip is ESP32-D0WDQ6 (revision 1)
Features: WiFi, BT, Dual Core, 240MHz, VRef calibration in efuse, Coding Scheme None
Crystal is 40MHz
MAC: xx:xx:xx:xx:xx:xx
Uploading stub...
Running stub...
Stub running...
Manufacturer: 20
Device: 4016
Detected flash size: 4MB
```

## Method 2: Check Bootloader Output

When the device boots, the bootloader often prints flash information:
```
I (xxx) boot: ESP-IDF v5.x.x 2nd stage bootloader
I (xxx) boot: compile time xx:xx:xx
I (xxx) boot: chip: ESP32-D0WDQ6 (revision X)
I (xxx) boot: SPI Speed      : 40MHz
I (xxx) boot: SPI Mode       : DIO
I (xxx) boot: SPI Flash Size : 4MB
```

## Method 3: Using esptool.py directly

```bash
# Windows
python -m esptool --port COM3 flash_id

# Linux/macOS  
python -m esptool --port /dev/ttyUSB0 flash_id
```

## Method 4: Check Board Documentation

- FYSETC E4 board specifications
- ESP32 module datasheet (if using a module)
- Check the physical flash chip on the board

## Common Flash Sizes

- **2MB** (0x200000): Common on basic ESP32 boards
- **4MB** (0x400000): Common on development boards
- **8MB** (0x800000): Higher-end boards
- **16MB** (0x1000000): Premium boards

## Adjusting Configuration

After determining your flash size:

1. **Update `sdkconfig.defaults`**:
   - For 2MB: `CONFIG_ESPTOOLPY_FLASHSIZE_2MB=y`
   - For 4MB: `CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y`
   - For 8MB: `CONFIG_ESPTOOLPY_FLASHSIZE_8MB=y`
   - For 16MB: `CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y`

2. **Update `partitions.csv`** to fit your flash size:
   - 2MB: Use smaller partitions (e.g., 960KB each)
   - 4MB: Can use larger partitions (e.g., 1.5MB each)
   - 8MB+: Can use very large partitions (e.g., 3MB+ each)

3. **Delete `sdkconfig`** and rebuild:
   ```bash
   rm sdkconfig
   idf.py reconfigure
   idf.py build
   ```

