# Bitfocus Companion Setup for Camera Controller

This guide explains how to configure Bitfocus Companion to recall camera presets via HTTP requests.

## Controller API Endpoint

**URL:** `http://<CONTROLLER_IP>/api/preset/goto`  
**Method:** `POST`  
**Content-Type:** `application/json`  
**Body:** `{"index": <preset_number>}`

## Available Presets

The controller supports 16 presets (0-15):
- **Preset 0:** Home position (0,0,0)
- **Presets 1-15:** User-defined camera positions

## Bitfocus Companion Configuration

### Step 1: Add HTTP Request Action

1. In Bitfocus Companion, create a new button or edit an existing one
2. Click **"+ Action"**
3. Select **"HTTP Request"** from the action list

### Step 2: Configure HTTP Request

For each preset, configure the HTTP Request action as follows:

#### Basic Settings:
- **Method:** `POST`
- **URL:** `http://<CONTROLLER_IP>/api/preset/goto`
  - (Replace `<CONTROLLER_IP>` with your controller's actual IP address)
- **Body Type:** `JSON`

#### Request Body (JSON):
```json
{
  "index": <preset_number>
}
```

Replace `<preset_number>` with the preset index (0-15).

### Step 3: Example Configurations

#### Preset 0 (Home):
- **URL:** `http://<CONTROLLER_IP>/api/preset/goto`
- **Body:** 
```json
{
  "index": 0
}
```

#### Preset 1:
- **URL:** `http://<CONTROLLER_IP>/api/preset/goto`
- **Body:**
```json
{
  "index": 1
}
```

(Continue for presets 2-15 following the same pattern)

### Step 4: Advanced Options (Optional)

#### Headers:
You may need to explicitly set the Content-Type header:
- **Header Name:** `Content-Type`
- **Header Value:** `application/json`

#### Timeout:
Set an appropriate timeout (e.g., 2000ms) if the controller doesn't respond quickly.

## Saving Presets in Bitfocus Companion

### Overview

You can save the camera's current position (pan, tilt, zoom) to any preset slot (1-15) using the `/api/preset/save` endpoint. This is useful for creating quick save buttons or updating existing presets.

**Important Notes:**
- Presets 1-15 can be saved
- Preset 0 is read-only (home position at 0,0,0)
- Saving a preset overwrites any previously saved data at that index
- The current camera position is automatically captured when the save command is sent

### Configuration Steps

1. Create a button for saving presets (or use an existing button)
2. Click **"+ Action"**
3. Select **"HTTP Request"** from the action list

### Basic Save Preset Setup

#### Settings:
- **Method:** `POST`
- **URL:** `http://<CONTROLLER_IP>/api/preset/save`
  - (Replace `<CONTROLLER_IP>` with your controller's actual IP address)
- **Body Type:** `JSON`

#### Request Body (JSON):
```json
{
  "index": <preset_number>
}
```

Replace `<preset_number>` with the preset index (1-15).

### Example Configurations

#### Save to Preset 1:
- **URL:** `http://<CONTROLLER_IP>/api/preset/save`
- **Body:**
```json
{
  "index": 1
}
```

#### Save to Preset 2:
- **URL:** `http://<CONTROLLER_IP>/api/preset/save`
- **Body:**
```json
{
  "index": 2
}
```

(Continue for presets 3-15 following the same pattern)

### Workflow: Position → Save → Recall

1. **Position the camera** using PTZ jog commands or manual positioning
2. **Save the position** by pressing your save preset button
3. **Recall the preset** later using the preset recall button

### Tips for Save Buttons

1. **Numbered Buttons:** Create dedicated save buttons for each preset (e.g., "Save Preset 1", "Save Preset 2", etc.)
2. **Variable Index:** If Companion supports it, you could use a variable for the preset index to create a single "Save Current" button that saves to a selected slot
3. **Visual Feedback:** Consider adding a confirmation message or visual indicator when a preset is successfully saved
4. **Save and Recall:** Create button pairs (Save Preset X + Recall Preset X) for easy preset management

### Advanced: Save Multiple Presets

If you want to save multiple presets in sequence, you can create a button with multiple actions:

**Action 1:** Save to Preset 1
```json
{"index": 1}
```

**Action 2:** Add delay (optional, e.g., 100ms)

**Action 3:** Save to Preset 2
```json
{"index": 2}
```

*Note: This saves the same position to multiple presets, which may or may not be what you want.*

## Using Variables

If your controller IP changes or you want to use variables, you can use Companion's variable syntax:
- **URL:** `http://$(internal:camera_ip)/api/preset/goto`
- Where `camera_ip` is a custom variable you define with your controller's IP address

## Other Available API Endpoints

The controller also supports these endpoints:

### Stop Movement:
- **Method:** `POST`
- **URL:** `http://<CONTROLLER_IP>/api/command`
- **Body:**
```json
{
  "command": "stop"
}
```

### Home Camera:
- **Method:** `POST`
- **URL:** `http://<CONTROLLER_IP>/api/command`
- **Body:**
```json
{
  "command": "home"
}
```

### Get Current Positions:
- **Method:** `GET`
- **URL:** `http://<CONTROLLER_IP>/api/positions`
- Returns JSON with current pan, tilt, and zoom positions

### Save Preset:
- **Method:** `POST`
- **URL:** `http://<CONTROLLER_IP>/api/preset/save`
- **Body:** `{"index": <preset_number>}`
- Saves the current camera position to the specified preset slot
- **Available slots:** 1-15 (preset 0 is read-only)
- Overwrites any existing preset data at the specified index

### PTZ Jog Control (Velocity Control):
- **Method:** `POST`
- **URL:** `http://<CONTROLLER_IP>/api/velocity`
- **Body:** `{"pan": <value>, "tilt": <value>, "zoom": <value>}`
- Controls continuous movement of pan, tilt, and zoom axes
- **Velocity Ranges:**
  - Pan: -1200 to +1200 steps/sec (negative = left, positive = right)
  - Tilt: -1200 to +1200 steps/sec (negative = down, positive = up)
  - Zoom: -130 to +130 steps/sec (negative = out, positive = in)
- To stop movement, send `{"pan": 0, "tilt": 0, "zoom": 0}`

## PTZ Jog Commands in Bitfocus Companion

### Setting Up Jog Buttons

You can create buttons for PTZ jogging (continuous movement while button is held) using HTTP Request actions with button press and release behaviors.

#### Basic Jog Setup

1. Create a button for each direction (Pan Left, Pan Right, Tilt Up, Tilt Down, Zoom In, Zoom Out)
2. For each button, add **two actions:**
   - **On Button Press:** Start movement
   - **On Button Release:** Stop movement

#### Example: Pan Left Button

**Action 1 (Button Press):**
- **Method:** `POST`
- **URL:** `http://<CONTROLLER_IP>/api/velocity`
- **Body Type:** `JSON`
- **Body:**
```json
{
  "pan": -600,
  "tilt": 0,
  "zoom": 0
}
```
- This sets pan velocity to -600 steps/sec (moving left)

**Action 2 (Button Release):**
- **Method:** `POST`
- **URL:** `http://<CONTROLLER_IP>/api/velocity`
- **Body Type:** `JSON`
- **Body:**
```json
{
  "pan": 0,
  "tilt": 0,
  "zoom": 0
}
```
- This stops all movement

#### Velocity Values for Jogging

Use moderate velocity values for smooth, controllable jogging:
- **Slow jog:** ±100-300 steps/sec (precise control)
- **Medium jog:** ±400-600 steps/sec (standard speed)
- **Fast jog:** ±800-1000 steps/sec (rapid movement)

### Stop Button

Create a dedicated stop button for emergency stops:

**Action (Button Press):**
- **Method:** `POST`
- **URL:** `http://<CONTROLLER_IP>/api/velocity`
- **Body:**
```json
{
  "pan": 0,
  "tilt": 0,
  "zoom": 0
}
```

Or use the stop command endpoint:
- **Method:** `POST`
- **URL:** `http://<CONTROLLER_IP>/api/command`
- **Body:**
```json
{
  "command": "stop"
}
```

### Tips for Jog Controls

1. **Variable Speeds:** Create multiple buttons with different velocity values (slow, medium, fast) for the same direction
2. **Precision Mode:** Consider creating a "precision mode" toggle that reduces all velocity values when enabled
3. **Safety:** Always include stop commands on button release to prevent runaway movement
4. **Feedback:** Use button states/colors to indicate when jog commands are active

## Troubleshooting

1. **Connection Issues:**
   - Verify the controller IP address is correct
   - Ensure the controller is powered on and connected to the network
   - Check that the HTTP server is running (access the web UI in a browser)

2. **401/403 Errors:**
   - The controller doesn't require authentication, so these shouldn't occur
   - If you see these, check the URL is correct

3. **404 Errors:**
   - Verify the endpoint path is exactly `/api/preset/goto`
   - Check that you're using POST method, not GET

4. **400 Bad Request:**
   - Verify the JSON body is correctly formatted
   - For preset recall: Ensure `index` is a number between 0-15
   - For preset save: Ensure `index` is a number between 1-15 (preset 0 cannot be saved)
   - Check that Content-Type header is set to `application/json`

5. **500 Server Error:**
   - The preset may not be valid/initialized
   - Check the controller logs for more details

6. **Save Preset Not Working:**
   - Verify the preset index is between 1-15 (preset 0 is read-only)
   - Ensure the camera is positioned where you want before saving
   - Check that the HTTP request is successfully sent (check Companion logs)
   - Verify the camera has actually moved to the desired position before saving

7. **Jog Commands Not Stopping:**
   - Ensure button release actions are configured to send stop command (all velocities = 0)
   - Verify the stop HTTP request is being sent when button is released
   - Manually send stop command if movement continues

## Testing

You can test the API endpoint using curl:

```bash
curl -X POST http://<CONTROLLER_IP>/api/preset/goto \
  -H "Content-Type: application/json" \
  -d '{"index": 1}'
```

Or using PowerShell:

```powershell
Invoke-RestMethod -Uri "http://<CONTROLLER_IP>/api/preset/goto" `
  -Method POST `
  -ContentType "application/json" `
  -Body '{"index": 1}'
```

### Testing Save Preset

Test saving to preset 1:
```bash
curl -X POST http://<CONTROLLER_IP>/api/preset/save \
  -H "Content-Type: application/json" \
  -d '{"index": 1}'
```

Or using PowerShell:
```powershell
Invoke-RestMethod -Uri "http://<CONTROLLER_IP>/api/preset/save" `
  -Method POST `
  -ContentType "application/json" `
  -Body '{"index": 1}'
```

### Testing PTZ Jog Commands

Test pan left movement:
```bash
curl -X POST http://<CONTROLLER_IP>/api/velocity \
  -H "Content-Type: application/json" \
  -d '{"pan": -500, "tilt": 0, "zoom": 0}'
```

Test tilt up movement:
```bash
curl -X POST http://<CONTROLLER_IP>/api/velocity \
  -H "Content-Type: application/json" \
  -d '{"pan": 0, "tilt": 500, "zoom": 0}'
```

Test zoom in:
```bash
curl -X POST http://<CONTROLLER_IP>/api/velocity \
  -H "Content-Type: application/json" \
  -d '{"pan": 0, "tilt": 0, "zoom": 50}'
```

Stop all movement:
```bash
curl -X POST http://<CONTROLLER_IP>/api/velocity \
  -H "Content-Type: application/json" \
  -d '{"pan": 0, "tilt": 0, "zoom": 0}'
```

Or using PowerShell:
```powershell
# Pan left
Invoke-RestMethod -Uri "http://<CONTROLLER_IP>/api/velocity" `
  -Method POST `
  -ContentType "application/json" `
  -Body '{"pan": -500, "tilt": 0, "zoom": 0}'

# Stop movement
Invoke-RestMethod -Uri "http://<CONTROLLER_IP>/api/velocity" `
  -Method POST `
  -ContentType "application/json" `
  -Body '{"pan": 0, "tilt": 0, "zoom": 0}'
```
