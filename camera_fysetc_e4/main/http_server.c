/**
 * @file http_server.c
 * @brief HTTP server for web-based motor control
 */

#include "http_server.h"
#include "motion_controller.h"
#include "motion_planner.h"  // For MAX_VELOCITY_* macros
#include "wifi_manager.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "cJSON.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_partition.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char* TAG = "http_server";
static httpd_handle_t server_handle = NULL;

// Web UI HTML (embedded)
static const char html_page[] = 
"<!DOCTYPE html>"
"<html><head>"
"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
"<title>PTZ Camera Control</title>"
"<style>"
"body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }"
".container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }"
"h1 { color: #333; text-align: center; }"
".section { margin: 20px 0; padding: 15px; background: #f9f9f9; border-radius: 5px; }"
".section h2 { margin-top: 0; color: #555; }"
".control-group { margin: 15px 0; }"
"label { display: block; margin-bottom: 5px; font-weight: bold; color: #666; }"
"input[type=\"range\"] { width: 100%; margin: 10px 0; }"
"input[type=\"text\"] { width: 100px; padding: 5px; margin: 0 10px; }"
"button { padding: 10px 20px; margin: 5px; font-size: 16px; cursor: pointer; border: none; border-radius: 5px; }"
".btn-primary { background: #4CAF50; color: white; }"
".btn-primary:hover { background: #45a049; }"
".btn-secondary { background: #2196F3; color: white; }"
".btn-secondary:hover { background: #0b7dda; }"
".btn-danger { background: #f44336; color: white; }"
".btn-danger:hover { background: #da190b; }"
".btn-warning { background: #ff9800; color: white; }"
".btn-warning:hover { background: #e68900; }"
".status { padding: 10px; margin: 10px 0; border-radius: 5px; }"
".status-info { background: #e3f2fd; color: #1976d2; }"
".status-success { background: #e8f5e9; color: #388e3c; }"
"#positions { font-family: monospace; font-size: 18px; }"
".preset-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(150px, 1fr)); gap: 10px; margin: 10px 0; }"
".preset-btn { padding: 15px; font-size: 14px; position: relative; }"
".preset-edit-btn { position: absolute; top: 2px; right: 2px; padding: 2px 6px; font-size: 10px; background: rgba(0,0,0,0.5); color: white; border: none; border-radius: 3px; cursor: pointer; }"
".preset-edit-btn:hover { background: rgba(0,0,0,0.7); }"
".modal { display: none; position: fixed; z-index: 1000; left: 0; top: 0; width: 100%; height: 100%; background: rgba(0,0,0,0.5); }"
".modal-content { background: white; margin: 5% auto; padding: 20px; border-radius: 10px; width: 90%; max-width: 600px; max-height: 80vh; overflow-y: auto; }"
".modal-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 20px; }"
".close { color: #aaa; font-size: 28px; font-weight: bold; cursor: pointer; }"
".close:hover { color: #000; }"
".form-group { margin: 15px 0; }"
".form-group label { display: block; margin-bottom: 5px; font-weight: bold; }"
".form-group input, .form-group select { width: 100%; padding: 8px; border: 1px solid #ddd; border-radius: 4px; box-sizing: border-box; }"
".form-row { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }"
".joystick-container { display: flex; gap: 20px; margin: 20px 0; flex-wrap: wrap; }"
".joystick-wrapper { display: flex; flex-direction: column; align-items: center; }"
".joystick-2d { width: 200px; height: 200px; border: 3px solid #333; border-radius: 50%; position: relative; background: #f5f5f5; cursor: crosshair; touch-action: none; }"
".joystick-2d::before { content: ''; position: absolute; width: 30px; height: 30px; background: #4CAF50; border-radius: 50%; top: calc(50% + var(--y, 0px)); left: calc(50% + var(--x, 0px)); transform: translate(-50%, -50%); transition: transform 0.05s; }"
".joystick-2d.active::before { background: #45a049; }"
".joystick-1d { width: 60px; height: 200px; border: 3px solid #333; border-radius: 30px; position: relative; background: #f5f5f5; cursor: ns-resize; touch-action: none; }"
".joystick-1d::before { content: ''; position: absolute; width: 50px; height: 30px; background: #2196F3; border-radius: 15px; left: 50%; top: calc(50% + var(--y, 0px)); transform: translate(-50%, -50%); transition: top 0.05s; }"
".joystick-1d.active::before { background: #0b7dda; }"
".joystick-label { margin-top: 10px; font-weight: bold; color: #666; }"
".joystick-value { margin-top: 5px; font-family: monospace; color: #333; }"
"</style>"
"</head><body>"
"<div class=\"container\">"
"<h1>PTZ Camera Control</h1>"
"<div class=\"section\">"
"<h2>Position Status</h2>"
"<div id=\"positions\" class=\"status status-info\">Loading...</div>"
"</div>"
"<div class=\"section\">"
"<h2>Joystick Control</h2>"
"<div class=\"joystick-container\">"
"<div class=\"joystick-wrapper\">"
"<div class=\"joystick-label\">PAN & TILT</div>"
"<div id=\"joystick_xy\" class=\"joystick-2d\"></div>"
"<div class=\"joystick-value\" id=\"xy_value\">X: 0.0, Y: 0.0</div>"
"</div>"
"<div class=\"joystick-wrapper\">"
"<div class=\"joystick-label\">ZOOM</div>"
"<div id=\"joystick_z\" class=\"joystick-1d\"></div>"
"<div class=\"joystick-value\" id=\"z_value\">Z: 0.0</div>"
"</div>"
"</div>"
"</div>"
"<div class=\"section\">"
"<h2>Velocity Control (Sliders)</h2>"
"<div class=\"control-group\">"
"<label>PAN: <span id=\"pan_val\">0.0</span> steps/s</label>"
"%SLIDER_PAN%"
"</div>"
"<div class=\"control-group\">"
"<label>TILT: <span id=\"tilt_val\">0.0</span> steps/s</label>"
"%SLIDER_TILT%"
"</div>"
"<div class=\"control-group\">"
"<label>ZOOM: <span id=\"zoom_val\">0.0</span> steps/s</label>"
"%SLIDER_ZOOM%"
"</div>"
"</div>"
"<div class=\"section\">"
"<h2>Commands</h2>"
"<button class=\"btn-primary\" onclick=\"sendCommand('home')\">Home All Axes</button>"
"<button class=\"btn-danger\" onclick=\"sendCommand('stop')\">Stop</button>"
"<button class=\"btn-secondary\" onclick=\"sendCommand('precision')\">Toggle Precision</button>"
"</div>"
"<div class=\"section\">"
"<h2>Firmware Update (OTA)</h2>"
"<p style=\"color: #666; font-size: 14px;\">Upload a new firmware binary file to update the device over-the-air.</p>"
"<input type=\"file\" id=\"firmware_file\" accept=\".bin\" style=\"margin: 10px 0; padding: 5px; width: 100%; max-width: 400px; border: 1px solid #ddd; border-radius: 4px;\">"
"<button class=\"btn-warning\" onclick=\"uploadFirmware()\" style=\"margin-top: 10px;\">Upload Firmware</button>"
"<div id=\"ota_status\" style=\"margin-top: 10px; padding: 10px; border-radius: 5px; display: none;\"></div>"
"</div>"
"<div class=\"section\">"
"<h2>Presets</h2>"
"<div class=\"preset-grid\" id=\"preset_grid\"></div>"
"<div style=\"margin-top: 10px;\">"
"<button class=\"btn-warning\" onclick=\"savePreset()\">Save Current Position</button>"
"<input type=\"number\" id=\"preset_save_idx\" min=\"0\" max=\"15\" value=\"0\" style=\"width: 60px; padding: 5px; margin-left: 10px;\">"
"</div>"
"</div>"
"<div id=\"preset_editor_modal\" class=\"modal\">"
"<div class=\"modal-content\">"
"<div class=\"modal-header\">"
"<h2>Edit Preset <span id=\"editor_preset_idx\">0</span></h2>"
"<span class=\"close\" onclick=\"closePresetEditor()\">&times;</span>"
"</div>"
"<form id=\"preset_editor_form\">"
"<div class=\"form-group\">"
"<h3>Position</h3>"
"<div class=\"form-row\">"
"<div><label>PAN:</label><input type=\"number\" id=\"editor_pos_pan\" step=\"0.1\"></div>"
"<div><label>TILT:</label><input type=\"number\" id=\"editor_pos_tilt\" step=\"0.1\"></div>"
"<div><label>ZOOM:</label><input type=\"number\" id=\"editor_pos_zoom\" step=\"0.1\"></div>"
"</div>"
"</div>"
"<div class=\"form-group\">"
"<h3>Motion Parameters</h3>"
"<div class=\"form-row\">"
"<div><label>Duration (s, 0=auto):</label><input type=\"number\" id=\"editor_duration\" min=\"0\" step=\"0.1\"></div>"
"<div><label>Max Speed Scale:</label><input type=\"number\" id=\"editor_speed_scale\" min=\"0\" step=\"0.1\"></div>"
"</div>"
"<div class=\"form-row\">"
"<div><label>Speed Multiplier:</label><input type=\"number\" id=\"editor_speed_mult\" min=\"0.1\" max=\"10\" step=\"0.1\" value=\"1.0\"></div>"
"<div><label>Accel Multiplier:</label><input type=\"number\" id=\"editor_accel_mult\" min=\"0.1\" max=\"10\" step=\"0.1\" value=\"1.0\"></div>"
"</div>"
"<div><label>Easing Type:</label>"
"<select id=\"editor_easing\">"
"<option value=\"0\">Linear</option>"
"<option value=\"1\">Smootherstep</option>"
"<option value=\"2\">Sigmoid</option>"
"</select></div>"
"<div><label>Approach Mode:</label>"
"<select id=\"editor_approach\">"
"<option value=\"0\">Direct</option>"
"<option value=\"1\">Home First</option>"
"<option value=\"2\">Safe Route</option>"
"</select></div>"
"</div>"
"<div class=\"form-group\">"
"<h3>Advanced</h3>"
"<div><label>Arrival Overshoot:</label><input type=\"number\" id=\"editor_overshoot\" min=\"0\" max=\"0.01\" step=\"0.001\"></div>"
"<div><label><input type=\"checkbox\" id=\"editor_precision\"> Precision Preferred</label></div>"
"</div>"
"<div style=\"margin-top: 20px; text-align: right;\">"
"<button type=\"button\" class=\"btn-secondary\" onclick=\"closePresetEditor()\">Cancel</button>"
"<button type=\"button\" class=\"btn-primary\" onclick=\"savePresetEditor()\" style=\"margin-left: 10px;\">Save</button>"
"</div>"
"</form>"
"</div>"
"</div>"
"</div>"
"<script>"
"let precisionMode = false;"
"let updatePosInterval;"
"let joystickXYActive = false;"
"let joystickZActive = false;"
"let currentVelocities = {pan: 0, tilt: 0, zoom: 0};"
"const MAX_VELOCITY = 500;"
"const MAX_VELOCITY_ZOOM = 50;"
"function updatePositions() {"
"  const posEl = document.getElementById('positions');"
"  if (!posEl) {"
"    console.error('positions element not found');"
"    return;"
"  }"
"  fetch('/api/positions').then(r => {"
"    if (!r.ok) {"
"      throw new Error('HTTP ' + r.status);"
"    }"
"    return r.json();"
"  }).then(data => {"
"    if (data && typeof data.pan === 'number' && typeof data.tilt === 'number' && typeof data.zoom === 'number') {"
"      posEl.textContent = `PAN: ${data.pan.toFixed(1)} | TILT: ${data.tilt.toFixed(1)} | ZOOM: ${data.zoom.toFixed(1)}`;"
"    } else {"
"      console.error('Invalid position data:', data);"
"      posEl.textContent = 'Error: Invalid data';"
"    }"
"  }).catch(e => {"
"    console.error('Failed to fetch positions:', e);"
"    if (posEl) {"
"      posEl.textContent = 'Error loading position';"
"    }"
"  });"
"}"
"function sendVelocities(pan, tilt, zoom) {"
"  currentVelocities = {pan, tilt, zoom};"
"  fetch('/api/velocity', {"
"    method: 'POST',"
"    headers: { 'Content-Type': 'application/json' },"
"    body: JSON.stringify({pan, tilt, zoom})"
"  }).catch(e => console.error('Failed to set velocity:', e));"
"}"
"function updateJoystickXY(x, y) {"
"  const joystick = document.getElementById('joystick_xy');"
"  const rect = joystick.getBoundingClientRect();"
"  const centerX = rect.left + rect.width / 2;"
"  const centerY = rect.top + rect.height / 2;"
"  const radius = rect.width / 2 - 15;"
"  const dx = x - centerX;"
"  const dy = y - centerY;"
"  const distance = Math.min(Math.sqrt(dx * dx + dy * dy), radius);"
"  const angle = Math.atan2(dy, dx);"
"  const posX = Math.cos(angle) * distance;"
"  const posY = Math.sin(angle) * distance;"
"  joystick.style.setProperty('--x', posX + 'px');"
"  joystick.style.setProperty('--y', posY + 'px');"
"  const normalizedX = posX / radius;"
"  const normalizedY = posY / radius;"
"  const velX = normalizedX * MAX_VELOCITY;"
"  const velY = normalizedY * MAX_VELOCITY;"
"  document.getElementById('xy_value').textContent = `X: ${velX.toFixed(1)}, Y: ${velY.toFixed(1)}`;"
"  sendVelocities(velX, velY, currentVelocities.zoom);"
"  document.getElementById('pan_vel').value = velX;"
"  document.getElementById('tilt_vel').value = velY;"
"  document.getElementById('pan_val').textContent = velX.toFixed(1);"
"  document.getElementById('tilt_val').textContent = velY.toFixed(1);"
"}"
"function updateJoystickZ(y) {"
"  const joystick = document.getElementById('joystick_z');"
"  const rect = joystick.getBoundingClientRect();"
"  const centerY = rect.top + rect.height / 2;"
"  const height = rect.height - 30;"
"  const dy = y - centerY;"
"  const normalized = Math.max(-1, Math.min(1, dy / (height / 2)));"
"  const posY = normalized * (height / 2);"
"  joystick.style.setProperty('--y', posY + 'px');"
"  const velZ = normalized * MAX_VELOCITY_ZOOM;"
"  document.getElementById('z_value').textContent = `Z: ${velZ.toFixed(1)}`;"
"  sendVelocities(currentVelocities.pan, currentVelocities.tilt, velZ);"
"  document.getElementById('zoom_vel').value = velZ;"
"  document.getElementById('zoom_val').textContent = velZ.toFixed(1);"
"}"
"function resetJoystickXY() {"
"  const joystick = document.getElementById('joystick_xy');"
"  joystick.classList.remove('active');"
"  joystick.style.setProperty('--x', '0px');"
"  joystick.style.setProperty('--y', '0px');"
"  document.getElementById('xy_value').textContent = 'X: 0.0, Y: 0.0';"
"  sendVelocities(0, 0, currentVelocities.zoom);"
"  document.getElementById('pan_vel').value = 0;"
"  document.getElementById('tilt_vel').value = 0;"
"  document.getElementById('pan_val').textContent = '0.0';"
"  document.getElementById('tilt_val').textContent = '0.0';"
"}"
"function resetJoystickZ() {"
"  const joystick = document.getElementById('joystick_z');"
"  joystick.classList.remove('active');"
"  joystick.style.setProperty('--y', '0px');"
"  document.getElementById('z_value').textContent = 'Z: 0.0';"
"  sendVelocities(currentVelocities.pan, currentVelocities.tilt, 0);"
"  document.getElementById('zoom_vel').value = 0;"
"  document.getElementById('zoom_val').textContent = '0.0';"
"}"
"const joystickXY = document.getElementById('joystick_xy');"
"joystickXY.addEventListener('mousedown', (e) => {"
"  joystickXYActive = true;"
"  joystickXY.classList.add('active');"
"  updateJoystickXY(e.clientX, e.clientY);"
"});"
"joystickXY.addEventListener('mousemove', (e) => {"
"  if (joystickXYActive) updateJoystickXY(e.clientX, e.clientY);"
"});"
"joystickXY.addEventListener('mouseup', () => {"
"  joystickXYActive = false;"
"  resetJoystickXY();"
"});"
"joystickXY.addEventListener('mouseleave', () => {"
"  if (joystickXYActive) {"
"    joystickXYActive = false;"
"    resetJoystickXY();"
"  }"
"});"
"joystickXY.addEventListener('touchstart', (e) => {"
"  e.preventDefault();"
"  joystickXYActive = true;"
"  joystickXY.classList.add('active');"
"  const touch = e.touches[0];"
"  updateJoystickXY(touch.clientX, touch.clientY);"
"});"
"joystickXY.addEventListener('touchmove', (e) => {"
"  e.preventDefault();"
"  if (joystickXYActive && e.touches.length > 0) {"
"    const touch = e.touches[0];"
"    updateJoystickXY(touch.clientX, touch.clientY);"
"  }"
"});"
"joystickXY.addEventListener('touchend', (e) => {"
"  e.preventDefault();"
"  joystickXYActive = false;"
"  resetJoystickXY();"
"});"
"const joystickZ = document.getElementById('joystick_z');"
"joystickZ.addEventListener('mousedown', (e) => {"
"  joystickZActive = true;"
"  joystickZ.classList.add('active');"
"  updateJoystickZ(e.clientY);"
"});"
"joystickZ.addEventListener('mousemove', (e) => {"
"  if (joystickZActive) updateJoystickZ(e.clientY);"
"});"
"joystickZ.addEventListener('mouseup', () => {"
"  joystickZActive = false;"
"  resetJoystickZ();"
"});"
"joystickZ.addEventListener('mouseleave', () => {"
"  if (joystickZActive) {"
"    joystickZActive = false;"
"    resetJoystickZ();"
"  }"
"});"
"joystickZ.addEventListener('touchstart', (e) => {"
"  e.preventDefault();"
"  joystickZActive = true;"
"  joystickZ.classList.add('active');"
"  const touch = e.touches[0];"
"  updateJoystickZ(touch.clientY);"
"});"
"joystickZ.addEventListener('touchmove', (e) => {"
"  e.preventDefault();"
"  if (joystickZActive && e.touches.length > 0) {"
"    const touch = e.touches[0];"
"    updateJoystickZ(touch.clientY);"
"  }"
"});"
"joystickZ.addEventListener('touchend', (e) => {"
"  e.preventDefault();"
"  joystickZActive = false;"
"  resetJoystickZ();"
"});"
"// Document-level handlers to catch mouse/touch releases outside control bounds"
"// These use capture phase to catch events even if they don't bubble to document"
"function handleGlobalMouseUp() {"
"  if (joystickXYActive) {"
"    joystickXYActive = false;"
"    resetJoystickXY();"
"  }"
"  if (joystickZActive) {"
"    joystickZActive = false;"
"    resetJoystickZ();"
"  }"
"}"
"document.addEventListener('mouseup', handleGlobalMouseUp, true);"
"document.addEventListener('touchend', handleGlobalMouseUp, true);"
"document.addEventListener('touchcancel', handleGlobalMouseUp, true);"
"document.addEventListener('mousemove', (e) => {"
"  if (joystickXYActive) updateJoystickXY(e.clientX, e.clientY);"
"  if (joystickZActive) updateJoystickZ(e.clientY);"
"});"
"function updateVelocities() {"
"  if (joystickXYActive || joystickZActive) return;"
"  const pan = parseFloat(document.getElementById('pan_vel').value);"
"  const tilt = parseFloat(document.getElementById('tilt_vel').value);"
"  const zoom = parseFloat(document.getElementById('zoom_vel').value);"
"  document.getElementById('pan_val').textContent = pan.toFixed(1);"
"  document.getElementById('tilt_val').textContent = tilt.toFixed(1);"
"  document.getElementById('zoom_val').textContent = zoom.toFixed(1);"
"  sendVelocities(pan, tilt, zoom);"
"}"
"document.getElementById('pan_vel').addEventListener('input', updateVelocities);"
"document.getElementById('tilt_vel').addEventListener('input', updateVelocities);"
"document.getElementById('zoom_vel').addEventListener('input', updateVelocities);"
"function sendCommand(cmd) {"
"  fetch('/api/command', {"
"    method: 'POST',"
"    headers: { 'Content-Type': 'application/json' },"
"    body: JSON.stringify({command: cmd})"
"  }).then(r => r.json()).then(data => {"
"    if (data.status === 'ok') {"
"      alert('Command executed: ' + cmd);"
"      if (cmd === 'precision') precisionMode = !precisionMode;"
"    } else {"
"      alert('Error: ' + (data.error || 'Unknown error'));"
"    }"
"  }).catch(e => {"
"    console.error('Command failed:', e);"
"    alert('Failed to send command');"
"  });"
"}"
"function gotoPreset(idx) {"
"  fetch('/api/preset/goto', {"
"    method: 'POST',"
"    headers: { 'Content-Type': 'application/json' },"
"    body: JSON.stringify({index: idx})"
"  }).then(r => r.json()).then(data => {"
"    if (data.status === 'ok') {"
"      alert('Moving to preset ' + idx);"
"    } else {"
"      alert('Error: ' + (data.error || 'Failed to move to preset'));"
"    }"
"  }).catch(e => {"
"    console.error('Goto preset failed:', e);"
"    alert('Failed to move to preset');"
"  });"
"}"
"function savePreset() {"
"  const idx = parseInt(document.getElementById('preset_save_idx').value);"
"  fetch('/api/preset/save', {"
"    method: 'POST',"
"    headers: { 'Content-Type': 'application/json' },"
"    body: JSON.stringify({index: idx})"
"  }).then(r => r.json()).then(data => {"
"    if (data.status === 'ok') {"
"      alert('Preset ' + idx + ' saved!');"
"      createPresetButtons();"
"    } else {"
"      alert('Error: ' + (data.error || 'Failed to save preset'));"
"    }"
"  }).catch(e => {"
"    console.error('Save preset failed:', e);"
"    alert('Failed to save preset');"
"  });"
"}"
"function createPresetButtons() {"
"  const grid = document.getElementById('preset_grid');"
"  if (!grid) {"
"    console.error('preset_grid element not found');"
"    return;"
"  }"
"  grid.innerHTML = '';"
"  for (let i = 0; i < 16; i++) {"
"    const btn = document.createElement('button');"
"    btn.className = 'btn-secondary preset-btn';"
"    btn.textContent = 'Preset ' + i;"
"    btn.onclick = () => gotoPreset(i);"
"    const editBtn = document.createElement('button');"
"    editBtn.className = 'preset-edit-btn';"
"    editBtn.textContent = 'Edit';"
"    editBtn.onclick = (e) => { e.stopPropagation(); openPresetEditor(i); };"
"    btn.appendChild(editBtn);"
"    grid.appendChild(btn);"
"  }"
"}"
"let currentEditingPreset = -1;"
"function openPresetEditor(idx) {"
"  currentEditingPreset = idx;"
"  document.getElementById('editor_preset_idx').textContent = idx;"
"  document.getElementById('preset_editor_modal').style.display = 'block';"
"  fetch('/api/preset/get?index=' + idx).then(r => r.json()).then(data => {"
"    if (data.status === 'ok' && data.preset) {"
"      const p = data.preset;"
"      document.getElementById('editor_pos_pan').value = p.pos[0] || 0;"
"      document.getElementById('editor_pos_tilt').value = p.pos[1] || 0;"
"      document.getElementById('editor_pos_zoom').value = p.pos[2] || 0;"
"      document.getElementById('editor_duration').value = p.duration_s || 0;"
"      document.getElementById('editor_speed_scale').value = p.max_speed_scale || 0;"
"      document.getElementById('editor_speed_mult').value = p.speed_multiplier || 1.0;"
"      document.getElementById('editor_accel_mult').value = p.accel_multiplier || 1.0;"
"      document.getElementById('editor_easing').value = p.easing_type || 0;"
"      document.getElementById('editor_approach').value = p.approach_mode || 0;"
"      document.getElementById('editor_overshoot').value = p.arrival_overshoot || 0;"
"      document.getElementById('editor_precision').checked = p.precision_preferred || false;"
"    } else {"
"      document.getElementById('editor_pos_pan').value = 0;"
"      document.getElementById('editor_pos_tilt').value = 0;"
"      document.getElementById('editor_pos_zoom').value = 0;"
"      document.getElementById('editor_duration').value = 0;"
"      document.getElementById('editor_speed_scale').value = 0;"
"      document.getElementById('editor_speed_mult').value = 1.0;"
"      document.getElementById('editor_accel_mult').value = 1.0;"
"      document.getElementById('editor_easing').value = 1;"
"      document.getElementById('editor_approach').value = 0;"
"      document.getElementById('editor_overshoot').value = 0;"
"      document.getElementById('editor_precision').checked = false;"
"    }"
"  }).catch(e => {"
"    console.error('Failed to load preset:', e);"
"    alert('Failed to load preset data');"
"  });"
"}"
"function closePresetEditor() {"
"  document.getElementById('preset_editor_modal').style.display = 'none';"
"  currentEditingPreset = -1;"
"}"
"function savePresetEditor() {"
"  if (currentEditingPreset < 0) return;"
"  const preset = {"
"    index: currentEditingPreset,"
"    pos: ["
"      parseFloat(document.getElementById('editor_pos_pan').value) || 0,"
"      parseFloat(document.getElementById('editor_pos_tilt').value) || 0,"
"      parseFloat(document.getElementById('editor_pos_zoom').value) || 0"
"    ],"
"    duration_s: parseFloat(document.getElementById('editor_duration').value) || 0,"
"    max_speed_scale: parseFloat(document.getElementById('editor_speed_scale').value) || 0,"
"    speed_multiplier: parseFloat(document.getElementById('editor_speed_mult').value) || 1.0,"
"    accel_multiplier: parseFloat(document.getElementById('editor_accel_mult').value) || 1.0,"
"    easing_type: parseInt(document.getElementById('editor_easing').value) || 0,"
"    approach_mode: parseInt(document.getElementById('editor_approach').value) || 0,"
"    arrival_overshoot: parseFloat(document.getElementById('editor_overshoot').value) || 0,"
"    precision_preferred: document.getElementById('editor_precision').checked,"
"    valid: true"
"  };"
"  fetch('/api/preset/update', {"
"    method: 'POST',"
"    headers: { 'Content-Type': 'application/json' },"
"    body: JSON.stringify(preset)"
"  }).then(r => r.json()).then(data => {"
"    if (data.status === 'ok') {"
"      alert('Preset ' + currentEditingPreset + ' updated!');"
"      closePresetEditor();"
"    } else {"
"      alert('Error: ' + (data.error || 'Failed to update preset'));"
"    }"
"  }).catch(e => {"
"    console.error('Update preset failed:', e);"
"    alert('Failed to update preset');"
"  });"
"}"
"window.onclick = function(event) {"
"  const modal = document.getElementById('preset_editor_modal');"
"  if (event.target == modal) {"
"    closePresetEditor();"
"  }"
"};"
"function uploadFirmware() {"
"  const fileInput = document.getElementById('firmware_file');"
"  const statusDiv = document.getElementById('ota_status');"
"  if (!fileInput || !fileInput.files || fileInput.files.length === 0) {"
"    if (statusDiv) {"
"      statusDiv.style.display = 'block';"
"      statusDiv.className = 'status status-info';"
"      statusDiv.textContent = 'Please select a firmware file first';"
"    }"
"    return;"
"  }"
"  const file = fileInput.files[0];"
"  if (statusDiv) {"
"    statusDiv.style.display = 'block';"
"    statusDiv.className = 'status status-info';"
"    statusDiv.textContent = 'Uploading firmware (' + (file.size / 1024 / 1024).toFixed(2) + ' MB)...';"
"  }"
"  fetch('/api/update', {"
"    method: 'POST',"
"    headers: { 'Content-Type': 'application/octet-stream' },"
"    body: file"
"  }).then(response => {"
"    if (!response.ok) {"
"      throw new Error('HTTP ' + response.status);"
"    }"
"    return response.json();"
"  }).then(data => {"
"    if (statusDiv) {"
"      if (data.status === 'ok') {"
"        statusDiv.className = 'status status-success';"
"        statusDiv.textContent = 'Firmware uploaded successfully! Device will reboot in a few seconds...';"
"        setTimeout(() => {"
"          if (statusDiv) {"
"            statusDiv.textContent = 'Device rebooting. Please wait and refresh the page in 30 seconds.';"
"          }"
"        }, 2000);"
"      } else {"
"        statusDiv.className = 'status status-info';"
"        statusDiv.style.background = '#ffebee';"
"        statusDiv.style.color = '#c62828';"
"        statusDiv.textContent = 'Error: ' + (data.error || 'Upload failed');"
"      }"
"    }"
"  }).catch(e => {"
"    console.error('Upload failed:', e);"
"    if (statusDiv) {"
"      statusDiv.className = 'status status-info';"
"      statusDiv.style.background = '#ffebee';"
"      statusDiv.style.color = '#c62828';"
"      statusDiv.textContent = 'Upload failed: ' + e.message;"
"    }"
"  });"
"}"
"// Initialize on page load"
"setTimeout(function() {"
"  try {"
"    updatePositions();"
"    updatePosInterval = setInterval(updatePositions, 500);"
"    createPresetButtons();"
"  } catch (e) {"
"    console.error('Initialization error:', e);"
"  }"
"}, 100);"
"</script>"
"</body></html>";

// Handler for root path - serve HTML page
static esp_err_t root_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    
    // Format slider inputs using macro values
    char slider_pan[128], slider_tilt[128], slider_zoom[128];
    int pan_step = (MAX_VELOCITY_PAN >= 100.0f) ? 5 : 1;
    int tilt_step = (MAX_VELOCITY_TILT >= 100.0f) ? 5 : 1;
    
    snprintf(slider_pan, sizeof(slider_pan), 
             "<input type=\"range\" id=\"pan_vel\" min=\"-%.0f\" max=\"%.0f\" value=\"0\" step=\"%d\">",
             MAX_VELOCITY_PAN, MAX_VELOCITY_PAN, pan_step);
    snprintf(slider_tilt, sizeof(slider_tilt),
             "<input type=\"range\" id=\"tilt_vel\" min=\"-%.0f\" max=\"%.0f\" value=\"0\" step=\"%d\">",
             MAX_VELOCITY_TILT, MAX_VELOCITY_TILT, tilt_step);
    snprintf(slider_zoom, sizeof(slider_zoom),
             "<input type=\"range\" id=\"zoom_vel\" min=\"-%.0f\" max=\"%.0f\" value=\"0\" step=\"1\">",
             MAX_VELOCITY_ZOOM, MAX_VELOCITY_ZOOM);
    
    // Replace placeholders in HTML
    // Calculate buffer size needed (original size + slider strings - placeholder strings)
    size_t html_len = strlen(html_page);
    size_t slider_pan_len = strlen(slider_pan);
    size_t slider_tilt_len = strlen(slider_tilt);
    size_t slider_zoom_len = strlen(slider_zoom);
    size_t placeholder_len = strlen("%SLIDER_PAN%") + strlen("%SLIDER_TILT%") + strlen("%SLIDER_ZOOM%");
    size_t buffer_size = html_len - placeholder_len + slider_pan_len + slider_tilt_len + slider_zoom_len + 1;
    
    char *html_buffer = malloc(buffer_size);
    if (html_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for HTML");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    // Replace placeholders using simple string replacement
    char *html_work = (char *)html_page;
    char *output = html_buffer;
    const char *placeholder_pan = "%SLIDER_PAN%";
    const char *placeholder_tilt = "%SLIDER_TILT%";
    const char *placeholder_zoom = "%SLIDER_ZOOM%";
    
    // Replace %SLIDER_PAN%
    char *pos = strstr(html_work, placeholder_pan);
    if (pos) {
        size_t len = pos - html_work;
        memcpy(output, html_work, len);
        output += len;
        memcpy(output, slider_pan, slider_pan_len);
        output += slider_pan_len;
        html_work = pos + strlen(placeholder_pan);
    } else {
        free(html_buffer);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    // Replace %SLIDER_TILT%
    pos = strstr(html_work, placeholder_tilt);
    if (pos) {
        size_t len = pos - html_work;
        memcpy(output, html_work, len);
        output += len;
        memcpy(output, slider_tilt, slider_tilt_len);
        output += slider_tilt_len;
        html_work = pos + strlen(placeholder_tilt);
    } else {
        free(html_buffer);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    // Replace %SLIDER_ZOOM%
    pos = strstr(html_work, placeholder_zoom);
    if (pos) {
        size_t len = pos - html_work;
        memcpy(output, html_work, len);
        output += len;
        memcpy(output, slider_zoom, slider_zoom_len);
        output += slider_zoom_len;
        html_work = pos + strlen(placeholder_zoom);
    } else {
        free(html_buffer);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    // Copy remaining suffix
    size_t suffix_len = strlen(html_work);
    memcpy(output, html_work, suffix_len);
    output += suffix_len;
    *output = '\0';
    
    httpd_resp_send(req, html_buffer, HTTPD_RESP_USE_STRLEN);
    free(html_buffer);
    return ESP_OK;
}

// Handler for /api/positions - GET current positions
static esp_err_t api_positions_handler(httpd_req_t *req) {
    float positions[3];
    motion_controller_get_positions(positions);
    
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "pan", positions[0]);
    cJSON_AddNumberToObject(json, "tilt", positions[1]);
    cJSON_AddNumberToObject(json, "zoom", positions[2]);
    
    char *json_string = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));
    
    free(json_string);
    cJSON_Delete(json);
    return ESP_OK;
}

// Handler for /api/velocity - POST set velocities
static esp_err_t api_velocity_handler(httpd_req_t *req) {
    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    float velocities[3] = {0, 0, 0};
    cJSON *pan = cJSON_GetObjectItem(json, "pan");
    cJSON *tilt = cJSON_GetObjectItem(json, "tilt");
    cJSON *zoom = cJSON_GetObjectItem(json, "zoom");
    
    if (pan) velocities[0] = (float)pan->valuedouble;
    if (tilt) velocities[1] = (float)tilt->valuedouble;
    if (zoom) velocities[2] = (float)zoom->valuedouble;
    
    motion_controller_set_velocities(velocities);
    
    cJSON_Delete(json);
    
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "ok");
    char *response_str = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_str, strlen(response_str));
    free(response_str);
    cJSON_Delete(response);
    
    return ESP_OK;
}

// Handler for /api/command - POST command
static esp_err_t api_command_handler(httpd_req_t *req) {
    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *cmd = cJSON_GetObjectItem(json, "command");
    if (cmd == NULL || !cJSON_IsString(cmd)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid command field");
        return ESP_FAIL;
    }
    
    const char *command = cmd->valuestring;
    bool success = false;
    
    if (strcmp(command, "home") == 0) {
        success = motion_controller_home(255);  // Home all axes
    } else if (strcmp(command, "stop") == 0) {
        motion_controller_stop();
        success = true;
    } else if (strcmp(command, "precision") == 0) {
        // Toggle precision mode - we'd need to track state or query it
        motion_controller_set_precision_mode(true);  // For now, just enable
        success = true;
    }
    
    cJSON_Delete(json);
    
    cJSON *response = cJSON_CreateObject();
    if (success) {
        cJSON_AddStringToObject(response, "status", "ok");
    } else {
        cJSON_AddStringToObject(response, "status", "error");
        cJSON_AddStringToObject(response, "error", "Command failed");
    }
    
    char *response_str = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_str, strlen(response_str));
    free(response_str);
    cJSON_Delete(response);
    
    return ESP_OK;
}

// Handler for /api/preset/goto - POST goto preset
static esp_err_t api_preset_goto_handler(httpd_req_t *req) {
    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *idx = cJSON_GetObjectItem(json, "index");
    if (idx == NULL || !cJSON_IsNumber(idx)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid index field");
        return ESP_FAIL;
    }
    
    uint8_t preset_idx = (uint8_t)idx->valueint;
    bool success = motion_controller_goto_preset(preset_idx);
    
    cJSON_Delete(json);
    
    cJSON *response = cJSON_CreateObject();
    if (success) {
        cJSON_AddStringToObject(response, "status", "ok");
    } else {
        cJSON_AddStringToObject(response, "status", "error");
        cJSON_AddStringToObject(response, "error", "Failed to move to preset");
    }
    
    char *response_str = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_str, strlen(response_str));
    free(response_str);
    cJSON_Delete(response);
    
    return ESP_OK;
}

// Handler for /api/preset/save - POST save preset
static esp_err_t api_preset_save_handler(httpd_req_t *req) {
    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *idx = cJSON_GetObjectItem(json, "index");
    if (idx == NULL || !cJSON_IsNumber(idx)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid index field");
        return ESP_FAIL;
    }
    
    uint8_t preset_idx = (uint8_t)idx->valueint;
    bool success = motion_controller_save_preset(preset_idx);
    
    cJSON_Delete(json);
    
    cJSON *response = cJSON_CreateObject();
    if (success) {
        cJSON_AddStringToObject(response, "status", "ok");
    } else {
        cJSON_AddStringToObject(response, "status", "error");
        cJSON_AddStringToObject(response, "error", "Failed to save preset");
    }
    
    char *response_str = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_str, strlen(response_str));
    free(response_str);
    cJSON_Delete(response);
    
    return ESP_OK;
}

// Handler for /api/preset/get - GET preset data
static esp_err_t api_preset_get_handler(httpd_req_t *req) {
    char query[64];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing query parameter");
        return ESP_FAIL;
    }
    
    char index_str[8];
    if (httpd_query_key_value(query, "index", index_str, sizeof(index_str)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing index parameter");
        return ESP_FAIL;
    }
    
    uint8_t preset_idx = (uint8_t)atoi(index_str);
    preset_t preset;
    bool success = motion_controller_get_preset(preset_idx, &preset);
    
    cJSON *response = cJSON_CreateObject();
    if (success && preset.valid) {
        cJSON_AddStringToObject(response, "status", "ok");
        cJSON *preset_json = cJSON_CreateObject();
        
        // Add position array
        cJSON *pos_array = cJSON_CreateArray();
        for (int i = 0; i < 3; i++) {
            cJSON_AddItemToArray(pos_array, cJSON_CreateNumber(preset.pos[i]));
        }
        cJSON_AddItemToObject(preset_json, "pos", pos_array);
        
        cJSON_AddNumberToObject(preset_json, "duration_s", preset.duration_s);
        cJSON_AddNumberToObject(preset_json, "max_speed_scale", preset.max_speed_scale);
        cJSON_AddNumberToObject(preset_json, "speed_multiplier", preset.speed_multiplier);
        cJSON_AddNumberToObject(preset_json, "accel_multiplier", preset.accel_multiplier);
        cJSON_AddNumberToObject(preset_json, "easing_type", preset.easing_type);
        cJSON_AddNumberToObject(preset_json, "approach_mode", preset.approach_mode);
        cJSON_AddNumberToObject(preset_json, "arrival_overshoot", preset.arrival_overshoot);
        cJSON_AddBoolToObject(preset_json, "precision_preferred", preset.precision_preferred);
        cJSON_AddBoolToObject(preset_json, "valid", preset.valid);
        
        cJSON_AddItemToObject(response, "preset", preset_json);
    } else {
        cJSON_AddStringToObject(response, "status", "not_found");
    }
    
    char *response_str = cJSON_Print(response);
    if (response_str == NULL) {
        cJSON_Delete(response);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, response_str, strlen(response_str));
    free(response_str);
    cJSON_Delete(response);
    
    // Ignore connection reset errors (client disconnected)
    if (ret == ESP_ERR_HTTPD_RESP_SEND) {
        return ESP_OK;
    }
    
    return ret;
}

// Handler for /api/preset/update - POST update preset
static esp_err_t api_preset_update_handler(httpd_req_t *req) {
    char content[512];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *idx = cJSON_GetObjectItem(json, "index");
    if (idx == NULL || !cJSON_IsNumber(idx)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid index field");
        return ESP_FAIL;
    }
    
    uint8_t preset_idx = (uint8_t)idx->valueint;
    preset_t preset;
    preset_init_default(&preset);
    
    // Parse position array
    cJSON *pos_array = cJSON_GetObjectItem(json, "pos");
    if (pos_array != NULL && cJSON_IsArray(pos_array)) {
        int array_size = cJSON_GetArraySize(pos_array);
        for (int i = 0; i < array_size && i < 3; i++) {
            cJSON *item = cJSON_GetArrayItem(pos_array, i);
            if (cJSON_IsNumber(item)) {
                preset.pos[i] = (float)item->valuedouble;
            }
        }
    }
    
    // Parse other fields
    cJSON *item;
    if ((item = cJSON_GetObjectItem(json, "duration_s")) != NULL && cJSON_IsNumber(item)) {
        preset.duration_s = (float)item->valuedouble;
    }
    if ((item = cJSON_GetObjectItem(json, "max_speed_scale")) != NULL && cJSON_IsNumber(item)) {
        preset.max_speed_scale = (float)item->valuedouble;
    }
    if ((item = cJSON_GetObjectItem(json, "speed_multiplier")) != NULL && cJSON_IsNumber(item)) {
        preset.speed_multiplier = (float)item->valuedouble;
    }
    if ((item = cJSON_GetObjectItem(json, "accel_multiplier")) != NULL && cJSON_IsNumber(item)) {
        preset.accel_multiplier = (float)item->valuedouble;
    }
    if ((item = cJSON_GetObjectItem(json, "easing_type")) != NULL && cJSON_IsNumber(item)) {
        preset.easing_type = (easing_type_t)item->valueint;
    }
    if ((item = cJSON_GetObjectItem(json, "approach_mode")) != NULL && cJSON_IsNumber(item)) {
        preset.approach_mode = (approach_mode_t)item->valueint;
    }
    if ((item = cJSON_GetObjectItem(json, "arrival_overshoot")) != NULL && cJSON_IsNumber(item)) {
        preset.arrival_overshoot = (float)item->valuedouble;
    }
    if ((item = cJSON_GetObjectItem(json, "precision_preferred")) != NULL && cJSON_IsBool(item)) {
        preset.precision_preferred = cJSON_IsTrue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "valid")) != NULL && cJSON_IsBool(item)) {
        preset.valid = cJSON_IsTrue(item);
    }
    
    cJSON_Delete(json);
    
    bool success = motion_controller_update_preset(preset_idx, &preset);
    
    cJSON *response = cJSON_CreateObject();
    if (success) {
        cJSON_AddStringToObject(response, "status", "ok");
    } else {
        cJSON_AddStringToObject(response, "status", "error");
        cJSON_AddStringToObject(response, "error", "Failed to update preset");
    }
    
    char *response_str = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_str, strlen(response_str));
    free(response_str);
    cJSON_Delete(response);
    
    return ESP_OK;
}

// Handler for /api/update - POST firmware update (OTA)
static esp_err_t api_update_handler(httpd_req_t *req) {
    esp_ota_handle_t ota_handle = 0;
    const esp_partition_t *update_partition = NULL;
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    
    // Check if OTA is already in progress
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            ESP_LOGW(TAG, "OTA update pending verification");
            cJSON *response = cJSON_CreateObject();
            cJSON_AddStringToObject(response, "status", "error");
            cJSON_AddStringToObject(response, "error", "OTA update pending verification. Please reboot.");
            char *response_str = cJSON_Print(response);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_send(req, response_str, strlen(response_str));
            free(response_str);
            cJSON_Delete(response);
            return ESP_OK;
        }
    }
    
    // Find next OTA partition
    update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(TAG, "No OTA partition found");
        cJSON *response = cJSON_CreateObject();
        cJSON_AddStringToObject(response, "status", "error");
        cJSON_AddStringToObject(response, "error", "No OTA partition available");
        char *response_str = cJSON_Print(response);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, response_str, strlen(response_str));
        free(response_str);
        cJSON_Delete(response);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Starting OTA update on partition: %s", update_partition->label);
    
    // Begin OTA update
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        cJSON *response = cJSON_CreateObject();
        cJSON_AddStringToObject(response, "status", "error");
        cJSON_AddStringToObject(response, "error", "Failed to begin OTA update");
        char *response_str = cJSON_Print(response);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, response_str, strlen(response_str));
        free(response_str);
        cJSON_Delete(response);
        return ESP_FAIL;
    }
    
    // Receive firmware data
    char *buf = malloc(1024);
    if (buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer");
        esp_ota_abort(ota_handle);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    int total_len = req->content_len;
    int received = 0;
    int content_received = 0;
    bool content_length_known = (total_len > 0);
    
    ESP_LOGI(TAG, "Receiving firmware update%s", content_length_known ? 
             " (size unknown, streaming)" : "");
    if (content_length_known) {
        ESP_LOGI(TAG, "Expected size: %d bytes", total_len);
    }
    
    // Receive firmware data
    // If content_len is 0, we'll receive until connection closes (chunked transfer)
    while (!content_length_known || content_received < total_len) {
        int recv_len = httpd_req_recv(req, buf, 1024);
        if (recv_len < 0) {
            if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            // If we don't know the content length and recv_len is 0, we're done
            if (!content_length_known && recv_len == 0) {
                break;
            }
            ESP_LOGE(TAG, "OTA receive error: %d", recv_len);
            free(buf);
            esp_ota_abort(ota_handle);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        
        if (recv_len == 0) {
            // Connection closed, we're done
            if (!content_length_known) {
                break;
            }
            // If we expected more data, this is an error
            if (content_received < total_len) {
                ESP_LOGE(TAG, "OTA receive incomplete: %d / %d bytes", content_received, total_len);
                free(buf);
                esp_ota_abort(ota_handle);
                httpd_resp_send_500(req);
                return ESP_FAIL;
            }
            break;
        }
        
        err = esp_ota_write(ota_handle, buf, recv_len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            free(buf);
            esp_ota_abort(ota_handle);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        content_received += recv_len;
        received += recv_len;
        
        // Log progress every 64KB
        if (received % 65536 < 1024) {
            if (content_length_known) {
                ESP_LOGI(TAG, "OTA progress: %d / %d bytes (%.1f%%)", 
                        received, total_len, (received * 100.0f) / total_len);
            } else {
                ESP_LOGI(TAG, "OTA progress: %d bytes received", received);
            }
        }
    }
    
    ESP_LOGI(TAG, "OTA data reception complete: %d bytes", received);
    
    free(buf);
    
    // Finish OTA update
    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "OTA validation failed");
            cJSON *response = cJSON_CreateObject();
            cJSON_AddStringToObject(response, "status", "error");
            cJSON_AddStringToObject(response, "error", "Firmware validation failed");
            char *response_str = cJSON_Print(response);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_send(req, response_str, strlen(response_str));
            free(response_str);
            cJSON_Delete(response);
            return ESP_FAIL;
        } else {
            ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
    }
    
    // Set boot partition
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "OTA update completed successfully. Firmware will be active after reboot.");
    
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "ok");
    cJSON_AddStringToObject(response, "message", "OTA update completed. Device will reboot.");
    char *response_str = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_str, strlen(response_str));
    free(response_str);
    cJSON_Delete(response);
    
    // Reboot after a short delay to allow response to be sent
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    
    return ESP_OK;
}

bool http_server_start(void) {
    // If server is already running, return success
    if (server_handle != NULL) {
        ESP_LOGI(TAG, "HTTP server already running");
        return true;
    }
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 10;
    
    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);
    
    if (httpd_start(&server_handle, &config) == ESP_OK) {
        // Register URI handlers
        httpd_uri_t root_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_handler,
        };
        httpd_register_uri_handler(server_handle, &root_uri);
        
        httpd_uri_t positions_uri = {
            .uri = "/api/positions",
            .method = HTTP_GET,
            .handler = api_positions_handler,
        };
        httpd_register_uri_handler(server_handle, &positions_uri);
        
        httpd_uri_t velocity_uri = {
            .uri = "/api/velocity",
            .method = HTTP_POST,
            .handler = api_velocity_handler,
        };
        httpd_register_uri_handler(server_handle, &velocity_uri);
        
        httpd_uri_t command_uri = {
            .uri = "/api/command",
            .method = HTTP_POST,
            .handler = api_command_handler,
        };
        httpd_register_uri_handler(server_handle, &command_uri);
        
        httpd_uri_t preset_goto_uri = {
            .uri = "/api/preset/goto",
            .method = HTTP_POST,
            .handler = api_preset_goto_handler,
        };
        httpd_register_uri_handler(server_handle, &preset_goto_uri);
        
        httpd_uri_t preset_save_uri = {
            .uri = "/api/preset/save",
            .method = HTTP_POST,
            .handler = api_preset_save_handler,
        };
        httpd_register_uri_handler(server_handle, &preset_save_uri);
        
        httpd_uri_t preset_get_uri = {
            .uri = "/api/preset/get",
            .method = HTTP_GET,
            .handler = api_preset_get_handler,
        };
        httpd_register_uri_handler(server_handle, &preset_get_uri);
        
        httpd_uri_t preset_update_uri = {
            .uri = "/api/preset/update",
            .method = HTTP_POST,
            .handler = api_preset_update_handler,
        };
        httpd_register_uri_handler(server_handle, &preset_update_uri);
        
        httpd_uri_t update_uri = {
            .uri = "/api/update",
            .method = HTTP_POST,
            .handler = api_update_handler,
        };
        httpd_register_uri_handler(server_handle, &update_uri);
        
        ESP_LOGI(TAG, "HTTP server started");
        return true;
    }
    
    ESP_LOGE(TAG, "Failed to start HTTP server");
    return false;
}

void http_server_stop(void) {
    if (server_handle) {
        httpd_stop(server_handle);
        server_handle = NULL;
        ESP_LOGI(TAG, "HTTP server stopped");
    }
}


